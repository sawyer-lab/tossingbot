#!/usr/bin/env python
import rospy
import numpy as np
import rospkg
from geometry_msgs.msg import Point, Quaternion
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState

# --- MODULE IMPORTS ---
from tossingbot import config as cfg
from tossingbot.perception.ros_camera import RosCamera
from tossingbot.perception.vision import VisionProcessor
from tossingbot.hardware.sawyer import SawyerInterface, RobotCommand, ControlMode
from tossingbot.hardware.gripper import GripperInterface
from tossingbot.planning.kinematics import CasadiKinematics
from tossingbot.planning.casadi_planner import CasadiPlanner
from tossingbot.planning.orientation_helper import RotationPrimitive
from tossingbot.environment.health_monitor import HealthMonitor
from tossingbot.tossing.motion_planner import TossingPlanner

class SimInterface:
    def __init__(self):
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_state_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.get_state_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        # self.object_names = ['I_shape', 'L_shape', 'T_shape']
        self.object_names = ['bar' , 'cross', 'cylinder']
        self.anchor_poses = {}
        self.picked_objects = set()  # Track which objects have been picked
        
    def spawn_new_problem(self):
        """Randomizes object locations with collision avoidance."""
        self.anchor_poses = {}
        self.picked_objects = set()  # Reset picked objects tracking
        margin = 0.05
        min_dist = 0.12  # Minimum spacing between objects (12cm)
        
        min_x, max_x = cfg.ROI_X[0] + margin, cfg.ROI_X[1] - margin
        min_y, max_y = cfg.ROI_Y[0] + margin, cfg.ROI_Y[1] - margin

        for obj in self.object_names:
            valid_pos = False
            attempts = 0
            
            # Rejection Sampling: Try up to 20 times to find a free spot
            while not valid_pos and attempts < 20:
                rand_x = np.random.uniform(min_x, max_x)
                rand_y = np.random.uniform(min_y, max_y)
                
                # Check distance against all currently placed objects
                collision = False
                for other_obj, pose_data in self.anchor_poses.items():
                    other_pos = pose_data['pos']
                    dist = np.sqrt((rand_x - other_pos.x)**2 + (rand_y - other_pos.y)**2)
                    if dist < min_dist:
                        collision = True
                        break
                
                if not collision:
                    valid_pos = True
                    q = RotationPrimitive.get_random_flat_quaternion()
                    # Spawn slightly higher (0.85) so they drop naturally
                    self.anchor_poses[obj] = {
                        'pos': Point(rand_x, rand_y, 0.75),
                        'ori': Quaternion(*q)
                    }
                attempts += 1
            
            if not valid_pos:
                rospy.logwarn(f"Could not find free space for {obj}, skipping...")

        self._apply_anchor()

    def reset_to_anchor(self):
        if not self.anchor_poses: self.spawn_new_problem()
        else: self._apply_anchor()

    def _apply_anchor(self):
        # 1. Teleport objects (except already picked ones)
        for obj, pose in self.anchor_poses.items():
            if obj in self.picked_objects:
                continue  # Skip already picked objects
            msg = ModelState()
            msg.model_name = obj
            msg.reference_frame = "world"
            msg.pose.position = pose['pos']
            msg.pose.orientation = pose['ori']
            
            # Kill momentum so they don't fly away
            msg.twist.linear.x = 0; msg.twist.linear.y = 0; msg.twist.linear.z = 0
            msg.twist.angular.x = 0; msg.twist.angular.y = 0; msg.twist.angular.z = 0
            
            try: self.set_state_srv(msg)
            except rospy.ServiceException: pass
            
        # 2. WAIT for physics to settle (Increased sleep)
        # 2.0 seconds is usually enough for objects to fall and stop jittering
        rospy.loginfo("Waiting for simulation to settle...")
        # rospy.sleep(2.0)

    def check_success(self):
        """
        Checks if any object was successfully picked.
        Returns: (success: bool, picked_object: str or None)
        """
        for obj in self.object_names:
            if obj in self.picked_objects:
                continue  # Skip already picked objects
            try:
                resp = self.get_state_srv(obj, "world")
                # Check if lifted above table surface
                # TABLE_HEIGHT = 0.75, we lift by SAFE_LIFT_HEIGHT = 0.15
                # So successful grasp should be at least 0.75 + 0.10 = 0.85m
                if resp.pose.position.z > cfg.TABLE_HEIGHT + 0.10:
                    self.picked_objects.add(obj)
                    rospy.loginfo(f"PICKED: {obj} ({len(self.picked_objects)}/{len(self.anchor_poses)})")
                    return True, obj
            except: pass
        return False, None
    
    def remove_picked_object(self, obj_name):
        """Teleport picked object far away (out of workspace)."""
        msg = ModelState()
        msg.model_name = obj_name
        msg.reference_frame = "world"
        msg.pose.position = Point(5.0, 5.0, 5.0)  # Far away
        msg.pose.orientation = Quaternion(0, 0, 0, 1)
        msg.twist.linear.x = 0; msg.twist.linear.y = 0; msg.twist.linear.z = 0
        msg.twist.angular.x = 0; msg.twist.angular.y = 0; msg.twist.angular.z = 0
        try:
            self.set_state_srv(msg)
        except rospy.ServiceException:
            pass
    
    def all_objects_picked(self):
        """Check if all objects in the scene have been picked."""
        return len(self.picked_objects) >= len(self.anchor_poses)

class TossingEnv:
    def __init__(self):
        if rospy.get_node_uri() is None: rospy.init_node('tossing_env')
        
        # Health Monitoring
        self.health = HealthMonitor()
        
        # Hardware & Perception
        self.camera = RosCamera()
        self.vision = VisionProcessor()
        self.robot = SawyerInterface()
        self.gripper = GripperInterface()
        self.sim = SimInterface()
        
        # Planning
        rp = rospkg.RosPack()
        urdf_path = rp.get_path('grasping') + "/sawyer_model.urdf"
        self.kinematics = CasadiKinematics(urdf_path, "base", "right_gripper_tip")
        self.planner = CasadiPlanner(self.kinematics) # Config injected automatically
        self.rot_helper = RotationPrimitive(num_rotations=cfg.NUM_ROTATIONS, total_deg=cfg.TOTAL_DEG)
        self.tossing_planner = TossingPlanner()

        # State Tracking
        self.max_steps = 50
        self.step_count = 0
        self.recovery_attempts = 0
        self.max_recovery_attempts = 3
        
        # Tossing release timer
        self.release_timer = None

        rospy.loginfo("Waiting for Camera Data...")
        while self.camera.get_latest_cloud()[0] is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

    def get_observation(self):
        """Returns the heightmap tensor (State)."""
        pts, cols = self.camera.get_latest_cloud()
        if pts is None: return None
        return self.vision.process(pts, cols)

    def reset(self, previous_success=False, force_new=False):
        """
        Handles the logic of 'New Scene' vs 'Retry Scene'.
        Now continues with same scene until all objects picked or max steps reached.
        Returns: (observation, is_new_scene_bool)
        """
        self.gripper.open()
        self.step_count += 1
        
        new_scene = False
        all_picked = self.sim.all_objects_picked()
        
        # Generate new scene if: forced, all objects picked, or max steps reached
        if force_new or all_picked or self.step_count >= self.max_steps:
            if all_picked:
                rospy.loginfo(f"ALL OBJECTS PICKED! Generating new scene after {self.step_count} steps.")
            elif self.step_count >= self.max_steps:
                rospy.loginfo(f"MAX STEPS ({self.max_steps}) REACHED. Picked {len(self.sim.picked_objects)}/{len(self.sim.anchor_poses)} objects.")
            else:
                rospy.loginfo("GENERATING NEW SCENE...")
            
            self.sim.spawn_new_problem()
            self.step_count = 0
            new_scene = True
            self.robot.move_to_joint_positions(cfg.NEUTRAL_JOINT_POS)
        else:
            # Retry same scene (reset unpicked objects to anchor positions)
            self.sim.reset_to_anchor()

        rospy.sleep(1.5)
        return self.get_observation(), new_scene

    def step(self, u, v, rot_idx):
        """
        Executes the grasp with health monitoring and recovery.
        Args:
            u, v: Pixel coordinates in the heightmap
            rot_idx: Rotation index (0..NUM_ROTATIONS)
        Returns: 
            reward (float): 1.0 for success, 0.0 for fail
        """
        # Mark step start for timeout detection
        self.health.mark_step_start()
        
        # Check health before attempting grasp
        is_healthy, reason = self.health.is_healthy()
        if not is_healthy:
            rospy.logwarn(f"Environment unhealthy before step: {reason}")
            if not self._attempt_recovery():
                return 0.0
        
        try:
            # 1. Convert Pixel to World
            target_pos = self.vision.pixel_to_world(u, v)
            
            # 2. Get Orientation
            quat_msg = self.rot_helper.get_quaternion(rot_idx)
            target_quat = [quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w]
            
            # 3. Define Waypoints
            hover_pos = target_pos.copy()
            hover_pos[2] += cfg.SAFE_LIFT_HEIGHT
            
            q_curr = self.robot.get_joint_positions()

            # 4. Execute Primitive: Hover -> Down -> Close -> Up
            # A. Hover
            ik_hover = self.planner.compute_inverse_kinematics(q_curr, hover_pos, target_quat)
            if ik_hover is None: return 0.0
            self.robot.move_to_joint_positions(ik_hover, timeout=2.0)

            # B. Down
            path_down = self.planner.plan_cartesian(self.robot.get_joint_positions(), target_pos, target_quat, duration=1.5)
            if not self._execute_trajectory(path_down): return 0.0
            
            # C. Grasp
            self.gripper.close()
            rospy.sleep(0.3)
            
            # D. Lift
            path_up = self.planner.plan_cartesian(self.robot.get_joint_positions(), hover_pos, target_quat, duration=1.5)
            self._execute_trajectory(path_up)

         
            # Execute tossing trajectory with gripper release
            sol = self.tossing_planner.get_trajectory(1.0)
            release_index = sol['index']
            dt = 0.01  # From TRAJECTORY_CONFIG
            release_time = release_index * dt
            
            rospy.loginfo(f"Executing toss: release at index={release_index}, time={release_time:.3f}s")
            
            traj = self.tossing_planner.map_to_7dof(sol['Q'], sol['Qd'], sol['Qdd'], 0.0)
            self._execute_trajectory(traj, gripper_release_time=release_time)


            # E. Wait for physics to settle before checking success
            rospy.sleep(0.3)

            
            
            grasp_width = self.gripper.get_current_position()
            is_holding = self.gripper.is_grasping()
            
            # Check if we actually picked an object from the scene
            success, picked_obj = self.sim.check_success()
            
            # Reward Logic: Need BOTH gripper holding AND object lifted
            # This prevents false positives (grasping table/air)
            if success and is_holding:
                reward = 1.0
                print(f"GRASP SUCCESS! Picked {picked_obj} (Width: {grasp_width:.4f}m)")
                # Remove the picked object from the scene
                self.sim.remove_picked_object(picked_obj)
                self.recovery_attempts = 0  # Reset recovery counter on success
            else:
                reward = 0.0
                if is_holding and not success:
                    print(f"GRASP FAILED - holding something but no object lifted (Width: {grasp_width:.4f}m)")
                else:
                    print(f"GRASP FAILED (Width: {grasp_width:.4f}m)")

            return reward
            
        except Exception as e:
            rospy.logerr(f"Exception during grasp execution: {e}")
            # Cancel any pending gripper release on exception
            self._cancel_gripper_release()
            # Attempt recovery on exception
            if not self._attempt_recovery():
                return 0.0
            return 0.0
    
    def _attempt_recovery(self):
        """
        Attempt to recover from environment failure.
        Returns: True if recovered, False if should abort
        """
        self.recovery_attempts += 1
        
        if self.recovery_attempts > self.max_recovery_attempts:
            rospy.logerr("MAX RECOVERY ATTEMPTS REACHED - Manual intervention required")
            return False
        
        rospy.logwarn(f"Attempting recovery (attempt {self.recovery_attempts}/{self.max_recovery_attempts})")
        recovered, reason = self.health.check_and_recover()
        
        if recovered:
            rospy.loginfo(f"Recovery successful: {reason}")
            self.recovery_attempts = 0
            # Re-initialize hardware connections after restart
            rospy.sleep(2.0)
            return True
        else:
            rospy.logerr(f"Recovery failed: {reason}")
            return False

    def _execute_trajectory(self, plan_data, gripper_release_time=None):
        if plan_data is None: return False
        
        # Handle both dict format (from planner) and list format (from map_to_7dof)
        if isinstance(plan_data, dict):
            # Convert dict with Q, Qd, Qdd arrays to list of dicts
            N = plan_data['Q'].shape[0]
            stream = [RobotCommand(
                position=plan_data['Q'][i].tolist(),
                velocity=plan_data['Qd'][i].tolist(),
                acceleration=plan_data['Qdd'][i].tolist()
            ) for i in range(N)]
        else:
            # Original list of dicts format
            stream = [RobotCommand(p['position'], p['velocity'], p['acceleration']) for p in plan_data]
        
        # Schedule gripper release if requested
        if gripper_release_time is not None:
            self._schedule_gripper_release(gripper_release_time)
        
        try:
            result = self.robot.execute_stream(stream, ControlMode.TRAJECTORY)
            return result
        finally:
            # Cancel timer if trajectory finished early or failed
            self._cancel_gripper_release()
    
    def _schedule_gripper_release(self, delay_seconds):
        """Schedule gripper to open after specified delay."""
        # Cancel any existing timer
        self._cancel_gripper_release()
        
        rospy.loginfo(f"Scheduling gripper release in {delay_seconds:.3f}s")
        self.release_timer = rospy.Timer(
            rospy.Duration(delay_seconds),
            self._gripper_release_callback,
            oneshot=True
        )
    
    def _gripper_release_callback(self, event):
        """Timer callback to open gripper during toss."""
        if self.gripper.is_grasping():
            rospy.loginfo("RELEASING GRIPPER during toss")
            self.gripper.open()
            # Optional: log endpoint velocity at release
            try:
                endpoint_state = self.robot.get_endpoint_state()
                if endpoint_state:
                    vel = endpoint_state['linear']
                    speed = np.linalg.norm([vel.x, vel.y, vel.z])
                    rospy.loginfo(f"Release speed: {speed:.3f} m/s")
            except:
                pass
        else:
            rospy.logwarn("Gripper release triggered but not grasping anything")
        
        self.release_timer = None
    
    def _cancel_gripper_release(self):
        """Cancel pending gripper release timer."""
        if self.release_timer is not None:
            self.release_timer.shutdown()
            self.release_timer = None