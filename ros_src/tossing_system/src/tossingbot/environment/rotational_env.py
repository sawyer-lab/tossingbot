#!/usr/bin/env python3.8
import sys
import rospy
import numpy as np
import rospkg
from geometry_msgs.msg import Point, Quaternion, Pose
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState
from intera_core_msgs.msg import IOComponentCommand

# --- MODULAR IMPORTS ---
from tossingbot import config
from tossingbot.perception.ros_camera import RosCamera
from tossingbot.perception.vision import VisionProcessor
from tossingbot.hardware.sawyer import SawyerInterface, RobotCommand, ControlMode
from tossingbot.hardware.gripper import GripperInterface
from tossingbot.planning.kinematics import CasadiKinematics
from tossingbot.planning.casadi_planner import CasadiPlanner
from tossingbot.planning.orientation_helper import RotationPrimitive

# ==============================================================================
# 1. SIMULATION INTERFACE (God Mode)
# ==============================================================================
#!/usr/bin/env python3.8
import sys
import rospy
import numpy as np
import rospkg
from geometry_msgs.msg import Point, Quaternion, Pose
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState
from intera_core_msgs.msg import IOComponentCommand

# --- MODULAR IMPORTS ---
from tossingbot import config
from tossingbot.perception.ros_camera import RosCamera
from tossingbot.perception.vision import VisionProcessor
from tossingbot.hardware.sawyer import SawyerInterface, RobotCommand, ControlMode
from tossingbot.hardware.gripper import GripperInterface
from tossingbot.planning.kinematics import CasadiKinematics
from tossingbot.planning.casadi_planner import CasadiPlanner
from tossingbot.planning.orientation_helper import RotationPrimitive

# ==============================================================================
# 1. SIMULATION INTERFACE (Multi-Object Support)
# ==============================================================================
class SimInterface:
    def __init__(self):
        rospy.wait_for_service('/gazebo/set_model_state')
        rospy.wait_for_service('/gazebo/get_model_state')
        self.set_state_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.get_state_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        
        # Define all interactable objects
        self.object_names = [f"banana_{i}" for i in range(5)] 
        
        # Stores the pose of ALL objects for the current "Problem"
        # Format: { 'banana_0': {'pos': Point, 'ori': Quat}, ... }
        self.anchor_poses = {} 

    def spawn_new_problem(self):
        """
        Generates COMPELTELY NEW random positions for ALL objects.
        Saves them as the 'anchor' for retries.
        """
        self.anchor_poses = {} # Reset storage
        
        # Spawn Safe Margin
        MARGIN = 0.05
        min_x = config.ROI_X[0] + MARGIN
        max_x = config.ROI_X[1] - MARGIN
        min_y = config.ROI_Y[0] + MARGIN
        max_y = config.ROI_Y[1] - MARGIN

        for obj_name in self.object_names:
            # 1. Randomize Orientation
            q_list = RotationPrimitive.get_random_flat_quaternion()
            
            # 2. Randomize Position
            rand_x = np.random.uniform(min_x, max_x)
            rand_y = np.random.uniform(min_y, max_y)
            
            # 3. Store in Anchor Dictionary
            self.anchor_poses[obj_name] = {
                'pos': Point(rand_x, rand_y, 0.78), # Drop height
                'ori': Quaternion(x=q_list[0], y=q_list[1], z=q_list[2], w=q_list[3])
            }
        
        # 4. Apply to Simulation
        self._apply_anchor()

    def reset_to_anchor(self):
        """
        Resets ALL objects to the saved anchor positions.
        Used to retry the exact same scenario.
        """
        if self.anchor_poses:
            self._apply_anchor()
        else:
            self.spawn_new_problem()

    def _apply_anchor(self):
        """Internal helper to teleport all objects in Gazebo."""
        for obj_name, pose_data in self.anchor_poses.items():
            msg = ModelState()
            msg.model_name = obj_name
            msg.reference_frame = "world"
            msg.pose.position = pose_data['pos']
            msg.pose.orientation = pose_data['ori']
            
            # Kill Velocity
            msg.twist.linear.x = 0; msg.twist.linear.y = 0; msg.twist.linear.z = 0
            msg.twist.angular.x = 0; msg.twist.angular.y = 0; msg.twist.angular.z = 0

            try:
                self.set_state_srv(msg)
            except rospy.ServiceException:
                rospy.logerr(f"Failed to set state for {obj_name}")
        
        rospy.sleep(0.5) # Wait for physics to settle once after moving everyone

    def check_success(self):
        """Returns True if ANY object is lifted above 0.90m"""
        for obj_name in self.object_names:
            try:
                resp = self.get_state_srv(obj_name, "world")
                # If any banana is high enough, it's a success
                if resp.pose.position.z > 0.90:
                    rospy.loginfo(f"🍌 SUCCESS: Lifted {obj_name}!")
                    return True
            except rospy.ServiceException:
                pass
        return False

# ==============================================================================
# 2. ROTATIONAL ENVIRONMENT (Standalone)
# ==============================================================================
class RotationalEnv:
    def __init__(self, num_rotations=4, max_steps_per_object=50):
        if rospy.get_node_uri() is None:
            rospy.init_node('rotational_env_node')

        rospy.loginfo("Initializing Rotational Env...")

        # 1. Perception
        self.camera = RosCamera(
            left_topic="/rgbd_camera_left/depth/points",
            right_topic="/rgbd_camera_right/depth/points"
        )
        self.vision = VisionProcessor()

        # 2. Planning
        rp = rospkg.RosPack()
        urdf = rp.get_path('grasping') + "/sawyer_model.urdf"
        self.kinematics = CasadiKinematics(urdf, "base", "right_gripper_tip")
        self.planner = CasadiPlanner(self.kinematics)

        # 3. Hardware
        self.robot = SawyerInterface()
        self.gripper = GripperInterface()
        
        # 4. Sim & Logic
        self.sim = SimInterface()
        self.rot_helper = RotationPrimitive(num_rotations=num_rotations)
        
        # Logic Variables
        self.max_steps = max_steps_per_object
        self.current_steps = 0
        self.NEUTRAL_Q = [0.0, -1.27, 0.0, 2.06, 0.0, 0.0, 0.0] # Elbow Up

        rospy.loginfo("Waiting for Camera...")
        while self.camera.get_latest_cloud()[0] is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
        
        step_size = self.rot_helper.angle_step
        rospy.loginfo(f"Env Ready ({num_rotations} Primitives, Step={step_size:.1f} deg)")

    def get_observation(self):
        """Returns the (3, H, W) tensor of the workspace."""
        pts, cols = self.camera.get_latest_cloud()
        if pts is None: return None
        return self.vision.process(pts, cols)

    def execute_plan(self, plan_data):
        if plan_data is None: return False
        stream = []
        for p in plan_data:
            stream.append(RobotCommand(
                position=p['position'],
                velocity=p['velocity'],
                acceleration=p['acceleration']
            ))
        return self.robot.execute_stream(stream, ControlMode.TRAJECTORY)

    def force_neutral(self):
        """
        Moves the robot to the safe neutral position.
        Call this ONLY once at the start of the script.
        """
        rospy.loginfo("🦾 Moving Arm to Neutral Home Position...")
        self.robot.move_to_joint_positions(self.NEUTRAL_Q)
        self.gripper.open()
        rospy.sleep(0.5)

    def reset_episode(self, previous_success=False, first_run=False):
        """
        Checks object status.
        - If Success/MaxSteps: Generates NEW problem.
        - If Fail: Resets object to ANCHOR (retries same problem).
        """
        # Ensure gripper is open for next attempt
        self.gripper.open()
        
        # Decide to reset Object
        self.current_steps += 1
        
        if first_run:
            rospy.loginfo("🆕 INITIALIZING FIRST EPISODE...")
            self.sim.spawn_new_problem()
            self.current_steps = 0
            
        elif previous_success:
            rospy.loginfo("✅ GRASP SUCCESS! Generating NEW Problem.")
            self.sim.spawn_new_problem()
            self.current_steps = 0
            
        elif self.current_steps >= self.max_steps:
            rospy.loginfo(f"⚠️ MAX STEPS ({self.max_steps}) REACHED. Giving up & Generating NEW Problem.")
            self.sim.spawn_new_problem()
            self.current_steps = 0
            
        else:
            # RETRY MODE:
            # We failed, but we want to retry the SAME scenario.
            # We must reset the object to the anchor to ensure it wasn't knocked away.
            self.sim.reset_to_anchor()
            
        # Return observation from current position
        rospy.sleep(0.2)
        return self.get_observation()

    def step(self, u, v, rot_idx):
        """
        Executes grasp at pixel (u,v) with rotation index.
        Returns: Reward (1.0 or 0.0)
        """
        # 1. Vision: Pixel -> World
        target_pos = self.vision.pixel_to_world(u, v)
        
        # --- Z-HEIGHT CORRECTION ---
        SAFE_GRASP_HEIGHT = 0.00 # 0cm above table
        if target_pos[2] < SAFE_GRASP_HEIGHT:
            target_pos[2] = SAFE_GRASP_HEIGHT

        # Safety Clamp (XY Reach)
        if np.linalg.norm(target_pos[:2]) > 0.90: 
            rospy.logwarn(f"⚠️ TARGET OUT OF REACH: {target_pos}")
            return 0.0

        # 2. Rotation: Index -> Quaternion
        quat_msg = self.rot_helper.get_quaternion(rot_idx)
        target_quat = [quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w]
        
        angle = self.rot_helper.get_angle(rot_idx)

        # Define Waypoints
        hover_pos = target_pos.copy()
        hover_pos[2] += 0.20 # Hover 20cm above grasp
        
        grasp_pos = target_pos.copy()
        q_curr = self.robot.get_joint_positions()

        # --- EXECUTION ---

        # 3. Hover (Fast)
        
        ik_hover = self.planner.compute_inverse_kinematics(
            q_start=q_curr, target_pos=hover_pos, target_quat=target_quat
        )
        if ik_hover is None:
            rospy.logwarn(f"⚠️ IK FAILED (HOVER): Can't reach {hover_pos}")
            return 0.0
        
        self.robot.move_to_joint_positions(ik_hover)

        
        # path_hover = self.planner.plan_cartesian(
        #     q_curr, hover_pos, target_quat=target_quat, 
        #     duration=1.5, check_floor=False
        # )
        
        # if path_hover is None:
        #     rospy.logwarn(f"⚠️ PLANNER FAILED (HOVER): Can't reach {hover_pos}")
        #     return 0.0
        # if not self.execute_plan(path_hover): return 0.0

        # 4. Descend (Slow, Check Floor)
        q_curr = self.robot.get_joint_positions()
        path_down = self.planner.plan_cartesian(
            q_curr, grasp_pos, target_quat=target_quat, 
            duration=2.0, check_floor=False
        )
        
        if path_down is None:
            rospy.logwarn(f"⚠️ PLANNER FAILED (DOWN): Collision likely at {grasp_pos}")
            return 0.0
        if not self.execute_plan(path_down): return 0.0

        # 5. Grasp
        self.gripper.close()
        rospy.sleep(0.2)
        
        # 6. Lift
        q_curr = self.robot.get_joint_positions()
        path_up = self.planner.plan_cartesian(
            q_curr, hover_pos, target_quat=target_quat, 
            duration=2.0, check_floor=False
        )
        if path_up is not None:
             self.execute_plan(path_up)

        # 7. Check Success
        success = self.sim.check_success()
        return 1.0 if success else 0.0

# ==============================================================================
# 2. ROTATIONAL ENVIRONMENT (Standalone)
# ==============================================================================
class RotationalEnv:
    def __init__(self, num_rotations=4, max_steps_per_object=50):
        if rospy.get_node_uri() is None:
            rospy.init_node('rotational_env_node')

        rospy.loginfo("Initializing Rotational Env...")

        # 1. Perception
        self.camera = RosCamera(
            left_topic="/rgbd_camera_left/depth/points",
            right_topic="/rgbd_camera_right/depth/points"
        )
        self.vision = VisionProcessor()

        # 2. Planning
        rp = rospkg.RosPack()
        urdf = rp.get_path('grasping') + "/sawyer_model.urdf"
        self.kinematics = CasadiKinematics(urdf, "base", "right_gripper_tip")
        self.planner = CasadiPlanner(self.kinematics)

        # 3. Hardware
        self.robot = SawyerInterface()
        self.gripper = GripperInterface()
        
        # 4. Sim & Logic
        self.sim = SimInterface()
        self.rot_helper = RotationPrimitive(num_rotations=num_rotations)
        
        # Logic Variables
        self.max_steps = max_steps_per_object
        self.current_steps = 0
        self.NEUTRAL_Q = [0.0, -1.27, 0.0, 2.06, 0.0, 0.0, 0.0] # Elbow Up

        rospy.loginfo("Waiting for Camera...")
        while self.camera.get_latest_cloud()[0] is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
        
        step_size = self.rot_helper.angle_step
        rospy.loginfo(f"Env Ready ({num_rotations} Primitives, Step={step_size:.1f} deg)")

    def get_observation(self):
        """Returns the (3, H, W) tensor of the workspace."""
        pts, cols = self.camera.get_latest_cloud()
        if pts is None: return None
        return self.vision.process(pts, cols)

    def execute_plan(self, plan_data):
        if plan_data is None: return False
        stream = []
        for p in plan_data:
            stream.append(RobotCommand(
                position=p['position'],
                velocity=p['velocity'],
                acceleration=p['acceleration']
            ))
        return self.robot.execute_stream(stream, ControlMode.TRAJECTORY)

    def force_neutral(self):
        """
        Moves the robot to the safe neutral position.
        Call this ONLY once at the start of the script.
        """
        rospy.loginfo("🦾 Moving Arm to Neutral Home Position...")
        self.robot.move_to_joint_positions(self.NEUTRAL_Q)
        self.gripper.open()
        rospy.sleep(0.5)

    def reset_episode(self, previous_success=False, first_run=False):
        """
        Checks object status.
        - If Success/MaxSteps: Generates NEW problem.
        - If Fail: Resets object to ANCHOR (retries same problem).
        """
        # Ensure gripper is open for next attempt
        self.gripper.open()
        
        # Decide to reset Object
        self.current_steps += 1
        
        if first_run:
            rospy.loginfo("🆕 INITIALIZING FIRST EPISODE...")
            self.sim.spawn_new_problem()
            self.current_steps = 0
            
        elif previous_success:
            rospy.loginfo("✅ GRASP SUCCESS! Generating NEW Problem.")
            self.sim.spawn_new_problem()
            self.current_steps = 0
            
        elif self.current_steps >= self.max_steps:
            rospy.loginfo(f"⚠️ MAX STEPS ({self.max_steps}) REACHED. Giving up & Generating NEW Problem.")
            self.sim.spawn_new_problem()
            self.current_steps = 0
            
        else:
            # RETRY MODE:
            # We failed, but we want to retry the SAME scenario.
            # We must reset the object to the anchor to ensure it wasn't knocked away.
            self.sim.reset_to_anchor()
            
        # Return observation from current position
        rospy.sleep(0.2)
        return self.get_observation()

    def step(self, u, v, rot_idx):
        """
        Executes grasp at pixel (u,v) with rotation index.
        Returns: Reward (1.0 or 0.0)
        """
        # 1. Vision: Pixel -> World
        target_pos = self.vision.pixel_to_world(u, v)
        
        # --- Z-HEIGHT CORRECTION ---
        SAFE_GRASP_HEIGHT = 0.00 # 0cm above table
        if target_pos[2] < SAFE_GRASP_HEIGHT:
            target_pos[2] = SAFE_GRASP_HEIGHT

        # Safety Clamp (XY Reach)
        if np.linalg.norm(target_pos[:2]) > 0.90: 
            rospy.logwarn(f"⚠️ TARGET OUT OF REACH: {target_pos}")
            return 0.0

        # 2. Rotation: Index -> Quaternion
        quat_msg = self.rot_helper.get_quaternion(rot_idx)
        target_quat = [quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w]
        
        angle = self.rot_helper.get_angle(rot_idx)

        # Define Waypoints
        hover_pos = target_pos.copy()
        hover_pos[2] += 0.20 # Hover 20cm above grasp
        
        grasp_pos = target_pos.copy()
        q_curr = self.robot.get_joint_positions()

        # --- EXECUTION ---

        # 3. Hover (Fast)
        
        ik_hover = self.planner.compute_inverse_kinematics(
            q_start=q_curr, target_pos=hover_pos, target_quat=target_quat
        )
        if ik_hover is None:
            rospy.logwarn(f"⚠️ IK FAILED (HOVER): Can't reach {hover_pos}")
            return 0.0
        
        self.robot.move_to_joint_positions(ik_hover)

        
        # path_hover = self.planner.plan_cartesian(
        #     q_curr, hover_pos, target_quat=target_quat, 
        #     duration=1.5, check_floor=False
        # )
        
        # if path_hover is None:
        #     rospy.logwarn(f"⚠️ PLANNER FAILED (HOVER): Can't reach {hover_pos}")
        #     return 0.0
        # if not self.execute_plan(path_hover): return 0.0

        # 4. Descend (Slow, Check Floor)
        q_curr = self.robot.get_joint_positions()
        path_down = self.planner.plan_cartesian(
            q_curr, grasp_pos, target_quat=target_quat, 
            duration=2.0, check_floor=False
        )
        
        if path_down is None:
            rospy.logwarn(f"⚠️ PLANNER FAILED (DOWN): Collision likely at {grasp_pos}")
            return 0.0
        if not self.execute_plan(path_down): return 0.0

        # 5. Grasp
        self.gripper.close()
        rospy.sleep(0.2)
        
        # 6. Lift
        q_curr = self.robot.get_joint_positions()
        path_up = self.planner.plan_cartesian(
            q_curr, hover_pos, target_quat=target_quat, 
            duration=2.0, check_floor=False
        )
        if path_up is not None:
             self.execute_plan(path_up)

        # 7. Check Success
        success = self.sim.check_success()
        return 1.0 if success else 0.0