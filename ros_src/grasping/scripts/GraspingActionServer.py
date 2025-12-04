#!/usr/bin/env python

import rospy
import actionlib
import os
from geometry_msgs.msg import Point, Pose, Quaternion
from grasping.msg import GraspAction, GraspFeedback, GraspResult
from intera_interface import Limb, Gripper
# We don't need servo_to_pose anymore if we use the planner for everything
# from servo import servo_to_pose 
from cartesian_planner import CasadiIKPlanner

class GraspingActionServer:
    
    def __init__(self):
        self.server = actionlib.SimpleActionServer('grasping_action', GraspAction, self.execute, False)
        self.server.start()
        
        self.hover_distance = 0.30
        self.tip_name = "right_gripper_tip" # Ensuring we control the tip, not wrist
        self.rate = rospy.Rate(100)
        
        # 90 degrees around Y-axis (Points Z-axis straight down)
        self.orientations = [
            Quaternion(
                             x=-0.00142460053167,
                             y=0.999994209902,
                             z=-0.00177030764765,
                             w=0.00253311793936)
        ]
        
        # --- PLANNER SETUP ---
        # Ensure this path is correct inside your container/system!
        self.urdf_path = "/home/kid/ros_ws/src/grasping/sawyer_model.urdf" 
        
        if not os.path.exists(self.urdf_path):
            rospy.logerr("URDF File not found at: " + self.urdf_path)
            rospy.logerr("Please run: rosrun xacro xacro --inorder ... > sawyer_model.urdf")
        else:
            rospy.loginfo("Loading CasADi Planner...")
            self.planner = CasadiIKPlanner(self.urdf_path, base_link="base", end_link=self.tip_name)
            rospy.loginfo("Grasping Action Server is READY.")


    def execute(self, goal):
        feedback = GraspFeedback()
        result = GraspResult()
        
        rospy.loginfo("\n" + "="*30)
        rospy.loginfo("NEW GOAL RECEIVED")
        rospy.loginfo("Target: [x={:.3f}, y={:.3f}, z={:.3f}]".format(
            goal.target_position.x, goal.target_position.y, goal.target_position.z))

        try:
            self.limb = Limb("right")
            self.gripper = Gripper()
        except:
            rospy.logerr("[ERROR] Failed to initialize limb/gripper.")
            result.success = False
            self.server.set_aborted(result)
            return
        
        # ---------------------------------------------------------
        # Step 1: Hover
        # ---------------------------------------------------------
        feedback.current_step = "Hovering"
        self.server.publish_feedback(feedback)
        
        hover_z = goal.target_position.z + self.hover_distance
        
        hover_pose = Pose(
            position=Point(
                x=goal.target_position.x,
                y=goal.target_position.y,
                z=hover_z
            ),
            orientation=self.orientations[goal.orientation_index]
        )
        
        rospy.loginfo("[STEP 1] Moving to Hover (using Planner)...")
        moved = self.move_to_pose(hover_pose)
        
        if not moved:
            rospy.logwarn("[FAIL] Planner could not reach Hover Pose.")
            result.success = False
            self.server.set_aborted(result)
            return

        # ---------------------------------------------------------
        # Step 2: Grasp
        # ---------------------------------------------------------
        rospy.loginfo("[STEP 2] Opening Gripper & Descending...")
        self.gripper.open()
        
        feedback.current_step = "Grasping"
        self.server.publish_feedback(feedback)
        
        grasp_pose = Pose(
            position=goal.target_position,
            orientation=self.orientations[goal.orientation_index]
        )
        
        # --- FIX: Use move_to_pose (Planner) instead of servo_to_pose ---
        # This ensures the Soft Orientation Constraint is applied!
        reached_grasp = self.move_to_pose(grasp_pose)
        
        # Check Error
        current_pose = self.limb.endpoint_pose()
        z_error = abs(current_pose["position"].z - grasp_pose.position.z)
        rospy.loginfo("[DEBUG] Descent Finished. Vertical Error: {:.4f} m".format(z_error))
        
        if z_error > 0.05: # If we are more than 5cm away, something is wrong
             rospy.logwarn("Grasp descent didn't reach target strictly. Closing anyway.")

        self.gripper.close()
        rospy.sleep(0.5) 

        # ---------------------------------------------------------
        # Step 3: Retract
        # ---------------------------------------------------------
        feedback.current_step = "Retracting"
        self.server.publish_feedback(feedback)
        
        rospy.loginfo("[STEP 3] Retracting...")
        # Reuse planner to pull back up smoothly
        self.move_to_pose(hover_pose)

        # ---------------------------------------------------------
        # Completion
        # ---------------------------------------------------------
        rospy.loginfo("[SUCCESS] Action Complete.")
        result.success = True
        self.server.set_succeeded(result)
        
    def move_to_pose(self, pose):
        """
        Uses CasADi Planner to move from Current -> Target
        """
        # 1. Get Current Joint Config
        current_joints = self.limb.joint_angles()
        joint_names = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
        q_start = [current_joints[n] for n in joint_names]
        
        # 2. Extract Target
        target_p = [pose.position.x, pose.position.y, pose.position.z]
        target_q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        
        # 3. Plan Path (15 steps is usually good for a short 15cm move)
        # Note: If moving far (Hover -> Start), maybe increase steps inside the planner class or pass it here
        path = self.planner.plan_path(q_start, target_p, target_q, steps=20)
        
        if path:
            # 4. Execute Path
            for q_step in path:
                cmd = dict(zip(joint_names, q_step))
                self.limb.set_joint_positions(cmd)
                # Small sleep to allow robot to reach setpoint. 
                # Decrease for smoother/faster motion, Increase for accuracy.
                rospy.sleep(0.04) 
            return True
        else:
            return False

if __name__ == "__main__":
    rospy.init_node('Grasping_action_server')
    server = GraspingActionServer()
    rospy.spin()