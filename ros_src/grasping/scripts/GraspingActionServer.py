#!/usr/bin/env python
import rospy
import actionlib
import os
import numpy as np
from geometry_msgs.msg import Point, Pose, Quaternion
from grasping.msg import GraspAction, GraspFeedback, GraspResult
from intera_interface import Limb, Gripper
from cartesian_planner import CasadiIKPlanner

class GraspingActionServer(object):
    
    def __init__(self):
        # 1. Initialize Node
        rospy.init_node('Grasping_action_server')
        
        self.server = actionlib.SimpleActionServer('grasping_action', GraspAction, self.execute, False)
        self.server.start()
        
        # 2. Configuration
        self.control_freq = 100.0 # Hz
        self.rate = rospy.Rate(self.control_freq) # The heartbeat
        self.hover_distance = 0.30
        self.tip_name = "right_gripper_tip" 
        
        self.orientations = [
            Quaternion(
                             x=-0.00142460053167,
                             y=0.999994209902,
                             z=-0.00177030764765,
                             w=0.00253311793936)
        ]
        
        # 3. Hardware Interfaces
        self.limb = Limb("right")
        self.gripper = Gripper()
        self.joint_names = self.limb.joint_names()

        # 4. Planner Setup
        self.urdf_path = "/home/kid/ros_ws/src/grasping/sawyer_model.urdf" 
        if not os.path.exists(self.urdf_path):
            rospy.logerr("URDF File not found at: {}".format(self.urdf_path))
        else:
            self.planner = CasadiIKPlanner(self.urdf_path, base_link="base", end_link=self.tip_name)
            rospy.loginfo("Grasping Server READY @ {} Hz".format(self.control_freq))

    def execute(self, goal):
        result = GraspResult()
        feedback = GraspFeedback()
        
        # Python 2 string formatting
        rospy.loginfo("GOAL: [{:.3f}, {:.3f}, {:.3f}]".format(
            goal.target_position.x, goal.target_position.y, goal.target_position.z))

        # --- STEP 1: HOVER ---
        feedback.current_step = "Hovering"
        self.server.publish_feedback(feedback)
        
        hover_pose = Pose(
            position=Point(x=goal.target_position.x, 
                           y=goal.target_position.y, 
                           z=goal.target_position.z + self.hover_distance),
            orientation=self.orientations[goal.orientation_index]
        )
        
        # Move fast (2.0 seconds)
        if not self.move_to_pose_smooth(hover_pose, duration=2.0):
            self.server.set_aborted(result)
            return

        # --- STEP 2: GRASP DESCENT ---
        feedback.current_step = "Descending"
        self.server.publish_feedback(feedback)
        self.gripper.open()
        
        grasp_pose = Pose(
            position=goal.target_position,
            orientation=self.orientations[goal.orientation_index]
        )
        
        # Move slower for precision (2.5 seconds)
        if not self.move_to_pose_smooth(grasp_pose, duration=2.5):
            self.server.set_aborted(result)
            return

        # --- STEP 3: ACTUATE GRIPPER ---
        feedback.current_step = "Closing Gripper"
        self.server.publish_feedback(feedback)
        self.gripper.close()
        
        # Wait 0.5s (keeping 100Hz sync)
        self.wait_duration(0.5)

        # --- STEP 4: RETRACT ---
        feedback.current_step = "Retracting"
        self.server.publish_feedback(feedback)
        
        # Move up (2.0 seconds)
        self.move_to_pose_smooth(hover_pose, duration=2.0)

        # --- DONE ---
        result.success = True
        self.server.set_succeeded(result)

    def move_to_pose_smooth(self, target_pose, duration):
        """
        Calculates the exact number of steps needed to hit 100Hz
        and streams them to the robot.
        """
        # 1. Calculate required resolution
        # Force float division just in case
        num_steps = int(duration * self.control_freq)
        
        # 2. Get Current Config
        current_joints = self.limb.joint_angles()
        q_start = [current_joints[n] for n in self.joint_names]
        
        target_p = [target_pose.position.x, target_pose.position.y, target_pose.position.z]
        target_q = [target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w]
        
        # 3. Plan Path (High Resolution)
        path = self.planner.plan_path(q_start, target_p, target_q, steps=num_steps)
        
        if path is None or len(path) == 0:
            rospy.logerr("Planner failed to find path.")
            return False

        # 4. Stream at 100Hz
        for q_step in path:
            # Check for Preemption (Safety)
            if self.server.is_preempt_requested():
                self.server.set_preempted()
                return False

            # Send Command
            # zip returns a list of tuples in Python 2, which is fine for dict()
            cmd = dict(zip(self.joint_names, q_step))
            self.limb.set_joint_positions(cmd)
            
            # Sleep specifically to maintain 100Hz
            self.rate.sleep()
            
        return True

    def wait_duration(self, seconds):
        """Waits for X seconds while keeping the node alive at 100Hz"""
        ticks = int(seconds * self.control_freq)
        # xrange is slightly more efficient in Py2 for loops, but range is fine too
        for _ in range(ticks):
            self.rate.sleep()

if __name__ == "__main__":
    server = GraspingActionServer()
    rospy.spin()