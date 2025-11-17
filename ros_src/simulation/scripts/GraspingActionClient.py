#!/usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import Point
from grasping.msg import GraspAction, GraspGoal

class GraspingActionClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('grasping_action', GraspAction)
        rospy.loginfo("Waiting for Grasping action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to Grasping action server.")

    def Grasp(self, target_position, orientation_index):
        goal = GraspGoal()
        goal.target_position = target_position
        goal.orientation_index = orientation_index
        rospy.loginfo("Sending Grasp goal: position={}, orientation_index={}".format(target_position, orientation_index))
        self.client.send_goal(goal, feedback_cb=self.feedback_callback)
        self.client.wait_for_result()
        result = self.client.get_result()
        return result.success

    def feedback_callback(self, feedback):
        rospy.loginfo("Grasping feedback: {}".format(feedback.current_step))
