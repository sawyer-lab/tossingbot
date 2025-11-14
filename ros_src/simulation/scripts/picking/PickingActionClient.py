#!/usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import Point
from simulation.msg import PickAction, PickGoal

class PickingActionClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('picking_action', PickAction)
        rospy.loginfo("Waiting for picking action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to picking action server.")

    def pick(self, target_position, orientation_index):
        goal = PickGoal()
        goal.target_position = target_position
        goal.orientation_index = orientation_index
        rospy.loginfo("Sending pick goal: position={}, orientation_index={}".format(target_position, orientation_index))
        self.client.send_goal(goal, feedback_cb=self.feedback_callback)
        self.client.wait_for_result()
        result = self.client.get_result()
        return result.success

    def feedback_callback(self, feedback):
        rospy.loginfo("Picking feedback: {}".format(feedback.current_step))
