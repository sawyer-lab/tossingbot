#!/usr/bin/env python

import rospy
import actionlib
from plain_perception.msg import LandingDetectionAction, LandingDetectionGoal

class LandingDetectionClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('landing_detection', LandingDetectionAction)
        rospy.loginfo("Waiting for landing detection action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to landing detection server")
    
    def feedback_callback(self, feedback):
        rospy.loginfo("Feedback: %s", feedback.status)
    
    def detect_landing(self, timeout=30.0):
        """
        Detect landing and return the landing index.
        
        Args:
            timeout: Maximum time to wait for result (seconds)
            
        Returns:
            int: Landing index, or -1 if failed/timeout
        """
        goal = LandingDetectionGoal()
        rospy.loginfo("Sending goal to detect landing...")
        
        self.client.send_goal(goal, feedback_cb=self.feedback_callback)
        
        finished = self.client.wait_for_result(rospy.Duration(timeout))
        
        if finished:
            result = self.client.get_result()
            rospy.loginfo("Landing Index: %d", result.landing_index)
            rospy.loginfo("Message: %s", result.message)
            return result.landing_index
        else:
            rospy.logwarn("Action timed out")
            self.client.cancel_goal()
            return -1
    
    def cancel(self):
        """Cancel the current goal."""
        self.client.cancel_goal()

if __name__ == '__main__':
    try:
        rospy.init_node('landing_client')
        
        client = LandingDetectionClient()
        
        # Can now call multiple times
        index1 = client.detect_landing()
        rospy.sleep(2.0)
        index2 = client.detect_landing()
        
    except rospy.ROSInterruptException:
        pass