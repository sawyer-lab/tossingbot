import rospy
import actionlib
from geometry_msgs.msg import Point
from tossing.msg import TossAction, TossGoal

class TossingActionClient:
    def __init__(self):
        rospy.loginfo("Creating action client for 'tossing_action'")
        self.client = actionlib.SimpleActionClient('tossing_action', TossAction)
        rospy.loginfo("Waiting for tossing action server...")
        
        # Add timeout to see if it's actually connecting
        if self.client.wait_for_server(rospy.Duration(10.0)):
            rospy.loginfo("Connected to tossing action server.")
        else:
            rospy.logerr("Tossing action server not available after 10 seconds!")
            rospy.logerr("Check that the server is running with: rosnode list | grep toss")
            raise Exception("Tossing action server not found")

    def toss(self, target_position, target_velocity):
        goal = TossGoal()
        goal.target_position = target_position
        goal.target_velocity = target_velocity
        rospy.loginfo("Sending toss goal: position={}, target_velocity={}".format(target_position, target_velocity))
        self.client.send_goal(goal, feedback_cb=self.feedback_callback)
        self.client.wait_for_result()
        result = self.client.get_result()
        return result.success

    def feedback_callback(self, feedback):
        rospy.loginfo("Tossing feedback: {}".format(feedback.current_step))