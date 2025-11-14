import rospy
import actionlib
from geometry_msgs.msg import Point
from simulation.msg import TossAction, TossGoal

class TossingActionClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('tossing_action', TossAction)
        rospy.loginfo("Waiting for tossing action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to tossing action server.")

    def toss(self, target_position, target_velocity):
        goal = TossGoal()
        goal.target_position = target_position
        goal.target_velocity = target_velocity
        rospy.loginfo("Sending pick goal: position={}, target_velocity={}".format(target_position, target_velocity))
        self.client.send_goal(goal, feedback_cb=self.feedback_callback)
        self.client.wait_for_result()
        result = self.client.get_result()
        return result.success

    def feedback_callback(self, feedback):
        rospy.loginfo("Tossing feedback: {}".format(feedback.current_step))
