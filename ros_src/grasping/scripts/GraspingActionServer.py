#!/usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import Point, Pose, Quaternion
from grasping.msg import GraspAction, GraspFeedback, GraspResult
from intera_interface import Limb, Gripper
from servo import servo_to_pose

class GraspingActionServer:
    
    def __init__(self):
        self.server = actionlib.SimpleActionServer('grasping_action', GraspAction, self.execute, False)
        self.server.start()
        self.hover_distance = 0.15
        self.tip_name = "right_gripper_tip"
        self.rate = rospy.Rate(100)
        self.orientations = [
            Quaternion(x=0.5201222224573814, y=0.8538840942230885, z=0.018356824109129057, w=0.004225440502089707),
            Quaternion(x=0.3435611712966451, y=0.938941257985801, z=0.01718393225261385, w=0.00772968962226557),
            Quaternion(x=0.15378749940184505, y=0.9879242241213907, z=0.015347877594357887, w=0.010935038876232166),
            Quaternion(x=-0.041922438258336144, y=0.9989433107237086, z=0.012906919834853963, w=0.013717964543223798),
            Quaternion(x=-0.23600337334964266, y=0.971569707921142, z=0.009983520623337707, w=0.015969963480851085),
            Quaternion(x=-0.42099012474085523, y=0.9068694964004237, z=0.0066874207077319775, w=0.017615611638516836),
            Quaternion(x=-0.5898427181432643, y=0.8072983169026616, z=0.0031219902526232403, w=0.018580865328563734),
            Quaternion(x=0.7360156845345732, y=-0.6767022474874311, z=0.0005631673210772487, w=-0.018832504005884956),
            Quaternion(x=0.8538947209243543, y=-0.5201047101127484, z=0.004224262533762417, w=-0.018358967893082067),
            Quaternion(x=0.938957625274501, y=-0.34351680424924885, z=0.007721742492273572, w=-0.017180158035836213),
            Quaternion(x=0.9879331290075164, y=-0.15373154604357372, z=0.010926307321131968, w=-0.015341452629881342),
            Quaternion(x=0.9989413649465675, y=0.04196553672180887, z=0.013707545019383701, w=-0.012928508648760469),
            Quaternion(x=0.9715556965898072, y=0.23606030325512037, z=0.01596872112796346, w=-0.010003079352515463),
            Quaternion(x=0.9068449956221245, y=0.4210431740932566, z=0.017609615988638545, w=-0.006685872367726574),
            Quaternion(x=0.8072863778156645, y=0.5898592139998273, z=0.01857585806107479, w=-0.003122394924607331),
            Quaternion(x=0.6767046199248393, y=0.7360136152144672, z=0.018828185872277962, w=0.0005612557954010435)
        ]
        rospy.loginfo("Grasping Action Server is ready.")


    def execute(self, goal):
        feedback = GraspFeedback()
        result = GraspResult()
        
        try:
            self.limb = Limb("right")
            self.gripper = Gripper()
        except:
            rospy.logerr("Failed to initialize limb or gripper interface.")
            result.success = False
            self.server.set_aborted(result)
            return
        

        # Step 1: Hover
        feedback.current_step = "Hovering"
        self.server.publish_feedback(feedback)
        hover_pose = Pose(
            position=Point(
                x=goal.target_position.x,
                y=goal.target_position.y,
                z=goal.target_position.z + self.hover_distance
            ),
            orientation=self.orientations[goal.orientation_index]
        )
        moved = self.move_to_pose(hover_pose)
        if not moved:
            rospy.logwarn("Failed to move to hover pose.")
            result.success = False
            self.server.set_aborted(result)
            return


        # Step 2: Grasp
        self.gripper.open()
        feedback.current_step = "Grasping"
        self.server.publish_feedback(feedback)
        grasp_pose = Pose(
            position=goal.target_position,
            orientation=self.orientations[goal.orientation_index]
        )
        servo_to_pose(self.limb, self.tip_name, self.rate, grasp_pose)
        current_pose = self.limb.endpoint_pose()
        error = abs(current_pose["position"].z - grasp_pose.position.z)
        print("Positioning error: {:.4f} m".format(error))
        self.gripper.close()
        rospy.sleep(1.0)

        # Step 3: Retract
        feedback.current_step = "Retracting"
        self.server.publish_feedback(feedback)
        servo_to_pose(self.limb, self.tip_name, self.rate, hover_pose)

        # if not self.gripper.is_gripping():
        #     rospy.logwarn("Gripper failed to grip the object.")
        #     result.success = False
        #     self.server.set_aborted(result)
        #     return

        # Task completed
        result.success = True
        self.server.set_succeeded(result)
        
    def move_to_pose(self, pose):
        joint_angles = self.limb.ik_request(pose, self.tip_name)
        if joint_angles:
            self.limb.move_to_joint_positions(joint_angles)
            return True
        else:
            rospy.logwarn("No valid joint angles found for the given pose.")
            return False
            


if __name__ == "__main__":
    rospy.init_node('Grasping_action_server')
    server = GraspingActionServer()
    rospy.spin()