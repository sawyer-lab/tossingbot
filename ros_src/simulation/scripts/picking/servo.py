from geometry_msgs.msg import Pose, Point
from tf.transformations import quaternion_slerp
import rospy

def servo_to_pose(limb, tip_name, rate, pose, time=4.0):
    current_pose = limb.endpoint_pose()
    steps = 100 * time
    ik_delta = Point()
    ik_delta.x = (current_pose['position'].x - pose.position.x) / steps
    ik_delta.y = (current_pose['position'].y - pose.position.y) / steps
    ik_delta.z = (current_pose['position'].z - pose.position.z) / steps
    q_current = [current_pose['orientation'].x,
                 current_pose['orientation'].y,
                 current_pose['orientation'].z,
                 current_pose['orientation'].w]
    q_pose = [pose.orientation.x,
              pose.orientation.y,
              pose.orientation.z,
              pose.orientation.w]
    for d in range(int(steps), -1, -1):
        if rospy.is_shutdown():
            return
        ik_step = Pose()
        ik_step.position.x = d * ik_delta.x + pose.position.x
        ik_step.position.y = d * ik_delta.y + pose.position.y
        ik_step.position.z = d * ik_delta.z + pose.position.z
        # Perform a proper quaternion interpolation
        q_slerp = quaternion_slerp(q_current, q_pose, d / steps)
        ik_step.orientation.x = q_slerp[0]
        ik_step.orientation.y = q_slerp[1]
        ik_step.orientation.z = q_slerp[2]
        ik_step.orientation.w = q_slerp[3]
        joint_angles = limb.ik_request(ik_step, tip_name)
        if joint_angles:
            limb.set_joint_positions(joint_angles)
        else:
            rospy.logwarn("No valid joint angles found for the given pose.")
            return
        rate.sleep()