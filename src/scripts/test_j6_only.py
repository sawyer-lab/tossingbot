#!/usr/bin/env python3.8
import rospy
import numpy as np
import sys
import os

# Add the path to tossing_system src to find tossingbot module
sys.path.append(os.path.join(os.getcwd(), 'ros_src/tossing_system/src'))

from tossingbot.hardware.sawyer import SawyerInterface, RobotCommand, ControlMode


def test_j6():
    rospy.init_node("test_j6_only", anonymous=True)
    
    rospy.loginfo("Initializing SawyerInterface...")
    try:
        robot = SawyerInterface()
    except Exception as e:
        rospy.logerr(f"Failed to initialize SawyerInterface: {e}")
        return

    # 1. Read current positions
    start_joints = robot.get_joint_positions()
    j6_start = start_joints[6]
    rospy.loginfo(f"Current Joint Positions: {np.round(start_joints, 4)}")
    rospy.loginfo(f"Starting j6 position: {j6_start:.4f} rad")

    # 2. Define a very small movement for j6 (+0.1 rad)
    # Ensure it's within typical limits (Sawyer j6 is usually +/- 4.7 rad or similar)
    delta = 0.1
    target_joints = list(start_joints)
    target_joints[6] += delta
    
    # SAFETY CHECK
    if target_joints[6] > 4.7 or target_joints[6] < -4.7:
        rospy.logerr(f"Target j6 ({target_joints[6]:.4f}) is outside safe bounds (+/- 4.7). Aborting.")
        return

    rospy.loginfo(f"Target j6 position: {target_joints[6]:.4f} rad")
    rospy.loginfo("Only j6 will move. Other joints will be commanded to stay at their current sensed positions:")
    for i, name in enumerate(robot._joint_names):
        status = "MOVING" if i == 6 else "STATIONARY"
        rospy.loginfo(f"  {name}: {start_joints[i]:.4f} -> {target_joints[i]:.4f} ({status})")
    
    print("\n" + "="*50)
    print("WARNING: This will move the physical robot.")
    print("Please ensure the area is clear and you have an E-STOP ready.")
    print("="*50 + "\n")
    
    user_input = input("Type 'yes' to proceed (OR anything else to abort): ")
    if user_input.lower() != 'yes':
        rospy.loginfo("Aborted by user.")
        return

    # 3. Move to target
    tolerance = 0.01
    max_error = 100.0
    timeout = 10.0
    start_time = rospy.Time.now()

    cmd = RobotCommand(position=target_joints)
    chunk = [cmd] * 10 # 0.1s worth of commands

    rospy.loginfo("Executing movement...")
    while max_error > tolerance and not rospy.is_shutdown():
        elapsed = (rospy.Time.now() - start_time).to_sec()
        if elapsed > timeout:
            rospy.logwarn(f"\nMove timeout reached. Final max error: {max_error:.4f}")
            break

        robot.execute_stream(chunk, ControlMode.POSITION)
        
        current = robot.get_joint_positions()
        errors = [abs(c - t) for c, t in zip(current, target_joints)]
        max_error = max(errors)
        
        sys.stdout.write(f"\rCurrent Positions: {np.round(current, 3)} | MaxErr: {max_error:.4f}")
        sys.stdout.flush()

    print("\n\nMovement finished.")
    
    # 4. Read final positions
    final_joints = robot.get_joint_positions()
    rospy.loginfo(f"Final j6 position: {final_joints[6]:.4f} rad (Shift: {final_joints[6] - j6_start:.4f})")

    # 5. Move back to original j6
    print("\n" + "="*50)
    user_input = input("Type 'yes' to move back to original position (OR anything else to stop here): ")
    if user_input.lower() != 'yes':
        rospy.loginfo("Stopping here. Done.")
        return
    
    target_joints[6] = j6_start
    start_time = rospy.Time.now()
    max_error = 100.0
    cmd = RobotCommand(position=target_joints)
    chunk = [cmd] * 10

    rospy.loginfo("Returning to original position...")
    while max_error > tolerance and not rospy.is_shutdown():
        elapsed = (rospy.Time.now() - start_time).to_sec()
        if elapsed > timeout:
            break
        robot.execute_stream(chunk, ControlMode.POSITION)
        current = robot.get_joint_positions()
        errors = [abs(c - t) for c, t in zip(current, target_joints)]
        max_error = max(errors)
        sys.stdout.write(f"\rCurrent Positions: {np.round(current, 3)} | MaxErr: {max_error:.4f}")
        sys.stdout.flush()

    print("\n\nReturned to start.")
    rospy.loginfo("Test complete.")

if __name__ == "__main__":
    try:
        test_j6()
    except rospy.ROSInterruptException:
        pass