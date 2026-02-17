#!/usr/bin/env python3
"""
Display robot configuration and state information
"""
import sys
sys.path.insert(0, '/home/fausto/Projects/tossingbot/src')

from robot_client import RobotClient


def main():
    print("=== Robot Information ===\n")
    robot = RobotClient(protocol='zmq', host='localhost')

    # Robot configuration
    print("Configuration:")
    print(f"  Robot name: {robot.get_robot_name()}")
    print(f"  Limbs: {robot.get_limb_names()}")
    print(f"  Cameras: {robot.get_camera_names()}")

    limbs = robot.get_limb_names()
    if limbs:
        print(f"\n  Joint names ({limbs[0]}):")
        joints = robot.get_joint_names(limbs[0])
        for i, joint in enumerate(joints, 1):
            print(f"    {i}. {joint}")

    # Robot state
    print("\nRobot State:")
    state = robot.robot_state()
    if state:
        print(f"  Enabled: {state.get('enabled', 'unknown')}")
        print(f"  Stopped: {state.get('stopped', 'unknown')}")
        print(f"  Error: {state.get('error', 'unknown')}")
        print(f"  E-Stop: {state.get('estop_button', 'unknown')}")

    # Joint angles
    print("\nCurrent Joint Angles (radians):")
    angles = robot.get_joint_angles()
    if angles and limbs:
        joints = robot.get_joint_names(limbs[0])
        for joint, angle in zip(joints, angles):
            print(f"  {joint}: {angle:.4f}")

    # Gripper
    print("\nGripper State:")
    gripper = robot.gripper_get_state()
    print(f"  Position: {gripper.get('position', 'unknown'):.4f} m")
    print(f"  Grasping: {gripper.get('is_grasping', 'unknown')}")

    print("\nDone!")


if __name__ == "__main__":
    main()
