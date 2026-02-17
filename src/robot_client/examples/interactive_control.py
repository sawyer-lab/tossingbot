#!/usr/bin/env python3
"""
Interactive robot control via ZMQ client
"""
import sys
sys.path.insert(0, '/home/fausto/Projects/tossingbot/src')

from robot_client import RobotClient


def print_state(robot):
    angles = robot.get_joint_angles()
    print(f"\nCurrent joint angles:")
    for i, angle in enumerate(angles):
        print(f"  j{i}: {angle:.4f} rad ({angle * 57.2958:.2f}°)")


def main():
    print("=== Interactive Robot Control ===")
    print("Connecting to robot...")

    robot = RobotClient(protocol='zmq', host='localhost', port=5555)

    print_state(robot)

    while True:
        print("\nCommands:")
        print("  1. Move to joint positions")
        print("  2. Show current state")
        print("  3. Open gripper")
        print("  4. Close gripper")
        print("  5. Home position")
        print("  q. Quit")

        choice = input("\nChoice: ").strip()

        if choice == 'q':
            break

        elif choice == '1':
            print("\nEnter 7 joint angles (comma-separated, in radians):")
            print("Example: 0, 0.3, 0.5, 0, 0, -0.2, 0")
            try:
                user_input = input("Angles: ").strip()
                angles = [float(x.strip()) for x in user_input.split(',')]

                if len(angles) != 7:
                    print(f"Error: Expected 7 angles, got {len(angles)}")
                    continue

                print(f"Moving to: {angles}")
                robot.move_to_joints(angles)
                print_state(robot)

            except ValueError as e:
                print(f"Error: Invalid input - {e}")

        elif choice == '2':
            print_state(robot)

        elif choice == '3':
            print("Opening gripper...")
            robot.gripper_open()

        elif choice == '4':
            print("Closing gripper...")
            robot.gripper_close()

        elif choice == '5':
            print("Moving to home position...")
            home = [0, 0, 0, 0, 0, 0, 0]
            robot.move_to_joints(home)
            print_state(robot)

        else:
            print("Invalid choice")

    print("\nDisconnected")


if __name__ == "__main__":
    main()
