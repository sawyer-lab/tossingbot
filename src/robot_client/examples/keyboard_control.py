#!/usr/bin/env python3
"""
Keyboard control for individual joints
1q 2w 3e 4r 5t 6y 7u - increase/decrease joints 0-6
oc - open/close gripper
"""
import sys
sys.path.insert(0, '/home/fausto/Projects/tossingbot/src')

from robot_client import RobotClient
import termios
import tty


def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key


def print_state(angles):
    print("\r", end="")
    print("Joints: ", end="")
    for i, angle in enumerate(angles):
        print(f"j{i}:{angle:6.3f} ", end="")
    print("  ", end="", flush=True)


def main():
    print("=== Keyboard Joint Control ===")
    print("1q 2w 3e 4r 5t 6y 7u - +/- joints 0-6")
    print("oc - open/close gripper")
    print("h - home position")
    print("x - exit")
    print()

    robot = RobotClient(protocol='zmq', host='localhost', port=5555)

    angles = robot.get_joint_angles()
    step = 0.05

    print_state(angles)

    while True:
        key = get_key()

        if key == 'x':
            break

        elif key == '1':
            angles[0] += step
        elif key == 'q':
            angles[0] -= step

        elif key == '2':
            angles[1] += step
        elif key == 'w':
            angles[1] -= step

        elif key == '3':
            angles[2] += step
        elif key == 'e':
            angles[2] -= step

        elif key == '4':
            angles[3] += step
        elif key == 'r':
            angles[3] -= step

        elif key == '5':
            angles[4] += step
        elif key == 't':
            angles[4] -= step

        elif key == '6':
            angles[5] += step
        elif key == 'y':
            angles[5] -= step

        elif key == '7':
            angles[6] += step
        elif key == 'u':
            angles[6] -= step

        elif key == 'o':
            robot.gripper_open()
            print("\nGripper: OPEN")
            print_state(angles)
            continue

        elif key == 'c':
            robot.gripper_close()
            print("\nGripper: CLOSE")
            print_state(angles)
            continue

        elif key == 'h':
            angles = [0, 0, 0, 0, 0, 0, 0]
            print("\nHome position")

        else:
            continue

        robot.move_to_joints(angles)
        print_state(angles)

    print("\n\nExiting")


if __name__ == "__main__":
    main()
