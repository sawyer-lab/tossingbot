#!/usr/bin/env python3
"""
gripper_test.py — interactive gripper tester

Connects to the robot server and lets you open/close the gripper,
inspect state, and run a simple open→close→open cycle.

Usage:
    python src/robot_client/examples/gripper_test.py
"""

import sys
import time
sys.path.insert(0, 'src')

from robot_client import RobotClient


def print_state(robot: RobotClient):
    state = robot.gripper_get_state()
    print("\n  Gripper state:")
    for k, v in state.items():
        print(f"    {k}: {v}")


def run_cycle(robot: RobotClient):
    print("\n[cycle] Open → Close → Open")

    print("  → opening...")
    ok = robot.gripper_open()
    print(f"    {'OK' if ok else 'FAIL'}")
    print_state(robot)
    time.sleep(1.0)

    print("  → closing...")
    ok = robot.gripper_close()
    print(f"    {'OK' if ok else 'FAIL'}")
    print_state(robot)
    time.sleep(1.0)

    print("  → opening (leave safe)...")
    ok = robot.gripper_open()
    print(f"    {'OK' if ok else 'FAIL'}")
    print_state(robot)


def interactive(robot: RobotClient):
    print("\nInteractive mode  (o=open  c=close  s=state  r=cycle  q=quit)")
    while True:
        try:
            cmd = input("\n> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            break

        if cmd == 'o':
            ok = robot.gripper_open()
            print(f"open: {'OK' if ok else 'FAIL'}")
        elif cmd == 'c':
            ok = robot.gripper_close()
            print(f"close: {'OK' if ok else 'FAIL'}")
        elif cmd == 's':
            print_state(robot)
        elif cmd == 'r':
            run_cycle(robot)
        elif cmd == 'q':
            break
        else:
            print("  unknown command")


def main():
    print("Gripper Test")
    print("=" * 40)

    robot = RobotClient()
    print(f"✓ Connected")

    print_state(robot)

    if len(sys.argv) > 1 and sys.argv[1] == '--cycle':
        run_cycle(robot)
    else:
        interactive(robot)

    print("\nDone.")


if __name__ == "__main__":
    main()
