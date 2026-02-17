#!/usr/bin/env python3
"""
Demo of separate client classes

Shows how to use each subsystem independently.
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from robot_client import (
    RobotClient,
    GripperClient,
    CameraClient,
    GazeboClient,
    HeadClient,
    LightsClient
)


def main():
    print("=== Separate Clients Demo ===\n")

    # Each subsystem has its own client
    robot = RobotClient(protocol='zmq', host='localhost')
    gripper = GripperClient(protocol='zmq', host='localhost')
    camera = CameraClient(protocol='zmq', host='localhost')
    gazebo = GazeboClient(protocol='zmq', host='localhost')
    head = HeadClient(protocol='zmq', host='localhost')
    lights = LightsClient(protocol='zmq', host='localhost')

    # Test robot
    print("[1] Robot - Getting joint angles...")
    angles = robot.get_joint_angles()
    print(f"    Angles: {[f'{a:.2f}' for a in angles]}")

    # Test gripper
    print("\n[2] Gripper - Getting state...")
    state = gripper.get_state()
    print(f"    Position: {state.get('position', 'N/A')}")
    print(f"    Grasping: {state.get('is_grasping', 'N/A')}")

    # Test camera
    print("\n[3] Camera - Listing cameras...")
    cameras = camera.get_camera_names()
    print(f"    Available: {cameras}")

    # Test gazebo
    print("\n[4] Gazebo - Listing available models...")
    models = gazebo.available_models()
    print(f"    Found {len(models)} models")
    print(f"    Examples: {models[:5]}")

    # Test head
    print("\n[5] Head - Getting pan angle...")
    pan = head.pan()
    print(f"    Pan angle: {pan:.3f} rad")

    # Test lights
    print("\n[6] Lights - Listing lights...")
    all_lights = lights.list_all()
    print(f"    Available: {all_lights}")

    print("\n=== Demo Complete ===")

    # Clean up
    robot.close()
    gripper.close()
    camera.close()
    gazebo.close()
    head.close()
    lights.close()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback
        traceback.print_exc()
