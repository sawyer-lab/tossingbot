#!/usr/bin/env python3
"""
Complete robot demo - lights, head, display, camera
"""
import sys
sys.path.insert(0, '/home/fausto/Projects/tossingbot/src')

from robot_client import RobotClient
import cv2
import numpy as np
import time


def main():
    print("=== Robot Demo ===")
    robot = RobotClient(protocol='zmq', host='localhost')

    # Lights
    print("\n1. Testing lights...")
    lights = robot.lights_list()
    print(f"Available lights: {lights}")
    if lights:
        print(f"Turning on {lights[0]}...")
        robot.lights_set(lights[0], True)
        time.sleep(1)
        print(f"Turning off {lights[0]}...")
        robot.lights_set(lights[0], False)

    # Head pan
    print("\n2. Testing head pan...")
    current = robot.head_pan()
    print(f"Current pan angle: {current:.3f} rad")
    print("Panning left...")
    robot.head_set_pan(0.5, speed=0.5)
    time.sleep(2)
    print("Panning center...")
    robot.head_set_pan(0.0, speed=0.5)
    time.sleep(2)

    # Camera + Display
    print("\n3. Testing camera and display...")
    img = robot.camera_get_image()
    if img is not None:
        print(f"Got camera image: {img.shape}")

        # Add text to image
        img_copy = img.copy()
        cv2.putText(img_copy, "Hello from TossingBot!", (50, 100),
                   cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 3)

        print("Displaying on head screen...")
        robot.display_image(img_copy)
        time.sleep(3)

        print("Clearing display...")
        robot.display_clear()

    print("\nDemo complete!")


if __name__ == "__main__":
    main()
