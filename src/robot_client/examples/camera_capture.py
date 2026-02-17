#!/usr/bin/env python3
"""
Camera capture example
Press 's' to save image, 'q' to quit
"""
import sys
sys.path.insert(0, '/home/fausto/Projects/tossingbot/src')

from robot_client import RobotClient
import cv2
import time


def main():
    print("=== Camera Capture ===")
    print("Connecting to robot...")

    robot = RobotClient(protocol='zmq', host='localhost')

    print("Starting camera...")
    robot.camera_start()
    time.sleep(2.0)

    print("Camera ready. Press 's' to save, 'q' to quit")

    img_count = 0

    while True:
        img = robot.camera_get_image()

        if img is not None:
            cv2.imshow('Robot Camera', img)

        key = cv2.waitKey(30) & 0xFF

        if key == ord('q'):
            break

        elif key == ord('s'):
            if img is not None:
                filename = f'/tmp/robot_image_{img_count:04d}.jpg'
                cv2.imwrite(filename, img)
                print(f"Saved: {filename}")
                img_count += 1

    cv2.destroyAllWindows()
    robot.camera_stop()
    print("Done")


if __name__ == "__main__":
    main()
