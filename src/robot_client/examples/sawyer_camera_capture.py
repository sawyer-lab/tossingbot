#!/usr/bin/env python3
"""
Sawyer Camera Capture

Live viewer for the Sawyer robot's internal cameras.
The robot has two cameras — you choose which one to use at startup.

    head  - camera mounted on the robot's head (wider view of workspace)
    hand  - camera mounted on the wrist / end-effector (close-up view)

Controls:
    s  - save current frame to /tmp/sawyer_<camera>_XXXX.jpg
    q  - quit
"""
import sys
sys.path.insert(0, '/home/fausto/Projects/tossingbot/src')

from robot_client import CameraClient
import cv2
import time


def choose_camera():
    print("\nSawyer Camera Capture")
    print("=" * 40)
    print("  [1] head  — head-mounted camera")
    print("  [2] hand  — wrist / end-effector camera")
    print()

    while True:
        choice = input("Choose camera (1/2) or type 'head'/'hand': ").strip().lower()
        if choice in ('1', 'head'):
            return CameraClient.HEAD
        if choice in ('2', 'hand'):
            return CameraClient.HAND
        print("  Enter 1, 2, 'head', or 'hand'")


def main():
    camera = choose_camera()
    print(f"\nUsing {camera} camera")

    print("Connecting to robot...")
    cam = CameraClient(protocol='zmq', host='localhost')

    print(f"Enabling {camera} camera...")
    if not cam.start(camera=camera):
        print("ERROR: failed to enable camera. Is the ZMQ server running?")
        sys.exit(1)

    print("Waiting for first frame...")
    time.sleep(2.0)

    print(f"\nStreaming {camera} camera — press 's' to save, 'q' to quit\n")

    img_count = 0
    last_img = None

    while True:
        img = cam.get_image(camera=camera)

        if img is not None:
            last_img = img
            display = img.copy()
            cv2.putText(display, f"Sawyer {camera} camera", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow(f'Sawyer — {camera} camera', display)

        key = cv2.waitKey(30) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('s') and last_img is not None:
            filename = f'/tmp/sawyer_{camera}_{img_count:04d}.jpg'
            cv2.imwrite(filename, last_img)
            print(f"Saved: {filename}")
            img_count += 1

    cv2.destroyAllWindows()
    cam.stop(camera=camera)
    cam.close()
    print("Done")


if __name__ == "__main__":
    main()
