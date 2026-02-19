#!/usr/bin/env python3
"""
hand_camera_to_head_display.py

Grabs the live hand (wrist) camera feed and mirrors it onto
the Sawyer head display in a continuous loop.

Controls:
    q  - quit
    s  - save current frame to /tmp/hand_frame_XXXX.jpg
"""
import sys
import os
import time

import cv2
import numpy as np

sys.path.insert(0, '/home/fausto/Projects/tossingbot/src')

from robot_client import CameraClient, HeadClient

CONTAINER_HOST = os.environ.get('ROBOT_HOST', 'localhost')
HEAD_DISPLAY_SIZE = (1024, 600)   # Sawyer head screen resolution (w, h)


def fit_to_display(img, size=HEAD_DISPLAY_SIZE):
    """Letterbox-fit img into the head display canvas (black bars if needed)."""
    w, h = size
    ih, iw = img.shape[:2]
    scale = min(w / iw, h / ih)
    nw, nh = int(iw * scale), int(ih * scale)
    resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    canvas = np.zeros((h, w, 3), dtype=np.uint8)
    x0 = (w - nw) // 2
    y0 = (h - nh) // 2
    canvas[y0:y0 + nh, x0:x0 + nw] = resized
    return canvas


def main():
    print("Hand Camera → Head Display")
    print("=" * 40)
    print("Connecting to robot...")

    cam  = CameraClient(protocol='zmq', host=CONTAINER_HOST)
    head = HeadClient(protocol='zmq', host=CONTAINER_HOST)

    # Check hand camera is reachable
    state = cam.get_state(camera=CameraClient.HAND)
    if state is None:
        print("ERROR: could not reach ZMQ server. Is the container running?")
        sys.exit(1)
    print(f"Hand camera topic : {state.get('topic')}")
    print(f"Image available   : {state.get('has_image')}")

    print("Starting hand camera stream...")
    cam.start(camera=CameraClient.HAND)

    # Wait for first frame
    print("Waiting for first frame ", end='', flush=True)
    for _ in range(30):
        time.sleep(0.2)
        s = cam.get_state(camera=CameraClient.HAND)
        if s and s.get('has_image'):
            print(" ready.")
            break
        print('.', end='', flush=True)
    else:
        print("\nWARNING: no frame yet, continuing anyway...")

    print("\nMirroring hand camera to head display — press 'q' to quit, 's' to save\n")

    frame_count = 0
    save_count  = 0

    while True:
        img = cam.get_image(camera=CameraClient.HAND)

        if img is not None:
            display = fit_to_display(img)

            # Overlay frame counter
            cv2.putText(display, f"Hand cam  frame {frame_count}",
                        (10, 28), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 255, 0), 2, cv2.LINE_AA)

            head.display_image(display)
            frame_count += 1

            # Also show locally so the operator can see it
            cv2.imshow('Hand → Head Display  (q=quit  s=save)', display)

        key = cv2.waitKey(30) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s') and img is not None:
            path = f'/tmp/hand_frame_{save_count:04d}.jpg'
            cv2.imwrite(path, img)
            print(f"Saved: {path}")
            save_count += 1

    cv2.destroyAllWindows()
    cam.stop(camera=CameraClient.HAND)
    cam.close()
    head.close()
    print("Done.")


if __name__ == '__main__':
    main()
