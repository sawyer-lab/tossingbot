#!/usr/bin/env python3
"""
head_demo.py — HeadClient usage examples

Demonstrates head pan control and head display:
  - Reading and setting pan angle at different speeds
  - Generating and displaying a test image
  - Overlaying text on the head screen
  - Clearing the display

Run from the repo root:
    python src/robot_client/examples/head_demo.py
"""
import sys
import time
import os
import math

import cv2
import numpy as np

sys.path.insert(0, '/home/fausto/Projects/tossingbot/src')

from robot_client import HeadClient

CONTAINER_HOST = os.environ.get('ROBOT_HOST', 'localhost')


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def make_solid_image(color_bgr, text=None, size=(1024, 600)):
    """Create a solid-colour BGR image with optional centred text."""
    img = np.full((size[1], size[0], 3), color_bgr, dtype=np.uint8)
    if text:
        font       = cv2.FONT_HERSHEY_SIMPLEX
        scale      = 2.5
        thickness  = 4
        (tw, th), _ = cv2.getTextSize(text, font, scale, thickness)
        x = (size[0] - tw) // 2
        y = (size[1] + th) // 2
        cv2.putText(img, text, (x, y), font, scale, (255, 255, 255), thickness, cv2.LINE_AA)
    return img


def make_gradient_image(size=(1024, 600)):
    """Blue→green horizontal gradient with centred label."""
    img = np.zeros((size[1], size[0], 3), dtype=np.uint8)
    for x in range(size[0]):
        t = x / size[0]
        b = int(255 * (1 - t))
        g = int(255 * t)
        img[:, x] = (b, g, 0)
    cv2.putText(img, 'TossingBot', (size[0]//2 - 200, size[1]//2),
                cv2.FONT_HERSHEY_SIMPLEX, 3, (255, 255, 255), 5, cv2.LINE_AA)
    return img


def make_crosshair_image(size=(1024, 600)):
    """Dark image with a centred crosshair — useful as a gripper aim display."""
    img = np.zeros((size[1], size[0], 3), dtype=np.uint8)
    cx, cy = size[0] // 2, size[1] // 2
    cv2.line(img, (cx - 80, cy), (cx + 80, cy), (0, 255, 0), 2)
    cv2.line(img, (cx, cy - 80), (cx, cy + 80), (0, 255, 0), 2)
    cv2.circle(img, (cx, cy), 40, (0, 255, 0), 2)
    cv2.circle(img, (cx, cy), 80, (0, 200, 0), 1)
    return img


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    head = HeadClient(protocol='zmq', host=CONTAINER_HOST)

    # ------------------------------------------------------------------
    # 1. Read current angle
    # ------------------------------------------------------------------
    print('=== 1. Current head pan ===')
    angle = head.pan()
    print(f'  Current angle: {angle:.4f} rad  ({math.degrees(angle):.1f} deg)')

    # ------------------------------------------------------------------
    # 2. Pan sweep: centre → left → right → centre
    # ------------------------------------------------------------------
    print('\n=== 2. Pan sweep ===')
    positions = [
        (0.0,  0.5, 'Centre'),
        (0.8,  0.4, 'Left  (~46 deg)'),
        (-0.8, 0.4, 'Right (~46 deg)'),
        (0.0,  0.5, 'Centre'),
    ]
    for angle_rad, speed, label in positions:
        print(f'  → {label}')
        head.set_pan(angle_rad, speed=speed)
        time.sleep(1.8)

    # ------------------------------------------------------------------
    # 3. Slow nod sweep to show speed parameter
    # ------------------------------------------------------------------
    print('\n=== 3. Slow sweep (speed=0.2) ===')
    head.set_pan(0.5, speed=0.2)
    time.sleep(3.0)
    head.set_pan(0.0, speed=0.2)
    time.sleep(3.0)

    # ------------------------------------------------------------------
    # 4. Display: solid colour cards
    # ------------------------------------------------------------------
    print('\n=== 4. Colour cards on head display ===')
    cards = [
        ((0,   0,   180), 'RED'),
        ((0,   180, 0),   'GREEN'),
        ((180, 0,   0),   'BLUE'),
    ]
    for bgr, label in cards:
        print(f'  Showing {label}')
        img = make_solid_image(bgr, text=label)
        head.display_image(img)
        time.sleep(1.2)

    # ------------------------------------------------------------------
    # 5. Display: gradient
    # ------------------------------------------------------------------
    print('\n=== 5. Gradient image ===')
    head.display_image(make_gradient_image())
    time.sleep(2.0)

    # ------------------------------------------------------------------
    # 6. Display: crosshair (gripper aim overlay)
    # ------------------------------------------------------------------
    print('\n=== 6. Crosshair overlay ===')
    head.display_image(make_crosshair_image())
    time.sleep(2.0)

    # ------------------------------------------------------------------
    # 7. Live angle readout on the screen while panning
    # ------------------------------------------------------------------
    print('\n=== 7. Live angle display while panning ===')
    head.set_pan(0.6, speed=0.3)
    for _ in range(20):
        a = head.pan()
        img = make_solid_image((30, 30, 30),
                               text=f'{math.degrees(a):+.1f} deg')
        head.display_image(img)
        time.sleep(0.15)
    head.set_pan(0.0, speed=0.3)
    time.sleep(2.0)

    # ------------------------------------------------------------------
    # 8. Leave TossingBot branding on screen
    # ------------------------------------------------------------------
    print('\n=== 8. TossingBot splash (stays on screen) ===')
    head.display_image(make_gradient_image())

    print('\nDone.')
    head.close()


if __name__ == '__main__':
    main()
