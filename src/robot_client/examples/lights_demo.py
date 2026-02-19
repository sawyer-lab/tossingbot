#!/usr/bin/env python3
"""
lights_demo.py — LightsClient usage examples

Demonstrates listing, reading, and controlling all robot lights
(navigator, cuff, head LEDs).

Run from the repo root:
    python src/robot_client/examples/lights_demo.py
"""
import sys
import time
import os

sys.path.insert(0, '/home/fausto/Projects/tossingbot/src')

from robot_client import LightsClient

CONTAINER_HOST = os.environ.get('ROBOT_HOST', 'localhost')


def main():
    lights = LightsClient(protocol='zmq', host=CONTAINER_HOST)

    # ------------------------------------------------------------------
    # 1. Discover available lights
    # ------------------------------------------------------------------
    print('=== 1. Discovering lights ===')
    names = lights.list_all()
    if not names:
        print('[WARN] No lights found — is the server running?')
        lights.close()
        return
    print(f'  Found {len(names)} light(s):')
    for n in names:
        state = lights.get(n)
        print(f'    {n}: {"ON" if state else "off"}')

    # ------------------------------------------------------------------
    # 2. Turn every light on, then off
    # ------------------------------------------------------------------
    print('\n=== 2. All lights ON ===')
    lights.all_on()
    time.sleep(1.5)

    print('=== 2. All lights OFF ===')
    lights.all_off()
    time.sleep(1.0)

    # ------------------------------------------------------------------
    # 3. Blink each light individually
    # ------------------------------------------------------------------
    print('\n=== 3. Individual blink ===')
    for n in names:
        print(f'  Blinking: {n}')
        lights.set(n, True)
        time.sleep(0.4)
        lights.set(n, False)
        time.sleep(0.2)

    # ------------------------------------------------------------------
    # 4. Fast strobe on all lights (10 cycles)
    # ------------------------------------------------------------------
    print('\n=== 4. Strobe (10x) ===')
    for _ in range(10):
        lights.all_on()
        time.sleep(0.1)
        lights.all_off()
        time.sleep(0.1)

    # ------------------------------------------------------------------
    # 5. Leave a navigator light on as status indicator
    # ------------------------------------------------------------------
    nav = next((n for n in names if 'navigator' in n.lower()), None)
    if nav:
        print(f'\n=== 5. Leaving {nav} on as status indicator ===')
        lights.set(nav, True)

    print('\nDone.')
    lights.close()


if __name__ == '__main__':
    main()
