#!/usr/bin/env python3.8
import rospy
import cv2
import numpy as np
import sys

# Import Config directly to get dimensions
from tossingbot import config
from tossingbot.env.gazebo_env import GazeboEnv

def run_calibration():
    rospy.init_node("coord_calibration")
    env = GazeboEnv()
    
    # Setup Window
    cv2.namedWindow("Calibration", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Calibration", 600, 600)
    
    print("------------------------------------------------")
    print("   COORDINATE CALIBRATION TOOL")
    print("------------------------------------------------")
    print(f" Grid Size: {config.IMG_H} x {config.IMG_W}")
    print("1. Click anywhere in the image.")
    print("2. The robot will move to HOVER over that point.")
    print("------------------------------------------------")

    def mouse_cb(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # 1. Map Click (Window) -> Grid (Tensor)
            # Window size might be different from Grid size due to resizing
            # We assume the user resizes the window, but we need the click relative to the image logic.
            
            # The displayed image is likely scaled or resized.
            # To be safe, let's grab the current displayed frame size from the loop, 
            # BUT since we can't access local vars easily here, let's assume 
            # we are displaying at a fixed scale or use a class.
            
            # SIMPLIFIED: We will force the display to be a specific size (e.g. 500x500)
            # so math is predictable.
            DISPLAY_SIZE = 500.0
            
            scale_x = config.IMG_W / DISPLAY_SIZE
            scale_y = config.IMG_H / DISPLAY_SIZE
            
            v = int(x * scale_x) # Col
            u = int(y * scale_y) # Row
            
            # Clamp
            v = min(max(v, 0), config.IMG_W - 1)
            u = min(max(u, 0), config.IMG_H - 1)
            
            # 2. Get World Coords
            target = env.vision.pixel_to_world(u, v)
            
            print(f"\n[CLICK] Pixel: ({u}, {v})")
            print(f" -> Mapped to World: X={target[0]:.3f}, Y={target[1]:.3f}")
            
            # 3. Move Robot (Hover 20cm above)
            hover_pos = [target[0], target[1], 0.20]
            
            # Use specific planner settings for safety
            q_curr = env.robot.get_joint_positions()
            path = env.planner.plan_cartesian(
                q_curr, hover_pos, duration=2.0, check_floor=False
            )
            
            if path:
                print(" -> Moving...")
                env.execute_plan(path)
            else:
                print(" -> [FAIL] Point out of reach or planner failed.")

    cv2.setMouseCallback("Calibration", mouse_cb)

    while not rospy.is_shutdown():
        obs = env.get_observation()
        if obs is None: 
            rospy.sleep(0.1)
            continue
        
        # Convert to BGR for display
        img = obs.permute(1, 2, 0).cpu().numpy()
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        
        # Draw Arrows to visualize the Coordinate System
        H, W, _ = img.shape
        cx, cy = W // 2, H // 2
        
        # Draw +X Axis (Robot Forward) -> Should correspond to Image Top (Row 0)
        cv2.arrowedLine(img, (cx, cy), (cx, cy-20), (0, 0, 255), 1)
        cv2.putText(img, "+X", (cx+5, cy-20), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,255), 1)

        # Draw +Y Axis (Robot Left) -> Should correspond to Image Left (Col 0)
        # (Assuming standard ROS coords: X=Fwd, Y=Left)
        cv2.arrowedLine(img, (cx, cy), (cx-20, cy), (0, 255, 0), 1)
        cv2.putText(img, "+Y", (cx-30, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,255,0), 1)

        # Resize for easy clicking (Match the callback logic!)
        display = cv2.resize(img, (500, 500), interpolation=cv2.INTER_NEAREST)
        cv2.imshow("Calibration", display)
        
        if cv2.waitKey(20) == ord('q'): break

if __name__ == "__main__":
    run_calibration()