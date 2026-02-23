#!/usr/bin/env python3.8
import rospy
import cv2
import numpy as np
import torch
import sys

# Standard Imports
sys.path.insert(0, '/geometry2_ws/devel/lib/python3/dist-packages')
sys.path.insert(0, '/cv_bridge_ws/devel/lib/python3/dist-packages')

from tossingbot.env.gazebo_env import GazeboEnv
from tossingbot.perception.rotation_transform import RotationTransform

def test_real_rotation():
    rospy.init_node("test_rotation_real")
    
    # 1. Setup
    env = GazeboEnv()
    transformer = RotationTransform()
    
    print("Waiting for camera data...")
    state_tensor = None
    while not rospy.is_shutdown():
        state_tensor = env.get_observation()
        if state_tensor is not None: break
        rospy.sleep(0.1)

    print("Got Data! Processing via Shared Helper...")
    state_gpu = state_tensor.unsqueeze(0) # (1, 3, H, W)
    views = []
    
    # Angles [0, 45, 90, 135]
    for i in range(4):
        angle_deg = i * 45.0
        
        # USE THE SHARED HELPER
        rot_tensor = transformer.to_gripper_frame(state_gpu, angle_deg)
        
        # Visualization
        img = rot_tensor.squeeze(0).permute(1, 2, 0).cpu().numpy()
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        
        # Draw "Vertical Gripper" Line (Green)
        H, W, _ = img.shape
        cx, cy = W // 2, H // 2
        cv2.line(img, (cx, cy-20), (cx, cy+20), (0, 255, 0), 2) 
        
        cv2.putText(img, f"Rot: {angle_deg}", (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255), 1)
        img = cv2.copyMakeBorder(img, 0, 0, 2, 2, cv2.BORDER_CONSTANT, value=[50, 50, 50])
        views.append(img)

    # Display
    grid_large = cv2.resize(np.hstack(views), (0,0), fx=3.0, fy=3.0, interpolation=cv2.INTER_NEAREST)
    
    while not rospy.is_shutdown():
        cv2.imshow("Shared Logic Verification", grid_large)
        if cv2.waitKey(10) == ord('q'): break
            
    cv2.destroyAllWindows()

if __name__ == "__main__":
    test_real_rotation()