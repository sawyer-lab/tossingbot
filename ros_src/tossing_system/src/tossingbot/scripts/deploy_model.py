#!/usr/bin/env python3.8
import sys
import os
import torch
import numpy as np
import cv2
import rospy
import rospkg

# --- MODULAR IMPORTS ---
from tossingbot.env.gazebo_env import GazeboEnv
from tossingbot.learning.network import TossingBot

# --- CONFIG ---
WEIGHTS_FILENAME = "tossingbot_manual.pth"

class DeployNode:
    def __init__(self):
        # 1. Init System
        self.env = GazeboEnv()
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        rospy.loginfo(f"Deploying on Device: {self.device}")

        # 2. Setup Paths & Load Model
        rp = rospkg.RosPack()
        pkg_path = rp.get_path('tossingbot_system')
        weights_path = os.path.join(pkg_path, "weights", WEIGHTS_FILENAME)
        
        self.model = TossingBot().to(self.device)
        
        if os.path.exists(weights_path):
            rospy.loginfo(f"Loading Brain: {weights_path}")
            self.model.load_state_dict(torch.load(weights_path))
            
            # CRITICAL: Eval mode freezes BatchNorm/Dropout for deterministic inference
            self.model.eval() 
        else:
            rospy.logerr(f"Weights file not found at {weights_path}")
            rospy.logerr("Did you run manual_trainer.py first?")
            sys.exit(1)

        # 3. GUI
        cv2.namedWindow("TossingBot Autonomous", cv2.WINDOW_NORMAL)
        self.step_count = 0

    def run(self):
        rospy.loginfo("--- STARTING AUTONOMOUS LOOP ---")
        
        while not rospy.is_shutdown():
            # ==========================================
            # 1. OBSERVE
            # ==========================================
            state = self.env.get_observation()
            if state is None:
                rospy.sleep(0.1)
                continue
            
            # Prepare Input: (1, 3, H, W)
            state_gpu = state.unsqueeze(0).to(self.device)

            # ==========================================
            # 2. THINK (Inference Only)
            # ==========================================
            with torch.no_grad(): # No gradients needed
                logits = self.model(state_gpu)
                heatmap = logits[0, 0, :, :] # Extract (H, W)

            # ==========================================
            # 3. ACT (Pure Greedy)
            # ==========================================
            # Find the absolute best pixel
            flat_idx = torch.argmax(heatmap).item()
            H, W = heatmap.shape
            
            u = flat_idx // W # Row
            v = flat_idx % W  # Col
            
            # Calculate Confidence (Sigmoid of the raw logit)
            confidence = torch.sigmoid(heatmap[u, v]).item() * 100.0

            rospy.loginfo(f"Step {self.step_count} | Action: ({u},{v}) | Confidence: {confidence:.1f}%")

            # Show what we are about to do
            self.visualize(state, heatmap, u, v, confidence)
            
            # Execute
            success = self.env.step(u, v)
            
            if success:
                rospy.loginfo(">>> GRASP SUCCESS")
            else:
                rospy.logwarn(">>> GRASP FAILED")

            # Reset for next attempt
            self.env.reset()
            self.step_count += 1

    def visualize(self, state, heatmap, u, v, conf):
        # RGB
        img_rgb = state.permute(1, 2, 0).cpu().numpy()
        img_rgb = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
        
        # Heatmap
        prob_map = torch.sigmoid(heatmap).cpu().numpy()
        img_heat = (prob_map * 255).astype(np.uint8)
        img_heat = cv2.applyColorMap(img_heat, cv2.COLORMAP_JET)
        
        # Resize Heatmap to match RGB
        if img_heat.shape[:2] != img_rgb.shape[:2]:
            img_heat = cv2.resize(img_heat, (img_rgb.shape[1], img_rgb.shape[0]), interpolation=cv2.INTER_NEAREST)

        # Draw Crosshair on Target
        # Green for RGB, White for Heatmap
        cv2.drawMarker(img_rgb, (v, u), (0, 255, 0), markerType=cv2.MARKER_CROSS, markerSize=15, thickness=2)
        cv2.drawMarker(img_heat, (v, u), (255, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=15, thickness=2)

        # Stack
        display = np.hstack([img_rgb, img_heat])
        display = cv2.resize(display, (0,0), fx=3.0, fy=3.0, interpolation=cv2.INTER_NEAREST)
        
        # Text
        text = f"Action: ({u},{v}) | Conf: {conf:.1f}%"
        cv2.putText(display, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        cv2.imshow("TossingBot Autonomous", display)
        cv2.waitKey(1) # Refresh window

if __name__ == "__main__":
    node = DeployNode()
    node.run()