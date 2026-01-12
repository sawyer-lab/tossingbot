#!/usr/bin/env python3.8
import sys
import os

# --- 1. PYTHON 3 ROS COMPATIBILITY ---
sys.path.insert(0, '/geometry2_ws/devel/lib/python3/dist-packages')
sys.path.insert(0, '/cv_bridge_ws/devel/lib/python3/dist-packages')

# --- 2. CRITICAL IMPORT ORDER FIX ---
try:
    import open3d as o3d
except ImportError:
    print("Warning: Could not import open3d early. This might cause crashes.")

import torch 
import cv2   
import rospy
import numpy as np
import random
import math

# --- 3. MODULE IMPORTS ---
from tossingbot import config as cfg
from tossingbot.environment.tossing_env import TossingEnv
from tossingbot.learning.agent import TossingAgent
from tossingbot.learning.utils import RotationTransformer

def get_epsilon(step):
    if step >= cfg.EXPLORE_STEPS: return cfg.EXPLORE_END
    frac = float(step) / cfg.EXPLORE_STEPS
    return cfg.EXPLORE_START - frac * (cfg.EXPLORE_START - cfg.EXPLORE_END)

def main():
    rospy.init_node('tossingbot_brain')

    # 1. Init
    env = TossingEnv()
    agent = TossingAgent()
    transformer = RotationTransformer() 
    cv2.namedWindow("Dashboard", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Dashboard", 1400, 900)
    
    # 2. Reset
    obs, _ = env.reset(force_new=True)
    failed_attempts = [] # Short term memory
    
    step_count = 0
    
    rospy.loginfo("STARTING MAIN LOOP...")

    # 3. Loop
    while not rospy.is_shutdown():
        if obs is None: 
            obs = env.get_observation()
            rospy.sleep(0.1); continue

        # --- A. THINK ---
        eps = get_epsilon(step_count)
        # agent returns the specific rotation index and the pixel IN THAT ROTATED FRAME
        rot_idx, u_rot, v_rot, debug = agent.get_action(obs, eps, failed_attempts)
        
        # --- B. TRANSFORM (Pixel Frame -> World Frame) ---
        # The angle this rotation represents (negative because we rotated image opposite)
        angle_deg = (cfg.TOTAL_DEG / cfg.NUM_ROTATIONS) * rot_idx
        
        u_world, v_world = transformer.rotate_pixel(
            u_rot, v_rot, angle_deg, cfg.IMG_H, cfg.IMG_W, to_gripper_frame=False
        )

        # --- C. VISUALIZE (BEFORE ACTING) ---
        # We pass everything needed to draw the full "Thought Process"
        viz_img = render_dashboard(obs, debug, rot_idx, u_rot, v_rot, u_world, v_world, angle_deg, failed_attempts)
        cv2.imshow("Dashboard", viz_img)
        cv2.waitKey(1) # Force draw

        # --- D. ACT ---
        print(f"\n[Step {step_count}] {debug['type']} (Eps: {eps:.2f})")
        print(f"   >>> Rot: {rot_idx} ({angle_deg:.1f}°) | Pixel: ({u_rot}, {v_rot}) -> World: ({u_world}, {v_world})")
        
        reward = env.step(u_world, v_world, rot_idx)
        
        # --- E. LEARN ---
        agent.buffer.push(obs.cpu(), u_rot, v_rot, rot_idx, reward)
        loss = agent.train()
        
        # --- F. SAVE ---
        if step_count % cfg.SAVE_INTERVAL == 0 and step_count > 0:
            agent.save_snapshot(step_count)

        # --- G. RESULT ---
        success = (reward > 0.5)
        msg = "SUCCESS" if success else "FAIL"
        print(f"   >>> Result: {msg} (Rew: {reward}) | Loss: {loss:.4f}")

        if success:
            failed_attempts = []
        else:
            failed_attempts.append((rot_idx, u_rot, v_rot))
        
        # --- H. NEXT EPISODE ---
        obs, is_new = env.reset(previous_success=success)
        if is_new: failed_attempts = []
            
        step_count += 1

def render_dashboard(obs_tensor, debug, chosen_rot, u_rot, v_rot, u_world, v_world, angle_deg, failures):
    """
    Stitches a comprehensive dashboard:
    [ Rot 0 ] [ Rot 1 ] [ Rot 2 ] [ Rot 3 ]
    [      ORIGINAL WORLD FRAME VIEW      ]
    """
    # Helper: Convert PyTorch tensor (C,H,W) to BGR Image
    def to_cv(t):
        if t.dim() == 4: t = t[0] # Handle batch dim
        img = t.detach().cpu().numpy()[:3].transpose(1, 2, 0)
        img = np.clip(img, 0, 1) * 255
        return cv2.cvtColor(img.astype(np.uint8), cv2.COLOR_RGB2BGR)

    # --- 1. TOP ROW: All 4 Rotations ---
    row_images = []
    for i in range(cfg.NUM_ROTATIONS):
        # Input Image
        in_img = to_cv(debug['inputs'][i]) 
        
        # Heatmap (Sigmoid -> ColorMap)
        heatmap_raw = torch.sigmoid(debug['heatmaps'][i, 0]).detach().cpu().numpy()
        heatmap_color = cv2.applyColorMap((heatmap_raw * 255).astype(np.uint8), cv2.COLORMAP_JET)
        
        # Combine (Input + Heatmap) side-by-side
        pair = np.hstack([in_img, heatmap_color])
        
        # --- Annotations ---
        # 1. Draw 'X' for past failures on this rotation
        for (f_rot, f_u, f_v) in failures:
            if f_rot == i:
                # Offset X because pair is [Input | Heatmap]
                # Draw on Input
                cv2.drawMarker(pair, (f_v, f_u), (0,0,255), cv2.MARKER_CROSS, 15, 2)
                # Draw on Heatmap (shift x by width of input)
                cv2.drawMarker(pair, (f_v + in_img.shape[1], f_u), (0,0,255), cv2.MARKER_CROSS, 15, 2)

        # 2. Highlight the CHOSEN rotation
        if i == chosen_rot:
            # Green Border
            cv2.rectangle(pair, (0,0), (pair.shape[1]-1, pair.shape[0]-1), (0, 255, 0), 4)
            # Target Dot on Input
            cv2.circle(pair, (v_rot, u_rot), 5, (0, 255, 0), -1)
            # Target Dot on Heatmap
            cv2.circle(pair, (v_rot + in_img.shape[1], u_rot), 5, (0, 255, 0), -1)

        row_images.append(pair)

    # Stack all rotations horizontally
    top_row = np.hstack(row_images)

    # --- 2. BOTTOM ROW: World View ---
    world_img = to_cv(obs_tensor)
    
    # Scale world image to match top row width
    scale = top_row.shape[1] / world_img.shape[1]
    new_h = int(world_img.shape[0] * scale)
    world_img_resized = cv2.resize(world_img, (top_row.shape[1], new_h))
    
    # Since we resized, we must scale the coordinates too
    u_world_sc = int(u_world * scale)
    v_world_sc = int(v_world * scale)
    
    # Draw Grasp Arrow (Position + Orientation)
    # Convert angle to radians (negative because image y is down)
    rad = np.deg2rad(-angle_deg) 
    arrow_len = 40 * scale
    end_v = int(v_world_sc + arrow_len * np.cos(rad))
    end_u = int(u_world_sc + arrow_len * np.sin(rad))
    
    # Draw Point
    cv2.circle(world_img_resized, (v_world_sc, u_world_sc), 8, (0, 255, 0), -1)
    # Draw Orientation
    cv2.arrowedLine(world_img_resized, (v_world_sc, u_world_sc), (end_v, end_u), (0, 0, 255), 3)
    
    # Text
    # cv2.putText(world_img_resized, f"CHOSEN: Rot {chosen_rot} ({angle_deg:.0f} deg) @ World({u_world}, {v_world})", 
    #             (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)

    # --- 3. COMBINE ---
    final_dashboard = np.vstack([top_row, world_img_resized])
    return final_dashboard

if __name__ == '__main__':
    main()