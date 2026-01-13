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

# =============================================================================
# DEBUG MODE: Set to False for normal training/execution
# =============================================================================
DEBUG_MODE = False  # Set to True for visual debugging with user confirmation

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
        # In DEBUG_MODE, disable failed_attempts inhibition to test all rotations
        fail_list = [] if DEBUG_MODE else failed_attempts
        rot_idx, u_rot, v_rot, debug = agent.get_action(obs, eps, failed_attempts=fail_list)
        
        # --- B. TRANSFORM (Pixel Frame -> World Frame) ---
        # Image was rotated by -(TOTAL_DEG/NUM_ROTATIONS)*rot_idx
        # To map coordinates back, we rotate by the SAME angle (not opposite!)
        # This is because after center crop, we're in the same coordinate space
        angle_deg = -(cfg.TOTAL_DEG / cfg.NUM_ROTATIONS) * rot_idx
        
        u_world, v_world = transformer.rotate_pixel(
            u_rot, v_rot, angle_deg, cfg.IMG_H, cfg.IMG_W, to_gripper_frame=False
        )

        # --- C. VISUALIZE (BEFORE ACTING) ---
        # We pass everything needed to draw the full "Thought Process"
        viz_img = render_dashboard(obs, debug, rot_idx, u_rot, v_rot, u_world, v_world, angle_deg, failed_attempts)
        cv2.imshow("Dashboard", viz_img)
        
        # DEBUG_MODE: Wait for user confirmation before executing
        if DEBUG_MODE:
            print(f"\n{'='*70}")
            print(f"[Step {step_count}] {debug['type']} (Eps: {eps:.2f})")
            print(f"  Rotation: {rot_idx} → Image rotated by {angle_deg:.1f}°")
            print(f"  Selected in ROTATED image: (u={u_rot}, v={v_rot})")
            print(f"  Transformed to WORLD image: (u={u_world}, v={v_world})")
            print(f"  (Should align visually: GREEN crosshair = CYAN crosshair location)")
            print(f"{'='*70}")
            print("Press ENTER to execute grasp (or 'q' to quit)...")
            
            key = cv2.waitKey(0)
            if key == ord('q'):
                break
        else:
            # Normal mode: just brief waitKey for cv2.imshow to update
            cv2.waitKey(1)

        # --- D. ACT ---
        action_type = "EXPLORE" if eps > random.random() else "EXPLOIT"
        print(f"\n[Step {step_count:05d}] {action_type} | Rot={rot_idx} | Pixel=({u_world},{v_world}) | Eps={eps:.3f}")
        reward = env.step(u_world, v_world, rot_idx)
        
        # --- E. LEARN ---
        agent.buffer.push(obs.cpu(), u_rot, v_rot, rot_idx, reward)
        loss = agent.train()
        
        # --- F. SAVE ---
        if step_count % cfg.SAVE_INTERVAL == 0 and step_count > 0:
            agent.save_snapshot(step_count)

        # --- G. RESULT ---
        success = (reward > 0.5)
        status = "✓ SUCCESS" if success else "✗ FAIL"
        print(f"   Result: {status} | Reward={reward:.1f} | Loss={loss:.4f}\n")

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
    Stitches a comprehensive dashboard with detailed visual debugging
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
        
        # 2. Highlight the CHOSEN rotation
        if i == chosen_rot:
            # Green Border
            cv2.rectangle(pair, (0,0), (pair.shape[1]-1, pair.shape[0]-1), (0, 255, 0), 3)
            
            # FINER CROSSHAIR on Input (selected point)
            # Draw thin crosshair lines
            cv2.line(pair, (v_rot-10, u_rot), (v_rot+10, u_rot), (0, 255, 0), 1)
            cv2.line(pair, (v_rot, u_rot-10), (v_rot, u_rot+10), (0, 255, 0), 1)
            # Center dot
            cv2.circle(pair, (v_rot, u_rot), 2, (0, 255, 0), -1)
            
            # Same on Heatmap (shift x by width of input)
            offset = in_img.shape[1]
            cv2.line(pair, (v_rot+offset-10, u_rot), (v_rot+offset+10, u_rot), (0, 255, 0), 1)
            cv2.line(pair, (v_rot+offset, u_rot-10), (v_rot+offset, u_rot+10), (0, 255, 0), 1)
            cv2.circle(pair, (v_rot + offset, u_rot), 2, (0, 255, 0), -1)

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
    
    # FINER CROSSHAIR for world position
    cv2.line(world_img_resized, (v_world_sc-20, u_world_sc), (v_world_sc+20, u_world_sc), (0, 255, 255), 2)
    cv2.line(world_img_resized, (v_world_sc, u_world_sc-20), (v_world_sc, u_world_sc+20), (0, 255, 255), 2)
    cv2.circle(world_img_resized, (v_world_sc, u_world_sc), 3, (0, 255, 255), -1)
    
    # Draw Orientation Arrow
    rad = np.deg2rad(-angle_deg) 
    arrow_len = 50 * scale
    end_v = int(v_world_sc + arrow_len * np.cos(rad))
    end_u = int(u_world_sc + arrow_len * np.sin(rad))
    cv2.arrowedLine(world_img_resized, (v_world_sc, u_world_sc), (end_v, end_u), (255, 0, 255), 2, tipLength=0.3)
    
    # Add text labels
    cv2.putText(world_img_resized, f"Rot{chosen_rot} ({angle_deg:.0f}deg) @ ({u_world},{v_world})", 
                (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(world_img_resized, f"CYAN=Target, MAGENTA=Orientation", 
                (20, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # --- 3. COMBINE ---
    final_dashboard = np.vstack([top_row, world_img_resized])
    return final_dashboard

if __name__ == '__main__':
    main()