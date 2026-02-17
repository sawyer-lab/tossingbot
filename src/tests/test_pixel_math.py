#!/usr/bin/env python3.8
import sys
import numpy as np
import cv2
import torch
import torch.nn.functional as F
import rospy
import random

# --- IMPORTS ---
sys.path.insert(0, '/geometry2_ws/devel/lib/python3/dist-packages')
sys.path.insert(0, '/cv_bridge_ws/devel/lib/python3/dist-packages')
from tossingbot import config as cfg
from tossingbot.environment.tossing_env import TossingEnv
from tossingbot.learning.utils import RotationTransformer, create_rotated_batch, normalize_tensor

def main():
    rospy.init_node('pixel_math_test')
    
    # 1. Setup
    env = TossingEnv()
    transformer = RotationTransformer()
    
    # 2. Get a single static image (No robot movement)
    print("Capturing Image...")
    obs, _ = env.reset(force_new=True)
    obs_cuda = obs.to('cuda')
    
    # 3. Generate the Batch (Simulate what the Network sees)
    # We use your EXACT function to ensure we debug the real pipeline
    rotated_batch = create_rotated_batch(obs_cuda, cfg.NUM_ROTATIONS, 'cuda')
    
    print("\n--- CONTROLS ---")
    print(" [SPACE] : Pick a new random pixel/rotation")
    print(" [Q]     : Quit")
    
    while not rospy.is_shutdown():
        # --- A. PICK RANDOM TARGET ---
        # 1. Choose a Rotation (e.g., 90 degrees)
        rot_idx = random.randint(0, cfg.NUM_ROTATIONS - 1)
        angle_deg = (360.0 / cfg.NUM_ROTATIONS) * rot_idx
        
        # 2. Choose a Pixel in that ROTATED frame
        # We pick something away from the center to make rotation errors obvious
        h, w = cfg.IMG_H, cfg.IMG_W
        u_rot = random.randint(h//4, 3*h//4)
        v_rot = random.randint(w//4, 3*w//4)
        
        # --- B. EXECUTE TRANSFORM (The Logic Under Test) ---
        # We want to know: "Where is this pixel in the World Frame?"
        u_world, v_world = transformer.rotate_pixel(
            u_rot, v_rot, angle_deg, h, w, to_gripper_frame=False
        )

        # --- C. VISUALIZE ---
        # 1. Get the specific rotated image the network would see
        img_rot_tensor = rotated_batch[rot_idx] # (C, H, W)
        
        # 2. Convert to CV2 for display
        img_rot_cv = tensor_to_cv(img_rot_tensor)
        img_world_cv = tensor_to_cv(obs)
        
        # 3. DRAW MARKERS
        # GREEN Dot on Rotated View (The "Network's Choice")
        cv2.circle(img_rot_cv, (v_rot, u_rot), 5, (0, 255, 0), -1)
        cv2.putText(img_rot_cv, f"Net View ({angle_deg:.0f} deg)", (10, 20), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # RED Dot on World View (The "Calculated World Pos")
        cv2.circle(img_world_cv, (v_world, u_world), 5, (0, 0, 255), -1)
        cv2.putText(img_world_cv, "World View (Result)", (10, 20), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        
        # 4. Show Side-by-Side
        # Resize for easier viewing
        scale = 2
        H, W = img_rot_cv.shape[:2]
        display = np.hstack([img_rot_cv, img_world_cv])
        display = cv2.resize(display, (W*2*scale, H*scale), interpolation=cv2.INTER_NEAREST)
        
        cv2.imshow("Pixel Math Debugger", display)
        
        # --- WAIT FOR USER ---
        key = cv2.waitKey(0)
        if key == ord('q'):
            break
            
def tensor_to_cv(t):
    """Converts (C,H,W) tensor to (H,W,3) BGR numpy array."""
    if t.dim() == 4: t = t[0]
    img = t.detach().cpu().numpy()[:3].transpose(1, 2, 0)
    
    # De-Normalize for display if needed (Assuming mean ~0.5)
    # If your image looks grey/weird, this is why, but geometry is still valid.
    img = (img - np.min(img)) / (np.max(img) - np.min(img)) 
    
    return cv2.cvtColor((img * 255).astype(np.uint8), cv2.COLOR_RGB2BGR)

if __name__ == '__main__':
    main()