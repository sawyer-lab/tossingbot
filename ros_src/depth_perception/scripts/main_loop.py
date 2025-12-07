#!/usr/bin/env python3.8
import sys
import os
import random
import numpy as np
import cv2
import torch
import torch.optim as optim
import torch.nn as nn

# PATH HACKS
sys.path.insert(0, '/geometry2_ws/devel/lib/python3/dist-packages')
sys.path.insert(0, '/cv_bridge_ws/devel/lib/python3/dist-packages')

import rospy
from gazebo_env import GazeboEnv
from network import TossingBot

# --- CONFIGURATION ---
LEARNING_RATE = 1e-4
MAX_STEPS = 10000
SAVE_PATH = "simple_grasp_model.pth"
BATCH_SIZE = 8 

class UnlimitedBuffer:
    def __init__(self):
        self.memory = []
    
    def push(self, state, u, v, reward):
        # Save to CPU RAM to save GPU memory
        self.memory.append((state.cpu(), u, v, float(reward)))

    def sample_batch(self, size):
        # Get random samples from history
        return random.sample(self.memory, size)

    def __len__(self):
        return len(self.memory)

def train_system():
    rospy.init_node("simple_trainer")
    
    # 1. SETUP
    env = GazeboEnv()
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    rospy.loginfo(f"Running on: {device}")

    # 2. NETWORK (The Brain)
    model = TossingBot().to(device)
    if os.path.exists(SAVE_PATH):
        model.load_state_dict(torch.load(SAVE_PATH))
        rospy.loginfo("Loaded existing brain.")
    model.train()

    optimizer = optim.Adam(model.parameters(), lr=LEARNING_RATE)
    # Loss: Binary Classification (Did I succeed or fail?)
    loss_fn = nn.BCEWithLogitsLoss() 

    buffer = UnlimitedBuffer()
    cv2.namedWindow("View", cv2.WINDOW_NORMAL)

    for step_idx in range(MAX_STEPS):
        if rospy.is_shutdown(): break

        # ----------------------------------------------------------------------
        # 1. OBSERVE (Get Image)
        # ----------------------------------------------------------------------
        state = env.reset() # (3, H, W) Tensor
        if state is None: continue
        
        state_gpu = state.unsqueeze(0).to(device) # Add batch dim: (1, 3, H, W)

        # ----------------------------------------------------------------------
        # 2. PREDICT (Forward Pass)
        # ----------------------------------------------------------------------
        # The net outputs a "heatmap" of scores
        logits = model(state_gpu) 
        grasp_map = logits[0, 0, :, :] # (H, W)

        # ----------------------------------------------------------------------
        # 3. ACT (Pure Greedy - No Randomness)
        # ----------------------------------------------------------------------
        # Find the single pixel with the HIGHEST score
        flat_idx = torch.argmax(grasp_map).item()
        W = grasp_map.shape[1]
        
        u = flat_idx // W  # Row
        v = flat_idx % W   # Col

        # IMPORTANT: If the network is brand new, it outputs noise. 
        # Ideally we want it to pick *somewhere*, so argmax is fine.

        # ----------------------------------------------------------------------
        # 4. EXECUTE
        # ----------------------------------------------------------------------
        reward = env.step(u, v) # Returns 1.0 or 0.0

        # Save to memory
        buffer.push(state, u, v, reward)
        rospy.loginfo(f"Step {step_idx} | Action ({u}, {v}) | Reward: {reward}")

        # ----------------------------------------------------------------------
        # 5. TRAIN (Backprop)
        # ----------------------------------------------------------------------
        loss_val = 0.0
        if len(buffer) >= BATCH_SIZE:
            # A. Create Batch (Current sample + Random past samples)
            mini_batch = buffer.sample_batch(BATCH_SIZE - 1)
            mini_batch.append((state.cpu(), u, v, float(reward))) # Always include current

            # B. Stack Tensors
            b_states = torch.stack([x[0] for x in mini_batch]).to(device)
            b_u = [x[1] for x in mini_batch]
            b_v = [x[2] for x in mini_batch]
            b_rewards = torch.tensor([x[3] for x in mini_batch], dtype=torch.float32).to(device).unsqueeze(1)

            # C. Forward Pass on Batch
            b_logits = model(b_states) # (Batch, 1, H, W)

            # D. Select ONLY the pixels we actually clicked
            # We don't want to calculate loss for pixels we didn't touch
            pred_values = []
            for i in range(BATCH_SIZE):
                val = b_logits[i, 0, b_u[i], b_v[i]]
                pred_values.append(val)
            pred_values = torch.stack(pred_values).unsqueeze(1)

            # E. Update Weights
            loss = loss_fn(pred_values, b_rewards)
            
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            loss_val = loss.item()

        # ----------------------------------------------------------------------
        # 6. VISUALIZE (Fixed Alignment)
        # ----------------------------------------------------------------------
        # RGB Image
        rgb_tensor = state.permute(1, 2, 0).cpu().numpy()
        rgb_img = cv2.cvtColor(rgb_tensor, cv2.COLOR_RGB2BGR)
        rgb_img = (rgb_img * 255).astype(np.uint8)

        # Heatmap (Prediction)
        prob_map = torch.sigmoid(grasp_map).detach().cpu().numpy()
        heatmap_img = (prob_map * 255).astype(np.uint8)
        
        # Resize Heatmap to match RGB exactly
        heatmap_img = cv2.resize(heatmap_img, (rgb_img.shape[1], rgb_img.shape[0]), interpolation=cv2.INTER_NEAREST)
        heatmap_color = cv2.applyColorMap(heatmap_img, cv2.COLORMAP_JET)

        # Draw Circles
        color = (0, 255, 0) if reward > 0.5 else (0, 0, 255) # Green vs Red
        cv2.circle(rgb_img, (v, u), 4, color, -1)
        cv2.circle(heatmap_color, (v, u), 4, color, -1)

        # Combine
        combined = np.hstack([rgb_img, heatmap_color])
        combined = cv2.resize(combined, (0,0), fx=3.0, fy=3.0, interpolation=cv2.INTER_NEAREST)
        
        # Text
        cv2.putText(combined, f"Step: {step_idx} Loss: {loss_val:.4f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
        cv2.imshow("View", combined)
        cv2.waitKey(1)

        if step_idx % 50 == 0:
            torch.save(model.state_dict(), SAVE_PATH)

    cv2.destroyAllWindows()

if __name__ == "__main__":
    train_system()