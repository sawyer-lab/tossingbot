#!/usr/bin/env python3
import rospy
import torch
import torch.optim as optim
import numpy as np
from gazebo_env import GazeboEnv
from network import TossingBot

def train_system():
    # 1. Init
    env = GazeboEnv()
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    
    # Load the combined Perception + Grasping model
    model = TossingBot().to(device)
    model.train()
    
    optimizer = optim.Adam(model.parameters(), lr=1e-4)
    loss_fn = torch.nn.BCEWithLogitsLoss()

    rospy.loginfo(f"Training started on {device} with {model.__class__.__name__}")

    for episode in range(10000):
        if rospy.is_shutdown(): break

        # --- STEP 1: OBSERVE ---
        state = env.reset() # (3, H, W)
        if state is None: continue
        
        # Add batch dim: (1, 3, H, W)
        state_tensor = state.unsqueeze(0).to(device)

        # --- STEP 2: FORWARD PASS ---
        # Output shape: (1, 2, H, W)
        logits = model(state_tensor)
        
        # We use Channel 0 for Grasp Quality
        q_map = logits[:, 0, :, :] 

        # --- STEP 3: ACT (Epsilon Greedy) ---
        epsilon = 0.5
        if np.random.rand() < epsilon:
            # Random Pixel
            u = np.random.randint(0, q_map.shape[1])
            v = np.random.randint(0, q_map.shape[2])
        else:
            # Best Pixel (Argmax of Q-map)
            flat_idx = torch.argmax(q_map).item()
            u = flat_idx // q_map.shape[2]
            v = flat_idx % q_map.shape[2]

        # Execute in Gazebo
        reward = env.step(u, v)
        rospy.loginfo(f"Ep {episode} | Pixel: ({u},{v}) | Reward: {reward}")

        # --- STEP 4: LEARN ---
        # We only have ground truth for the ONE pixel we selected.
        # We create a target just for that pixel.
        
        target_val = torch.tensor(reward, dtype=torch.float32).to(device)
        pred_val = q_map[0, u, v] # The network's prediction at that pixel
        
        loss = loss_fn(pred_val, target_val)

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        if episode % 10 == 0:
            rospy.loginfo(f"--- Loss: {loss.item():.4f} ---")

if __name__ == "__main__":
    train_system()