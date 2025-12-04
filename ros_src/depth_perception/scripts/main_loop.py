#!/usr/bin/env python3.8
import sys
import os

# --- PATH HACKS ---
sys.path.insert(0, '/geometry2_ws/devel/lib/python3/dist-packages')
sys.path.insert(0, '/cv_bridge_ws/devel/lib/python3/dist-packages')

import rospy
import torch
import torch.optim as optim
import torch.nn.functional as F
import numpy as np
import cv2

from gazebo_env import GazeboEnv
from network import TossingBot

# --- CONFIGURATION ---
LEARNING_RATE = 1e-4
MAX_EPISODES = 5000
SAVE_PATH = "tossingbot_grasp.pth"
EPSILON_START = 0.5
EPSILON_END = 0.1
EPSILON_DECAY = 1000

def train_system():
    if rospy.get_node_uri() is None:
        rospy.init_node("tossingbot_trainer")

    env = GazeboEnv()
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    rospy.loginfo(f"Training on Device: {device}")

    model = TossingBot().to(device)
    if os.path.exists(SAVE_PATH):
        rospy.loginfo(f"Loading checkpoint: {SAVE_PATH}")
        model.load_state_dict(torch.load(SAVE_PATH))
    model.train()

    optimizer = optim.Adam(model.parameters(), lr=LEARNING_RATE)
    loss_fn = torch.nn.BCEWithLogitsLoss(reduction='none') 

    # --- METRICS HISTORY ---
    reward_history = []
    loss_history = []

    epsilon = EPSILON_START
    
    for episode in range(MAX_EPISODES):
        if rospy.is_shutdown(): break

        # --- 1. OBSERVE ---
        state = env.reset() # (3, H, W)
        if state is None: continue
        
        state_tensor = state.unsqueeze(0).to(device)

        # --- 2. FORWARD PASS ---
        logits = model(state_tensor)
        grasp_map_logits = logits[:, 0, :, :] # (1, H, W)

        # --- 3. ACT ---
        if epsilon > EPSILON_END:
            epsilon -= (EPSILON_START - EPSILON_END) / EPSILON_DECAY

        if np.random.rand() < epsilon:
            H, W = grasp_map_logits.shape[1], grasp_map_logits.shape[2]
            u = np.random.randint(0, H)
            v = np.random.randint(0, W)
            action_type = "RND"
        else:
            flat_idx = torch.argmax(grasp_map_logits).item()
            u = flat_idx // grasp_map_logits.shape[2]
            v = flat_idx % grasp_map_logits.shape[2]
            action_type = "NET"

        # --- 4. EXECUTE ---
        reward = env.step(u, v)
        
        # Track Metrics
        reward_history.append(reward)
        if len(reward_history) > 100: reward_history.pop(0) # Keep last 100
        success_rate = sum(reward_history) / len(reward_history) * 100.0

        rospy.loginfo(f"Ep {episode} | {action_type} @ ({u},{v}) | R: {reward} | Success Rate (100 avg): {success_rate:.1f}%")

        # --- 5. LEARN ---
        target_val = torch.tensor([[reward]], dtype=torch.float32).to(device)
        pred_val = grasp_map_logits[0, u, v].view(1, 1)
        
        loss = loss_fn(pred_val, target_val)
        
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        
        loss_history.append(loss.item())

        # --- 6. VISUALIZATION (Every 10 Steps) ---
        if episode % 10 == 0:
            torch.save(model.state_dict(), SAVE_PATH)
            
            # A. Prepare Input Image (RGB)
            # Permute (3,H,W) -> (H,W,3) and convert to BGR for OpenCV
            rgb_tensor = state.permute(1, 2, 0).cpu().numpy()
            rgb_img = cv2.cvtColor(rgb_tensor, cv2.COLOR_RGB2BGR)
            rgb_img = (rgb_img * 255).astype(np.uint8)

            # B. Prepare Heatmap (Prediction)
            prob_map = torch.sigmoid(grasp_map_logits[0]).detach().cpu().numpy()
            heatmap_img = (prob_map * 255).astype(np.uint8)
            heatmap_color = cv2.applyColorMap(heatmap_img, cv2.COLORMAP_JET)

            # C. Draw Action on BOTH (Circle where we clicked)
            # Green = Success, Blue = Fail
            color = (0, 255, 0) if reward > 0.5 else (255, 0, 0) 
            
            # Note: v is x-axis (col), u is y-axis (row)
            cv2.circle(rgb_img, (v, u), 2, color, -1)
            cv2.circle(heatmap_color, (v, u), 2, color, -1)

            # D. Combine Side-by-Side
            # Input Left, Heatmap Right
            combined_img = np.hstack([rgb_img, heatmap_color])
            
            # Scale up for easy viewing (2x size)
            combined_img = cv2.resize(combined_img, (0,0), fx=3.0, fy=3.0, interpolation=cv2.INTER_NEAREST)

            # E. Add Text Stats
            text = f"Ep: {episode} | Loss: {loss.item():.4f} | Success: {success_rate:.1f}% | Eps: {epsilon:.2f}"
            cv2.putText(combined_img, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            if reward > 0.5:
                cv2.putText(combined_img, "SUCCESS", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            else:
                cv2.putText(combined_img, "FAIL", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            cv2.imshow("Training Cockpit", combined_img)
            cv2.waitKey(1)

    rospy.loginfo("Training Complete.")
    cv2.destroyAllWindows()

if __name__ == "__main__":
    train_system()