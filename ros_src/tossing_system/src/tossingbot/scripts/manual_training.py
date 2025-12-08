#!/usr/bin/env python3.8
import sys
import os
import torch
import torch.optim as optim
import torch.nn as nn
import numpy as np
import cv2
import rospy
import rospkg # <--- NEW IMPORT

# --- MODULAR IMPORTS ---
from tossingbot.env.gazebo_env import GazeboEnv
from tossingbot.learning.network import TossingBot
from tossingbot.learning.buffer import ReplayBuffer

# --- CONFIG ---
LEARNING_RATE = 1e-4
BATCH_SIZE = 8 
WEIGHTS_FILENAME = "tossingbot_manual.pth"

class ManualTrainer:
    def __init__(self):
        # 1. Init System
        self.env = GazeboEnv()
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        rospy.loginfo(f"Trainer on Device: {self.device}")

        # 2. SETUP PATHS (The Fix)
        rp = rospkg.RosPack()
        package_path = rp.get_path('tossingbot_system')
        
        # Create a 'weights' folder inside the package if it doesn't exist
        self.weights_dir = os.path.join(package_path, "weights")
        if not os.path.exists(self.weights_dir):
            os.makedirs(self.weights_dir)
            
        self.save_path = os.path.join(self.weights_dir, WEIGHTS_FILENAME)
        rospy.loginfo(f"Weights will be saved to: {self.save_path}")

        # 3. Init AI
        self.model = TossingBot().to(self.device)
        
        if os.path.exists(self.save_path):
            rospy.loginfo(f"Loading existing weights found at {self.save_path}")
            self.model.load_state_dict(torch.load(self.save_path))
        else:
            rospy.loginfo("No existing weights found. Starting fresh.")

        self.model.train() 

        self.optimizer = optim.Adam(self.model.parameters(), lr=LEARNING_RATE)
        self.loss_fn = nn.BCEWithLogitsLoss()
        self.buffer = ReplayBuffer(capacity=1000)

        # 4. GUI State
        self.pending_action = None
        self.display_scale = 3.0 
        self.last_reward = None
        self.step_count = 0
        self.current_loss = 0.0

        # 5. Setup Window
        cv2.namedWindow("Manual Trainer", cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("Manual Trainer", self._mouse_cb)

        rospy.loginfo("--- READY: Click on the Banana! ---")

    def _mouse_cb(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.pending_action = (x, y)

    def run(self):
        while not rospy.is_shutdown():
            # ==========================================
            # 1. OBSERVE & PREDICT
            # ==========================================
            state = self.env.get_observation()
            if state is None:
                rospy.sleep(0.1)
                continue

            state_gpu = state.unsqueeze(0).to(self.device)
            with torch.no_grad():
                logits = self.model(state_gpu)
                heatmap = logits[0, 0, :, :] 

            # ==========================================
            # 2. ACT (Human Input)
            # ==========================================
            if self.pending_action is not None:
                win_x, win_y = self.pending_action
                self.pending_action = None 
                
                # Unscale
                raw_x = int(win_x / self.display_scale)
                raw_y = int(win_y / self.display_scale)
                
                # Check bounds
                C, H, W = state.shape
                
                if raw_x >= W: raw_x -= W
                
                v = np.clip(raw_x, 0, W - 1) # Col
                u = np.clip(raw_y, 0, H - 1) # Row
                
                # Execute
                rospy.loginfo(f"Executing Manual Action: ({u}, {v})")
                reward = self.env.step(u, v)
                self.last_reward = reward
                
                # Store & Learn
                self.buffer.push(state, u, v, reward)
                self.train_step(state, u, v, reward)
                
                # Reset
                self.env.reset()
                self.step_count += 1
                
                # Checkpoint (Save to Absolute Path)
                if self.step_count % 10 == 0:
                    torch.save(self.model.state_dict(), self.save_path)
                    rospy.loginfo("Checkpoint Saved.")

            # ==========================================
            # 3. VISUALIZE
            # ==========================================
            self.draw_ui(state, heatmap)
            key = cv2.waitKey(20)
            if key == ord('q'): break

    def train_step(self, current_state, u, v, reward):
        if len(self.buffer) < 1: return

        n_sample = min(len(self.buffer), BATCH_SIZE - 1)
        mini_batch = self.buffer.sample(n_sample)
        mini_batch.append((current_state.cpu(), u, v, float(reward)))
        
        b_states = torch.stack([x[0] for x in mini_batch]).to(self.device)
        b_u = [x[1] for x in mini_batch]
        b_v = [x[2] for x in mini_batch]
        b_rewards = torch.tensor([x[3] for x in mini_batch], dtype=torch.float32).to(self.device).unsqueeze(1)

        self.optimizer.zero_grad()
        b_logits = self.model(b_states)
        
        pred_vals = []
        for i in range(len(mini_batch)):
            val = b_logits[i, 0, b_u[i], b_v[i]]
            pred_vals.append(val)
        pred_vals = torch.stack(pred_vals).unsqueeze(1)

        loss = self.loss_fn(pred_vals, b_rewards)
        loss.backward()
        self.optimizer.step()
        
        self.current_loss = loss.item()
        rospy.loginfo(f"TRAINED. Loss: {self.current_loss:.4f}")

    def draw_ui(self, state, heatmap):
        img_rgb = state.permute(1, 2, 0).cpu().numpy()
        img_rgb = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
        
        prob_map = torch.sigmoid(heatmap).cpu().numpy()
        img_heat = (prob_map * 255).astype(np.uint8)
        img_heat = cv2.applyColorMap(img_heat, cv2.COLORMAP_JET)
        
        if img_heat.shape[:2] != img_rgb.shape[:2]:
            img_heat = cv2.resize(img_heat, (img_rgb.shape[1], img_rgb.shape[0]), interpolation=cv2.INTER_NEAREST)

        display = np.hstack([img_rgb, img_heat])
        display = cv2.resize(display, (0,0), fx=self.display_scale, fy=self.display_scale, interpolation=cv2.INTER_NEAREST)
        
        info = f"Steps: {self.step_count} | Loss: {self.current_loss:.3f} | Buf: {len(self.buffer)}"
        cv2.putText(display, info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        if self.last_reward is not None:
            msg = "SUCCESS" if self.last_reward > 0.5 else "FAIL"
            col = (0, 255, 0) if self.last_reward > 0.5 else (0, 0, 255)
            cv2.putText(display, msg, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, col, 2)

        cv2.imshow("Manual Trainer", display)

if __name__ == "__main__":
    trainer = ManualTrainer()
    trainer.run()