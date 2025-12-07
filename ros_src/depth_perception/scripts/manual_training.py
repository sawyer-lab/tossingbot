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
SAVE_PATH = "tossingbot_manual.pth"
BATCH_SIZE = 8 # Train on current click + 7 past clicks
DISPLAY_SIZE = (1024, 512) # Window Size

class UnlimitedBuffer:
    def __init__(self):
        self.memory = []
    
    def push(self, state, u, v, reward):
        # Store on CPU to save VRAM
        self.memory.append((state.cpu(), u, v, float(reward)))

    def sample_batch(self, size):
        if len(self.memory) < size:
            return self.memory # Return what we have
        return random.sample(self.memory, size)

    def __len__(self):
        return len(self.memory)

class ManualTrainer:
    def __init__(self):
        rospy.init_node("manual_trainer")
        
        # 1. Setup Env & Network
        self.env = GazeboEnv()
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        self.model = TossingBot().to(self.device)
        if os.path.exists(SAVE_PATH):
            rospy.loginfo(f"Loaded existing weights: {SAVE_PATH}")
            self.model.load_state_dict(torch.load(SAVE_PATH))
        self.model.train() # Enable Gradients
        
        self.optimizer = optim.Adam(self.model.parameters(), lr=LEARNING_RATE)
        self.loss_fn = nn.BCEWithLogitsLoss()
        
        self.buffer = UnlimitedBuffer()
        
        # 2. UI State
        self.pending_action = None # Stores (u, v) when user clicks
        self.last_reward = None
        self.step_count = 0
        self.current_loss = 0.0
        
        cv2.namedWindow("Manual Training", cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("Manual Training", self.mouse_cb)

        rospy.loginfo("--- READY ---")
        rospy.loginfo("1. CLICK on the object to Grasp.")
        rospy.loginfo("2. Watch the Heatmap update after the grasp.")

    def mouse_cb(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # We map the mouse click later inside the loop
            # because we need the current image dimensions
            self.pending_action = (x, y)

    def run(self):
        while not rospy.is_shutdown():
            # -----------------------------------------------------------
            # 1. Get Observation & Prediction
            # -----------------------------------------------------------
            state = self.env.get_observation() # (3, H, W) Tensor
            
            if state is None:
                rospy.sleep(0.1)
                continue

            # Forward Pass (Get current heatmap)
            state_gpu = state.unsqueeze(0).to(self.device)
            with torch.no_grad():
                logits = self.model(state_gpu)
                grasp_map = logits[0, 0, :, :] # (H, W)
                prob_map = torch.sigmoid(grasp_map).cpu().numpy()

            # -----------------------------------------------------------
            # 2. Check for User Click
            # -----------------------------------------------------------
            if self.pending_action is not None:
                # Convert Screen (x,y) -> Grid (v, u)
                screen_x, screen_y = self.pending_action
                H, W = grasp_map.shape
                
                # Calculate scale based on the display size
                # We render visualization at 2x or 3x, so we must map back
                # NOTE: We do this mapping inside visualize() usually, 
                # but here we need to reverse-engineer the display logic.
                
                # Simplified: The display is constructed from RGB (W) + Heatmap (W)
                # So the total width is 2*W. 
                # If user clicks, we assume the visualization logic below:
                
                # Let's handle the Logic Step first, then Render.
                # Just wait for the visualization loop to define the scale factors
                pass 

            # -----------------------------------------------------------
            # 3. Visualization & Interaction Loop
            # -----------------------------------------------------------
            # Prepare RGB
            rgb_tensor = state.permute(1, 2, 0).cpu().numpy()
            rgb_img = cv2.cvtColor(rgb_tensor, cv2.COLOR_RGB2BGR)
            rgb_img = (rgb_img * 255).astype(np.uint8)
            
            # Prepare Heatmap
            heatmap_img = (prob_map * 255).astype(np.uint8)
            heatmap_img = cv2.resize(heatmap_img, (rgb_img.shape[1], rgb_img.shape[0]), interpolation=cv2.INTER_NEAREST)
            heatmap_color = cv2.applyColorMap(heatmap_img, cv2.COLORMAP_JET)

            # Combine Side-by-Side
            combined = np.hstack([rgb_img, heatmap_color])
            
            # Scale up for easy clicking (e.g., 3x bigger)
            SCALE = 3
            display_img = cv2.resize(combined, (0,0), fx=SCALE, fy=SCALE, interpolation=cv2.INTER_NEAREST)

            # --- PROCESS CLICK ---
            if self.pending_action:
                click_x, click_y = self.pending_action
                self.pending_action = None # Reset
                
                # Map Click back to Grid
                # Image is [RGB | Heatmap]
                # If click is on RGB side:
                real_x = int(click_x / SCALE)
                real_y = int(click_y / SCALE)
                
                img_w = rgb_img.shape[1]
                
                # If user clicked on heatmap side, shift x back
                if real_x >= img_w:
                    real_x -= img_w
                
                # Clamp
                v = np.clip(real_x, 0, img_w - 1) # Col
                u = np.clip(real_y, 0, rgb_img.shape[0] - 1) # Row
                
                # --- EXECUTE ROBOT ---
                rospy.loginfo(f"Executing Manual Action at Grid ({u}, {v})")
                reward = self.env.step(u, v)
                self.last_reward = reward
                
                # --- TRAIN NETWORK ---
                self.buffer.push(state, u, v, reward)
                self.train_step(state, u, v, reward)
                self.step_count += 1
                
                # Save weights
                if self.step_count % 10 == 0:
                    torch.save(self.model.state_dict(), SAVE_PATH)

                # --- CRITICAL FIX: RESET THE SCENE ---
                rospy.loginfo("Resetting Object...")
                self.env.reset() 
                
                # Sleep briefly to make sure the camera sees the NEW banana location
                # before the next loop starts
                rospy.sleep(0.5)

            # --- DRAW UI OVERLAY ---
            if self.last_reward is not None:
                msg = "LAST: SUCCESS" if self.last_reward > 0.5 else "LAST: FAIL"
                col = (0, 255, 0) if self.last_reward > 0.5 else (0, 0, 255)
                cv2.putText(display_img, msg, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, col, 2)

            info = f"Steps: {self.step_count} | Loss: {self.current_loss:.4f} | Buffer: {len(self.buffer)}"
            cv2.putText(display_img, info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            cv2.imshow("Manual Training", display_img)
            key = cv2.waitKey(10)
            if key == ord('q'): break

    def train_step(self, current_state, u, v, reward):
        """Trains on a batch including the most recent action"""
        if len(self.buffer) < 1: return

        # 1. Create Batch
        # Always take the current action (so we learn immediately)
        # Plus random samples from history
        mini_batch = self.buffer.sample_batch(BATCH_SIZE - 1)
        mini_batch.append((current_state.cpu(), u, v, float(reward)))
        
        # 2. Stack Tensors
        b_states = torch.stack([x[0] for x in mini_batch]).to(self.device)
        b_u = [x[1] for x in mini_batch]
        b_v = [x[2] for x in mini_batch]
        b_rewards = torch.tensor([x[3] for x in mini_batch], dtype=torch.float32).to(self.device).unsqueeze(1)

        # 3. Forward Pass
        # Important: We must re-run forward pass on batch to get gradient graph
        self.optimizer.zero_grad()
        logits = self.model(b_states) # (B, 1, H, W)
        
        # 4. Masked Loss
        # Extract only the predicted values at the executed pixels
        pred_values = []
        for i in range(len(mini_batch)):
            val = logits[i, 0, b_u[i], b_v[i]]
            pred_values.append(val)
        pred_values = torch.stack(pred_values).unsqueeze(1) # (B, 1)

        # 5. Backprop
        loss = self.loss_fn(pred_values, b_rewards)
        loss.backward()
        self.optimizer.step()
        
        self.current_loss = loss.item()
        rospy.loginfo(f"TRAINED. Loss: {self.current_loss:.4f}")

if __name__ == "__main__":
    trainer = ManualTrainer()
    trainer.run()