#!/usr/bin/env python3.8
import sys
import os
import cv2
import time
import pickle
import numpy as np
import random

# --- PYTHON 3 COMPATIBILITY ---
sys.path.insert(0, '/geometry2_ws/devel/lib/python3/dist-packages')
sys.path.insert(0, '/cv_bridge_ws/devel/lib/python3/dist-packages')

import rospy
import rospkg
import torch
import torch.optim as optim
import torch.nn as nn
import torchvision.transforms.functional as TF

# --- IMPORTS ---
from tossingbot.environment.rotational_env import RotationalEnv 
from network import TossingBot_Modular
from buffer import PrioritizedReplayBuffer
from utils import RotationTransformer

# --- CONFIGURATION ---
LEARNING_RATE = 1e-4
MOMENTUM = 0.9
WEIGHT_DECAY = 2e-5
BATCH_SIZE = 8
BUFFER_CAPACITY = 2000
NUM_ROTATIONS = 4    
TRAIN_INTERVAL = 1    
GRADIENT_STEPS = 1    
SAVE_INTERVAL = 50    
SAVE_PATH = "tossingbot_auto.pth"

# Exploration: Start 100% Random to fill buffer
EXPLORE_START = 0.5   
EXPLORE_END = 0.1     
EXPLORE_STEPS = 15000

class TossingNode:
    def __init__(self):
        rospy.init_node('tossingbot_brain')
        
        # 1. SETUP HARDWARE
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        rospy.loginfo(f"🟢 BRAIN INITIALIZED ON: {self.device}")
        
        # 2. INIT COMPONENTS
        self.env = RotationalEnv(num_rotations=NUM_ROTATIONS)
        self.transformer = RotationTransformer(device=self.device)
        self.buffer = PrioritizedReplayBuffer(capacity=BUFFER_CAPACITY)

        # 3. NETWORK
        self.model = TossingBot_Modular(input_channels=4).to(self.device)
        self.optimizer = optim.SGD(self.model.parameters(), 
                                   lr=LEARNING_RATE, 
                                   momentum=MOMENTUM, 
                                   weight_decay=WEIGHT_DECAY)
        self.loss_fn = nn.BCEWithLogitsLoss(reduction='none') 

        # 4. PATHS
        rp = rospkg.RosPack()
        self.save_dir = os.path.join(rp.get_path('tossingbot_system'), "weights")
        self.weights_path = os.path.join(self.save_dir, SAVE_PATH)
        
        # 5. STATE
        self.step_count = 0
        self.current_loss = 0.0
        
        self.load_snapshot()
        
        # 6. VIZ WINDOW
        cv2.namedWindow("Auto Brain", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Auto Brain", 1200, 800)

    def get_epsilon(self):
        if self.step_count >= EXPLORE_STEPS: return EXPLORE_END
        fraction = float(self.step_count) / float(EXPLORE_STEPS)
        return EXPLORE_START - fraction * (EXPLORE_START - EXPLORE_END)

    def run(self):
        rospy.loginfo("🚀 STARTING SELF-SUPERVISED LOOP...")
        
        self.env.force_neutral()
        state_tensor = self.env.reset_episode(first_run=True)
        
        while not rospy.is_shutdown():
            if state_tensor is None:
                 state_tensor = self.env.get_observation()
                 if state_tensor is None: rospy.sleep(0.1); continue

            # Move to GPU
            state_gpu = state_tensor.to(self.device)

            # --- 1. FORWARD PASS (Multi-View) ---
            with torch.no_grad():
                # Get predictions for all rotations
                full_vol, rotated_inputs, raw_heatmaps = self.forward_multi_view(state_gpu)
                
            # --- 2. SELECT ACTION ---
            epsilon = self.get_epsilon()
            H, W = full_vol.shape[1:]
            
            if random.random() < epsilon:
                # Explore
                action_type = "🎲 RND"
                rot_idx = random.randint(0, NUM_ROTATIONS - 1)
                margin = 15
                u_rot = random.randint(margin, H - margin - 1) 
                v_rot = random.randint(margin, W - margin - 1)
                conf = 0.0
            else:
                # Exploit
                action_type = "🧠 NET"
                # Argmax over (Rot, H, W) volume
                flat_idx = torch.argmax(full_vol).item()
                rot_idx = flat_idx // (H * W)
                rem = flat_idx % (H * W)
                u_rot = rem // W
                v_rot = rem % W
                conf = torch.sigmoid(full_vol[rot_idx, u_rot, v_rot]).item()

            angle_deg = self.env.rot_helper.get_angle(rot_idx)
            
            # --- 3. VISUALIZE ---
            self.visualize_dashboard(state_gpu, rotated_inputs, raw_heatmaps, 
                                     rot_idx, u_rot, v_rot, conf)

            # --- 4. EXECUTE (Coordinate Transform) ---
            # Map selected pixel in Rotated Frame -> World Frame
            u_world, v_world = self.transformer.rotate_pixel(
                u_rot, v_rot, angle_deg, H, W, to_gripper_frame=False
            )
            
            print("-" * 50)
            print(f"[STEP {self.step_count}] Mode: {action_type} (Eps: {epsilon:.2f})")
            print(f"   >>> Rot: {rot_idx} ({angle_deg:.1f}°) | Pixel: ({u_rot}, {v_rot})")
            
            reward = self.env.step(u_world, v_world, rot_idx)
            
            msg = "✅ SUCCESS" if reward > 0.5 else "❌ FAIL"
            print(f"   >>> Result: {msg} (Rew: {reward})")

            # --- 5. STORE ---
            self.buffer.push(state_tensor.cpu(), u_rot, v_rot, rot_idx, reward)

            # --- 6. TRAIN ---
            if self.step_count % TRAIN_INTERVAL == 0 and len(self.buffer) > BATCH_SIZE:
                self.train_burst()

            # --- 7. SAVE ---
            if self.step_count % SAVE_INTERVAL == 0 and self.step_count > 0:
                self.save_snapshot()

            self.step_count += 1
            state_tensor = self.env.reset_episode(previous_success=(reward > 0.5))

    def forward_multi_view(self, state_tensor):
        """Generates input batch by rotating the single state image."""
        batch_rotated = []
        for i in range(NUM_ROTATIONS):
            # ROTATION LOGIC:
            # To see what the gripper sees at angle X, we rotate image by -X
            angle = -self.env.rot_helper.get_angle(i)
            rot_img = self.transformer.to_gripper_frame(state_tensor, angle)
            batch_rotated.append(rot_img)
        
        # Stack into batch (N, C, H, W)
        rotated_inputs_vol = torch.stack(batch_rotated)
        
        # Forward Pass
        batch_out = self.model(rotated_inputs_vol)
        
        # Return Raw Logits Volume
        # We don't rotate back here because we want to select the action in the ROTATED frame
        return batch_out.squeeze(1), rotated_inputs_vol, batch_out

    def train_burst(self):
        total_loss = 0.0
        for _ in range(GRADIENT_STEPS):
            # Sample (State, u_rot, v_rot, rot_idx, reward)
            batch, indices, weights = self.buffer.sample(BATCH_SIZE)
            
            b_states = [x[0] for x in batch]
            b_u = [x[1] for x in batch]
            b_v = [x[2] for x in batch]
            b_rot = [x[3] for x in batch]
            b_rew = torch.tensor([x[4] for x in batch], dtype=torch.float32).to(self.device).unsqueeze(1)
            weights = torch.tensor(weights, dtype=torch.float32).to(self.device).unsqueeze(1)

            # Re-Generate the specific rotated view seen during collection
            rotated_imgs = []
            valid_pixels_u = []
            valid_pixels_v = []
            
            for i in range(len(batch)):
                st = b_states[i].to(self.device)
                rot_idx = b_rot[i]
                angle = self.env.rot_helper.get_angle(rot_idx)
                
                # Rotate image by -angle (to gripper frame)
                rot_img = self.transformer.to_gripper_frame(st, -angle)
                rotated_imgs.append(rot_img)
                
                valid_pixels_u.append(b_u[i])
                valid_pixels_v.append(b_v[i])

            tensor_input = torch.stack(rotated_imgs)
            
            self.optimizer.zero_grad()
            logits = self.model(tensor_input)
            
            # Gather specific pixel predictions
            pred_vals = []
            for i in range(len(batch)):
                u, v = valid_pixels_u[i], valid_pixels_v[i]
                # Clamp for safety
                u = min(max(u, 0), logits.shape[2]-1)
                v = min(max(v, 0), logits.shape[3]-1)
                pred_vals.append(logits[i, 0, u, v])
            
            pred_vals = torch.stack(pred_vals).unsqueeze(1)
            
            # Weighted Loss (PER)
            loss = self.loss_fn(pred_vals, b_rew) * weights
            loss = loss.mean()
            
            loss.backward()
            self.optimizer.step()
            
            # Update Priorities
            errors = torch.abs(pred_vals - b_rew).detach().cpu().numpy()
            self.buffer.update_priorities(indices, errors)
            
            total_loss += loss.item()
            
        self.current_loss = total_loss / GRADIENT_STEPS

    def _tensor_to_cv(self, tensor_img):
        # (C, H, W) -> (H, W, C)
        # Take only first 3 channels (RGB) for viz, ignore Depth
        img = tensor_img[:3, :, :].permute(1, 2, 0).detach().cpu().numpy()
        # Normalize if necessary (assuming 0-1 float)
        img = np.clip(img * 255, 0, 255).astype(np.uint8)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    def visualize_dashboard(self, state, rotated_inputs, raw_heatmaps, chosen_rot, u_rot, v_rot, conf):
        # 1. TOP ROW: Only show 4 samples to keep it fast (0, 90, 180, 270 approx)
        pairs = []
        indices_to_show = np.linspace(0, NUM_ROTATIONS-1, 4, dtype=int)
        
        # Always include the chosen one if not in list
        if chosen_rot not in indices_to_show:
            indices_to_show[-1] = chosen_rot
            indices_to_show.sort()

        for i in indices_to_show:
            input_cv = self._tensor_to_cv(rotated_inputs[i])
            
            h = raw_heatmaps[i].squeeze(0).squeeze(0).cpu().numpy()
            h = 1.0 / (1.0 + np.exp(-h)) # Sigmoid
            h_img = (h * 255).astype(np.uint8)
            heatmap_cv = cv2.applyColorMap(h_img, cv2.COLORMAP_JET)
            
            # Mark Dot
            if i == chosen_rot:
                cv2.circle(input_cv, (v_rot, u_rot), 4, (0, 0, 255), -1)
                cv2.rectangle(input_cv, (0,0), (input_cv.shape[1], input_cv.shape[0]), (0,255,0), 2)
            
            pair = np.hstack([input_cv, heatmap_cv])
            pairs.append(pair)

        top_row = np.hstack(pairs)
        
        # 2. BOTTOM: World View
        world_rgb = self._tensor_to_cv(state)
        H, W, _ = world_rgb.shape
        
        angle = self.env.rot_helper.get_angle(chosen_rot)
        u_world, v_world = self.transformer.rotate_pixel(u_rot, v_rot, angle, H, W, to_gripper_frame=False)
        
        # Draw Target and Arrow
        cv2.circle(world_rgb, (v_world, u_world), 5, (0, 255, 0), -1)
        
        angle_rad = np.deg2rad(-angle) # Visual arrow angle
        end_x = int(v_world + 40 * np.cos(angle_rad))
        end_y = int(u_world + 40 * np.sin(angle_rad))
        cv2.arrowedLine(world_rgb, (v_world, u_world), (end_x, end_y), (0, 0, 255), 2)
        


        # Scale bottom to match top width
        scale = top_row.shape[1] / world_rgb.shape[1]
        new_h = int(world_rgb.shape[0] * scale)
        bottom_row = cv2.resize(world_rgb, (top_row.shape[1], new_h))
        
        final_img = np.vstack([top_row, bottom_row])
        cv2.imshow("Auto Brain", final_img)
        cv2.waitKey(1)

    def save_snapshot(self):
        if not os.path.exists(self.save_dir): os.makedirs(self.save_dir)
        checkpoint = {
            'model_state': self.model.state_dict(),
            'optimizer_state': self.optimizer.state_dict(),
            'step_count': self.step_count,
        }
        torch.save(checkpoint, self.weights_path)
        try:
            with open(self.weights_path.replace(".pth", "_buffer.pkl"), 'wb') as f:
                pickle.dump(self.buffer, f)
            rospy.loginfo(f"💾 SAVED SNAPSHOT: Step {self.step_count}")
        except: pass

    def load_snapshot(self):
        if os.path.exists(self.weights_path):
            rospy.loginfo(f"🔄 LOADING: {self.weights_path}")
            try:
                ckpt = torch.load(self.weights_path, map_location=self.device)
                self.model.load_state_dict(ckpt['model_state'])
                self.optimizer.load_state_dict(ckpt['optimizer_state'])
                self.step_count = ckpt['step_count']
            except: pass
        
        buf_path = self.weights_path.replace(".pth", "_buffer.pkl")
        if os.path.exists(buf_path):
            try:
                with open(buf_path, 'rb') as f: self.buffer = pickle.load(f)
                rospy.loginfo(f"   >>> Buffer: {len(self.buffer)} items")
            except: pass

if __name__ == '__main__':
    TossingNode().run()