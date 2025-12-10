#!/usr/bin/env python3.8
import sys
import os
import torch
import torch.optim as optim
import torch.nn as nn
import numpy as np
import cv2
import rospy
import rospkg
import torchvision.transforms.functional as TF

from tossingbot import config
from tossingbot.env.rotational_env import RotationalEnv
from tossingbot.learning.network import TossingBot
from tossingbot.learning.buffer import ReplayBuffer
from tossingbot.perception.rotation_transform import RotationTransform


# --- CONFIG ---
LEARNING_RATE = 2e-4
BATCH_SIZE = 8
NUM_ROTATIONS = 4 
GRADIENT_STEPS = 10   
SAVE_FILE = "tossingbot_rotational_invariant.pth"

class ManualRotationalTrainer:
    def __init__(self):
        self.env = RotationalEnv()
        self.transformer = RotationTransform()
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        rospy.loginfo(f"Manual Trainer on: {self.device}")

        rp = rospkg.RosPack()
        self.save_path = os.path.join(rp.get_path('tossingbot_system'), "weights", SAVE_FILE)
        
        self.model = TossingBot(input_channels=3).to(self.device)
        
        if os.path.exists(self.save_path):
            rospy.loginfo(f"Loading weights: {self.save_path}")
            self.model.load_state_dict(torch.load(self.save_path))
        else:
            rospy.loginfo("Starting Fresh.")

        self.model.train()
        self.optimizer = optim.Adam(self.model.parameters(), lr=LEARNING_RATE)
        self.loss_fn = nn.BCEWithLogitsLoss()
        self.buffer = ReplayBuffer(capacity=500)
        
        self.step_count = 0
        self.current_loss = 0.0
        
        # --- STATE ---
        self.selected_rot_idx = 0  # <--- THE MASTER INDEX
        self.pending_click = None
        self.last_reward = None
        
        # Simple Zoom for Display (No resizing tricks)
        self.ZOOM = 4.0 
        
        cv2.namedWindow("Manual Calibration", cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("Manual Calibration", self._mouse_cb)

    def _mouse_cb(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.pending_click = (x, y)

    def run(self):
        rospy.loginfo("--- MANUAL ROTATION MODE ---")
        rospy.loginfo(f"Grid Size: {config.IMG_H}x{config.IMG_W}")
        rospy.loginfo("Press 'r' to cycle rotation. Click Left Image to execute.")
        
        while not rospy.is_shutdown():
            state = self.env.get_observation()
            if state is None: rospy.sleep(0.1); continue
            
            state_gpu = state.unsqueeze(0).to(self.device)

            with torch.no_grad():
                # We need these for visualization only
                full_vol, rotated_inputs, raw_heatmaps = self.forward_multi_view(state_gpu)

            # --- INPUT HANDLING ---
            key = cv2.waitKey(20) & 0xFF
            if key == ord('r'):
                # Cycle Index: 0 -> 1 -> 2 -> 3 -> 0
                self.selected_rot_idx = (self.selected_rot_idx + 1) % NUM_ROTATIONS
                print(f"Selected Rotation Index: {self.selected_rot_idx}")
                
            elif key == ord('q'):
                break

            if self.pending_click:
                win_x, win_y = self.pending_click
                self.pending_click = None 
                
                # --- 1. COORDINATE MAPPING ---
                # UI Layout: [ Main View ] [ Net Input ] [ Net Output ]
                # Each panel width = IMG_W * ZOOM
                panel_width = int(config.IMG_W * self.ZOOM)
                panel_height = int(config.IMG_H * self.ZOOM)

                # Ignore clicks outside the Global View (Left Panel)
                if win_x < panel_width and win_y < panel_height:
                    
                    v = int(win_x / self.ZOOM)
                    u = int(win_y / self.ZOOM)
                    
                    # Clamp
                    v = min(max(v, 0), config.IMG_W - 1)
                    u = min(max(u, 0), config.IMG_H - 1)
                    
                    # USE THE SELECTED INDEX
                    rot_idx = self.selected_rot_idx
                    
                    rospy.loginfo(f"CLICK -> Grid({u},{v}) | Rotation Index: {rot_idx}")
                    
                    # --- 2. VISUAL CONFIRMATION ---
                    self.visualize_manual(state, rotated_inputs, raw_heatmaps, click_marker=(v, u))
                    cv2.waitKey(200) 
                    
                    # --- 3. EXECUTE ---
                    # Pass the exact index to the environment
                    reward = self.env.step(u, v, rot_idx)
                    self.last_reward = reward
                    
                    # Store & Learn
                    self.buffer.push_rotational(state, u, v, rot_idx, reward)
                    self.train_burst()
                    
                    if self.step_count % 5 == 0:
                        torch.save(self.model.state_dict(), self.save_path)
                    
                    self.env.reset()
                    self.step_count += 1

            self.visualize_manual(state, rotated_inputs, raw_heatmaps)

    def forward_multi_view(self, state_tensor):
        batch_rotated = []
        for i in range(NUM_ROTATIONS):
            angle = -self.env.rot_helper.get_angle(i) 
            rot_img = self.transformer.to_gripper_frame(state_tensor, angle)
            batch_rotated.append(rot_img)
        
        rotated_inputs_vol = torch.cat(batch_rotated, dim=0)
        batch_out = self.model(rotated_inputs_vol)
        
        final_maps = []
        for i in range(NUM_ROTATIONS):
            angle = self.env.rot_helper.get_angle(i)
            unrot_map = self.transformer.to_world_frame(batch_out[i], angle)
            final_maps.append(unrot_map)
            
        return torch.cat(final_maps, dim=0), rotated_inputs_vol, batch_out

    def train_burst(self):
        total_loss = 0.0
        
        # Guard: Don't train if buffer is empty
        if len(self.buffer) == 0:
            return

        for _ in range(GRADIENT_STEPS):
            # Sample (Safely handle small buffers)
            sample_size = min(len(self.buffer), BATCH_SIZE)
            batch = self.buffer.sample(sample_size)
            
            # GUARD: If sampling returned empty list, skip
            if not batch:
                continue

            b_states = [x[0] for x in batch]; b_u = [x[1] for x in batch]
            b_v = [x[2] for x in batch]; b_rot = [x[3] for x in batch]
            b_rew = torch.tensor([x[4] for x in batch], dtype=torch.float32).to(self.device).unsqueeze(1)

            rotated_imgs = []; rotated_pixels = [] 
            for i in range(len(batch)):
                state = b_states[i].to(self.device)
                
                # USE THE INDEX FROM BUFFER
                rot_idx = b_rot[i]
                angle = self.env.rot_helper.get_angle(rot_idx)
                
                rot_img = self.transformer.to_gripper_frame(state, angle)
                rotated_imgs.append(rot_img)
                
                H, W = state.shape[1], state.shape[2]
                u_n, v_n = self.transformer.rotate_pixel(b_u[i], b_v[i], angle, H, W, to_gripper_frame=True)
                rotated_pixels.append((u_n, v_n))

            # Safety check before stacking
            if len(rotated_imgs) == 0: continue

            tensor_input = torch.stack(rotated_imgs) 
            self.optimizer.zero_grad()
            logits = self.model(tensor_input)
            
            pred_vals = []
            for i in range(len(batch)):
                u_p, v_p = rotated_pixels[i]
                
                # Clamp coordinates to be inside heatmap
                H_out, W_out = logits.shape[2:]
                u_p = min(max(u_p, 0), H_out - 1)
                v_p = min(max(v_p, 0), W_out - 1)
                
                val = logits[i, 0, u_p, v_p]
                pred_vals.append(val)
            pred_vals = torch.stack(pred_vals).unsqueeze(1)

            loss = self.loss_fn(pred_vals, b_rew)
            loss.backward()
            self.optimizer.step()
            total_loss += loss.item()
            
        self.current_loss = total_loss / GRADIENT_STEPS
        rospy.loginfo(f"TRAINED. Loss: {self.current_loss:.4f}")

    def _tensor_to_cv(self, tensor_img):
        img = tensor_img.permute(1, 2, 0).cpu().numpy()
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    def visualize_manual(self, state, rotated_inputs, raw_heatmaps, click_marker=None):
        # USE THE MASTER INDEX
        idx = self.selected_rot_idx
        angle_deg = self.env.rot_helper.get_angle(idx)

        # 1. Main View (Global)
        main_rgb = self._tensor_to_cv(state)
        main_disp = cv2.resize(main_rgb, (0,0), fx=self.ZOOM, fy=self.ZOOM, interpolation=cv2.INTER_NEAREST)
        
        # Dimensions
        H_disp, W_disp, _ = main_disp.shape
        cx, cy = W_disp // 2, H_disp // 2
        
        # Draw Orientation Arrow (Matches Robot Yaw)
        # Yaw is inverted relative to image space, so we use -angle
        angle_rad = np.deg2rad(-angle_deg) 
        end_x = int(cx + 40 * np.cos(angle_rad))
        end_y = int(cy + 40 * np.sin(angle_rad))
        cv2.arrowedLine(main_disp, (cx, cy), (end_x, end_y), (0, 0, 255), 3)
        
        if click_marker:
            v, u = click_marker
            dx, dy = int(v * self.ZOOM), int(u * self.ZOOM)
            cv2.circle(main_disp, (dx, dy), 5, (255, 0, 255), -1)

        # 2. Net Input (What the network sees for THIS index)
        net_input = self._tensor_to_cv(rotated_inputs[idx])
        net_disp = cv2.resize(net_input, (W_disp, H_disp), interpolation=cv2.INTER_NEAREST)
        
        # 3. Net Output (Raw heatmap for THIS index)
        h = raw_heatmaps[idx].squeeze(0).cpu().numpy()
        h = 1.0 / (1.0 + np.exp(-h))
        h_img = (h * 255).astype(np.uint8)
        heatmap_cv = cv2.applyColorMap(h_img, cv2.COLORMAP_JET)
        heatmap_disp = cv2.resize(heatmap_cv, (W_disp, H_disp), interpolation=cv2.INTER_NEAREST)

        # Layout
        row = np.hstack([main_disp, net_disp, heatmap_disp])
        
        # Labels
        cv2.putText(row, f"ROT IDX: {idx} ({angle_deg:.0f} deg)", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.putText(row, "GLOBAL VIEW", (10, H_disp-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
        cv2.putText(row, "NET INPUT (Rotated)", (W_disp+10, H_disp-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
        cv2.putText(row, "CONFIDENCE", (W_disp*2+10, H_disp-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

        if self.last_reward is not None:
             res = "SUCCESS" if self.last_reward > 0.5 else "FAIL"
             col = (0, 255, 0) if self.last_reward > 0.5 else (0, 0, 255)
             cv2.putText(row, f"LAST: {res}", (W_disp-150, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2)

        cv2.imshow("Manual Calibration", row)

if __name__ == "__main__":
    ManualRotationalTrainer().run()