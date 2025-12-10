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

from tossingbot.env.rotational_env import RotationalEnv
from tossingbot.learning.network import TossingBot
from tossingbot.learning.buffer import ReplayBuffer
from tossingbot.perception.rotation_transform import RotationTransform

# --- CONFIG ---
LEARNING_RATE = 1e-4
BATCH_SIZE = 8
NUM_ROTATIONS = 4 

# FILES
OLD_WEIGHTS = "tossingbot_manual.pth"  # Start smart
NEW_WEIGHTS = "tossingbot_auto.pth"    # Save here

# CONSTANTS
TRAIN_INTERVAL = 1
GRADIENT_STEPS = 1

class AutoTrainer:
    def __init__(self):
        # 1. Init
        self.env = RotationalEnv()
        self.transformer = RotationTransform()
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        rospy.loginfo(f"Auto Trainer on: {self.device}")

        # 2. Weights
        rp = rospkg.RosPack()
        self.weights_dir = os.path.join(rp.get_path('tossingbot_system'), "weights")
        self.old_path = os.path.join(self.weights_dir, OLD_WEIGHTS)
        self.new_path = os.path.join(self.weights_dir, NEW_WEIGHTS)

        self.model = TossingBot(input_channels=3).to(self.device)
        
        # Load Logic
        if os.path.exists(self.new_path):
            rospy.loginfo(f"Resuming Auto Training: {self.new_path}")
            self.model.load_state_dict(torch.load(self.new_path))
        elif os.path.exists(self.old_path):
            rospy.loginfo(f"Bootstrapping from Manual: {self.old_path}")
            self.model.load_state_dict(torch.load(self.old_path))
        else:
            rospy.loginfo("Starting Fresh.")

        self.model.train()
        self.optimizer = optim.Adam(self.model.parameters(), lr=LEARNING_RATE)
        self.loss_fn = nn.BCEWithLogitsLoss()
        self.buffer = ReplayBuffer(capacity=2000)
        
        self.step_count = 0
        self.current_loss = 0.0
        
        cv2.namedWindow("Auto Brain", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Auto Brain", 1200, 400) # Wide layout

    def run(self):
        rospy.loginfo("--- STARTING SELF-SUPERVISED LOOP ---")
        
        while not rospy.is_shutdown():
            state = self.env.get_observation()
            if state is None: rospy.sleep(0.1); continue
            
            state_gpu = state.unsqueeze(0).to(self.device) 

            # 1. FORWARD PASS
            with torch.no_grad():
                # We need full_vol for decision, and others for vis
                full_vol, rotated_inputs, raw_heatmaps = self.forward_multi_view(state_gpu)
                
                # Apply Mask (Ignore corners)
                full_vol = self.apply_workspace_mask(full_vol)

            # 2. SELECT ACTION (Greedy Argmax)
            # Find the best pixel across ALL rotations and ALL locations
            flat_idx = torch.argmax(full_vol).item()
            
            H, W = full_vol.shape[1:]
            rot_idx = flat_idx // (H * W)
            rem = flat_idx % (H * W)
            u = rem // W
            v = rem % W
            
            conf = torch.sigmoid(full_vol[rot_idx, u, v]).item()

            angle_deg = self.env.rot_helper.get_angle(rot_idx)
            rospy.loginfo(f"Step {self.step_count} | Action: Rot {rot_idx} ({angle_deg:.0f}) @ ({u},{v}) | Conf: {conf:.2f}")
            
            # --- VISUALIZE BEFORE ACTING ---
            self.visualize_dashboard(state, rotated_inputs, raw_heatmaps, rot_idx, u, v, conf)
            
            # 3. EXECUTE
            reward = self.env.step(u, v, rot_idx)
            
            # 4. STORE & LEARN
            self.buffer.push_rotational(state, u, v, rot_idx, reward)
            
            if self.step_count % TRAIN_INTERVAL == 0 and len(self.buffer) > BATCH_SIZE:
                self.train_burst()
                if self.step_count % 25 == 0:
                    torch.save(self.model.state_dict(), self.new_path)
            
            self.env.reset()
            self.step_count += 1

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

    def apply_workspace_mask(self, heatmap_vol):
        """Zero out corners to prevent hallucination."""
        C, H, W = heatmap_vol.shape
        cy, cx = H // 2, W // 2
        radius = min(H, W) // 2 - 2
        Y, X = np.ogrid[:H, :W]
        dist = np.sqrt((X - cx)**2 + (Y - cy)**2)
        mask = torch.from_numpy(dist <= radius).float().to(self.device)
        return heatmap_vol * mask + (1 - mask) * -100.0

    def train_burst(self):
        total_loss = 0.0
        for _ in range(GRADIENT_STEPS):
            batch = self.buffer.sample(BATCH_SIZE)
            
            b_states = [x[0] for x in batch]; b_u = [x[1] for x in batch]
            b_v = [x[2] for x in batch]; b_rot = [x[3] for x in batch]
            b_rew = torch.tensor([x[4] for x in batch], dtype=torch.float32).to(self.device).unsqueeze(1)

            rotated_imgs = []; rotated_pixels = [] 
            for i in range(BATCH_SIZE):
                state = b_states[i].to(self.device)
                rot_idx = b_rot[i]
                u_orig, v_orig = b_u[i], b_v[i]
                angle = self.env.rot_helper.get_angle(rot_idx)
                
                rot_img = self.transformer.to_gripper_frame(state, angle)
                rotated_imgs.append(rot_img)
                
                H, W = state.shape[1], state.shape[2]
                u_n, v_n = self.transformer.rotate_pixel(u_orig, v_orig, angle, H, W, to_gripper_frame=True)
                rotated_pixels.append((u_n, v_n))

            tensor_input = torch.stack(rotated_imgs) 
            self.optimizer.zero_grad()
            logits = self.model(tensor_input)
            
            pred_vals = []
            for i in range(BATCH_SIZE):
                val = logits[i, 0, rotated_pixels[i][0], rotated_pixels[i][1]]
                pred_vals.append(val)
            pred_vals = torch.stack(pred_vals).unsqueeze(1)

            loss = self.loss_fn(pred_vals, b_rew)
            loss.backward()
            self.optimizer.step()
            total_loss += loss.item()
            
        self.current_loss = total_loss / GRADIENT_STEPS

    def _tensor_to_cv(self, tensor_img):
        img = tensor_img.permute(1, 2, 0).cpu().numpy()
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    def visualize_dashboard(self, state, rotated_inputs, raw_heatmaps, chosen_rot, u, v, conf):
        """
        Shows 4 Pairs side-by-side: [ Input 0 | Map 0 ] [ Input 45 | Map 45 ] ...
        """
        pairs = []
        for i in range(NUM_ROTATIONS):
            # Input
            input_cv = self._tensor_to_cv(rotated_inputs[i])
            
            # Heatmap
            h = raw_heatmaps[i].squeeze(0).cpu().numpy()
            h = 1.0 / (1.0 + np.exp(-h))
            h_img = (h * 255).astype(np.uint8)
            heatmap_cv = cv2.applyColorMap(h_img, cv2.COLORMAP_JET)

            if heatmap_cv.shape[:2] != input_cv.shape[:2]:
                heatmap_cv = cv2.resize(heatmap_cv, (input_cv.shape[1], input_cv.shape[0]), interpolation=cv2.INTER_NEAREST)
            
            # Draw Gripper Line
            H, W, _ = input_cv.shape
            cx, cy = W // 2, H // 2
            cv2.line(input_cv, (cx, cy-15), (cx, cy+15), (0, 255, 0), 2)

            pair = np.hstack([input_cv, heatmap_cv])

            label = f"Rot {i*45}"
            if i == chosen_rot:
                # Highlight Selected Action
                cv2.rectangle(pair, (0,0), (pair.shape[1]-1, pair.shape[0]-1), (0,255,0), 3)
                label += " (ACT)"
                
                # Draw the specific pixel picked on the rotated view
                # We need to map world (u,v) -> rotated (u_n, v_n) to show it here
                angle = self.env.rot_helper.get_angle(i)
                u_n, v_n = self.transformer.rotate_pixel(u, v, angle, H, W, to_gripper_frame=True)
                cv2.circle(input_cv, (v_n, u_n), 3, (0,0,255), -1) # Red Dot on Input
                cv2.circle(heatmap_cv, (v_n, u_n), 3, (255,255,255), -1) # White Dot on Map
                
                # Re-stack because we drew on them
                pair = np.hstack([input_cv, heatmap_cv])
            
            cv2.putText(pair, label, (10, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255), 1)
            pairs.append(pair)

        # Layout: Horizontal Strip
        strip = np.hstack(pairs)
        final = cv2.resize(strip, (0,0), fx=2.0, fy=2.0, interpolation=cv2.INTER_NEAREST)
        
        cv2.putText(final, f"Step: {self.step_count} Loss: {self.current_loss:.4f} Conf: {conf:.2f}", (10, final.shape[0]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

        cv2.imshow("Auto Brain", final)
        cv2.waitKey(1)

if __name__ == "__main__":
    AutoTrainer().run()