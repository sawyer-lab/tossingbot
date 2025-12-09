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

# --- IMPORTS ---
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
        
        self.selected_rot_idx = 0 
        self.pending_click = None
        self.last_reward = None
        
        # Display Config
        self.UI_SCALE = 3.0
        
        cv2.namedWindow("Manual Calibration", cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("Manual Calibration", self._mouse_cb)

    def _mouse_cb(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.pending_click = (x, y)

    def run(self):
        rospy.loginfo("--- MANUAL ROTATION MODE ---")
        
        while not rospy.is_shutdown():
            state = self.env.get_observation()
            if state is None: rospy.sleep(0.1); continue
            
            state_gpu = state.unsqueeze(0).to(self.device)

            with torch.no_grad():
                # Get views for visualization
                full_vol, rotated_inputs, raw_heatmaps = self.forward_multi_view(state_gpu)

            # --- INPUT HANDLING ---
            key = cv2.waitKey(20) & 0xFF
            if key == ord('r'):
                self.selected_rot_idx = (self.selected_rot_idx + 1) % NUM_ROTATIONS
                print(f"Selected Rotation: {self.env.rot_helper.get_angle(self.selected_rot_idx)} deg")
            elif key == ord('q'):
                break

            if self.pending_click:
                win_x, win_y = self.pending_click
                self.pending_click = None 
                
                # 1. Unscale Mouse Click
                # The UI is: [ RGB (W*Scale) | Heatmap (W*Scale) ]
                # We only care about clicks on the Left Half (RGB)
                
                real_x = int(win_x / self.UI_SCALE)
                real_y = int(win_y / self.UI_SCALE)
                
                H, W = state.shape[1], state.shape[2]
                
                # Ignore clicks outside the RGB image
                if real_x < W and real_y < H:
                    v = np.clip(real_x, 0, W - 1) # Col
                    u = np.clip(real_y, 0, H - 1) # Row
                    
                    rot_idx = self.selected_rot_idx
                    angle_deg = self.env.rot_helper.get_angle(rot_idx)
                    
                    # Debug Mapping
                    world_target = self.env.vision.pixel_to_world(u, v)
                    rospy.loginfo(f"CLICK: ({u},{v}) -> WORLD: {np.round(world_target, 3)}")
                    
                    # 2. VISUAL CONFIRMATION (Draw Purple Dot)
                    # We render one frame with the dot so user sees where they clicked
                    self.visualize_manual(state, rotated_inputs, raw_heatmaps, click_marker=(v, u))
                    cv2.waitKey(200) # Small pause to see the dot
                    
                    # 3. EXECUTE
                    reward = self.env.step(u, v, rot_idx)
                    self.last_reward = reward
                    
                    self.buffer.push_rotational(state, u, v, rot_idx, reward)
                    self.train_burst()
                    
                    if self.step_count % 5 == 0:
                        torch.save(self.model.state_dict(), self.save_path)
                    
                    self.env.reset()
                    self.step_count += 1

            # Visualize
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
        for _ in range(GRADIENT_STEPS):
            batch = self.buffer.sample(min(len(self.buffer), BATCH_SIZE))
            
            b_states = [x[0] for x in batch]; b_u = [x[1] for x in batch]
            b_v = [x[2] for x in batch]; b_rot = [x[3] for x in batch]
            b_rew = torch.tensor([x[4] for x in batch], dtype=torch.float32).to(self.device).unsqueeze(1)

            rotated_imgs = []; rotated_pixels = [] 
            for i in range(len(batch)):
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
            for i in range(len(batch)):
                u_p, v_p = rotated_pixels[i]
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
        idx = self.selected_rot_idx
        angle_deg = self.env.rot_helper.get_angle(idx)

        # 1. Main View (Click Here)
        main_rgb = self._tensor_to_cv(state)
        
        # Show Gripper Orientation (Arrow) at center
        H, W, _ = main_rgb.shape
        cx, cy = W // 2, H // 2
        
        # Convert OpenCV positive angle (CW) vs Math (CCW).
        # We need to visualize the *robot's* rotation.
        # Robot +Angle = CCW. Image +Angle = CW.
        # We negate angle for visualization to match physical reality
        angle_rad = np.deg2rad(-angle_deg) 
        
        end_x = int(cx + 30 * np.cos(angle_rad))
        end_y = int(cy + 30 * np.sin(angle_rad))
        cv2.arrowedLine(main_rgb, (cx, cy), (end_x, end_y), (0, 255, 0), 2)
        
        if click_marker:
            cv2.circle(main_rgb, click_marker, 3, (255, 0, 255), -1) # Purple Dot

        # 2. Network View (What the Brain sees)
        net_input = self._tensor_to_cv(rotated_inputs[idx])
        # Draw vertical reference
        cv2.line(net_input, (cx, cy-20), (cx, cy+20), (0, 255, 0), 1)

        # 3. Network Heatmap
        h = raw_heatmaps[idx].squeeze(0).cpu().numpy()
        h = 1.0 / (1.0 + np.exp(-h))
        h_img = (h * 255).astype(np.uint8)
        net_output = cv2.applyColorMap(h_img, cv2.COLORMAP_JET)

        # Layout: [ Main ] [ Net Input ] [ Net Output ]
        # Stack Horizontally
        row = np.hstack([main_rgb, net_input, net_output])
        
        # Overlay Text
        cv2.putText(row, f"ROTATION: {angle_deg:.0f}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.putText(row, f"CLICK LEFT IMAGE", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        if self.last_reward is not None:
             res = "SUCCESS" if self.last_reward > 0.5 else "FAIL"
             cv2.putText(row, f"LAST: {res}", (150, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        final = cv2.resize(row, (0,0), fx=self.UI_SCALE, fy=self.UI_SCALE, interpolation=cv2.INTER_NEAREST)
        cv2.imshow("Manual Calibration", final)

if __name__ == "__main__":
    ManualRotationalTrainer().run()