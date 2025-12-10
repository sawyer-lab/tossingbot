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
                # raw_heatmaps are in the ROTATED frame (matching the input images)
                _, rotated_inputs, raw_heatmaps = self.forward_multi_view(state_gpu)
                
                # Apply mask to the RAW heatmaps
                # raw_heatmaps = self.apply_workspace_mask(raw_heatmaps)

            # 2. SELECT ACTION (Greedy Argmax on RAW ROTATED MAPS)
            flat_idx = torch.argmax(raw_heatmaps).item()
            
            # [FIX] Use -2: to grab the last two dims (Height, Width) 
            # regardless of whether the shape is (N, C, H, W) or (N, H, W)
            H, W = raw_heatmaps.shape[-2:] 
            
            # Calculate the area per rotation (Stride)
            # We must account for the Channel dimension if it exists
            C = raw_heatmaps.shape[1] 
            stride = C * H * W 
            
            # Map flat index to Rotation Index
            rot_idx = flat_idx // stride
            
            # Find the remainder to get local pixel (u,v)
            rem = flat_idx % stride
            # If C > 1, we need to mod out the channel too, but assuming C=1 for grasping:
            pixel_idx = rem % (H * W)
            
            u_rot = pixel_idx // W   # Row (Y)
            v_rot = pixel_idx % W    # Col (X)
            
            # Select confidence
            conf = torch.sigmoid(raw_heatmaps[rot_idx, 0, u_rot, v_rot]).item()
            angle_deg = self.env.rot_helper.get_angle(rot_idx)

            rospy.loginfo(f"Step {self.step_count} | Action: Rot {rot_idx} | Conf: {conf:.2f}")

            # 3. VISUALIZE (Correct Alignment)
            # Now u_rot, v_rot ARE in the rotated frame, so they will match the image perfectly.
            self.visualize_dashboard(state, rotated_inputs, raw_heatmaps, rot_idx, u_rot, v_rot, conf)
            
            # 4. EXECUTE (Convert to World for Robot)
            # Now we consciously un-rotate for the robot execution
            u_world, v_world = self.transformer.rotate_pixel(
                u_rot, v_rot, 
                angle_deg, 
                H, W, 
                to_gripper_frame=False
            )
            
            reward = self.env.step(u_world, v_world, rot_idx)
            
            # 5. BUFFER (Store World Coords)
            self.buffer.push_rotational(state, u_world, v_world, rot_idx, reward)
            
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
        # [FIX] Robustly get H and W regardless of 3D or 4D input
        H, W = heatmap_vol.shape[-2:]
        
        cy, cx = H // 2, W // 2
        radius = min(H, W) // 2 - 2
        
        Y, X = np.ogrid[:H, :W]
        dist = np.sqrt((X - cx)**2 + (Y - cy)**2)
        mask = torch.from_numpy(dist <= radius).float().to(self.device)
        
        # Broadcasting handles the (N, C, H, W) vs (H, W) multiplication automatically
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

    def visualize_dashboard(self, state, rotated_inputs, raw_heatmaps, chosen_rot, u_rot, v_rot, conf):
        """
        Top: 4 Rotation Pairs.
        Bottom: Large Global View with re-mapped target.
        """
        # --- TOP HALF: ROTATED VIEWS ---
        pairs = []
        for i in range(NUM_ROTATIONS):
            # 1. Input Image
            input_cv = self._tensor_to_cv(rotated_inputs[i])
            
            # 2. Heatmap
            h = raw_heatmaps[i].squeeze(0).cpu().numpy()
            h = 1.0 / (1.0 + np.exp(-h))
            h_img = (h * 255).astype(np.uint8)
            heatmap_cv = cv2.applyColorMap(h_img, cv2.COLORMAP_JET)
            
            # Resize if needed
            if heatmap_cv.shape[:2] != input_cv.shape[:2]:
                heatmap_cv = cv2.resize(heatmap_cv, (input_cv.shape[1], input_cv.shape[0]), interpolation=cv2.INTER_NEAREST)

            # 3. Draw Selected Point (If this is the chosen rotation)
            if i == chosen_rot:
                # Draw Red Dot on Input (Where the net clicked)
                cv2.circle(input_cv, (v_rot, u_rot), 3, (0, 0, 255), -1)
                # Draw White Dot on Heatmap
                cv2.circle(heatmap_cv, (v_rot, u_rot), 3, (255, 255, 255), -1)
                
                # Combine
                pair = np.hstack([input_cv, heatmap_cv])
                
                # Green Border for Selection
                cv2.rectangle(pair, (0,0), (pair.shape[1]-1, pair.shape[0]-1), (0,255,0), 3)
                label = f"Rot {i*45} (CHOSEN)"
            else:
                pair = np.hstack([input_cv, heatmap_cv])
                label = f"Rot {i*45}"

            cv2.putText(pair, label, (10, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255), 1)
            pairs.append(pair)
            
        # Stack pairs horizontally into one long strip
        top_row = np.hstack(pairs)

        # --- BOTTOM HALF: WORLD VIEW ---
        # 1. Get Base Image
        world_rgb = self._tensor_to_cv(state)
        H, W, _ = world_rgb.shape
        
        # 2. Map the (u_rot, v_rot) back to World Pixel (u_world, v_world)
        # We use the transformer logic to "Un-Rotate" the pixel
        angle = self.env.rot_helper.get_angle(chosen_rot)
        
        # NOTE: rotate_pixel(..., to_gripper=False) means "Un-rotate"
        u_world, v_world = self.transformer.rotate_pixel(u_rot, v_rot, angle, H, W, to_gripper_frame=False)
        
        # 3. Draw the Target on World View
        # Dot (Location)
        cv2.circle(world_rgb, (v_world, u_world), 3, (0, 255, 0), -1) 
        
        # Arrow (Orientation)
        # Note: We visualize the robot's physical rotation
        # If angle=0, arrow points Right (Standard 0)
        # If angle=90, arrow points Up (Standard 90)
        # BUT: Image Y is flipped. So +90 means Down visually if we use sin/cos directly.
        # Let's align with the previous manual tester logic:
        angle_rad = np.deg2rad(-angle) 
        end_x = int(v_world + 30 * np.cos(angle_rad))
        end_y = int(u_world + 30 * np.sin(angle_rad))
        cv2.arrowedLine(world_rgb, (v_world, u_world), (end_x, end_y), (0, 0, 255), 2)

        

        # --- FINAL LAYOUT ---
        # Resize Bottom to match Top width
        scale = top_row.shape[1] / world_rgb.shape[1]
        new_h = int(world_rgb.shape[0] * scale)
        bottom_row = cv2.resize(world_rgb, (top_row.shape[1], new_h), interpolation=cv2.INTER_NEAREST)
        
        final_grid = np.vstack([top_row, bottom_row])
        
        # Zoom for visibility
        final_large = cv2.resize(final_grid, (0,0), fx=1.5, fy=1.5, interpolation=cv2.INTER_NEAREST)

        cv2.imshow("Auto Brain", final_large)
        cv2.waitKey(1)

if __name__ == "__main__":
    AutoTrainer().run()