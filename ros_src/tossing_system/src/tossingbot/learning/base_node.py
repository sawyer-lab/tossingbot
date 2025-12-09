import os
import torch
import torch.optim as optim
import torch.nn as nn
import cv2
import rospy
import rospkg
import numpy as np

from tossingbot.env.gazebo_env import GazeboEnv
from tossingbot.learning.network import TossingBot
from tossingbot.learning.buffer import ReplayBuffer

class BaseLearningNode:
    def __init__(self, node_name, training_enabled=True):
        rospy.init_node(node_name)
        
        # --- CONFIGURATION ---
        self.LEARNING_RATE = 1e-4
        self.BATCH_SIZE = 1000
        self.SAVE_FILENAME = "tossingbot.pth"
        self.training_enabled = training_enabled

        # 1. Init Environment
        self.env = GazeboEnv()
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        rospy.loginfo(f"[{node_name}] Running on: {self.device}")

        # 2. Paths
        rp = rospkg.RosPack()
        pkg_path = rp.get_path('tossingbot_system')
        self.weights_dir = os.path.join(pkg_path, "weights")
        if not os.path.exists(self.weights_dir): os.makedirs(self.weights_dir)
        self.save_path = os.path.join(self.weights_dir, self.SAVE_FILENAME)

        # 3. Network Setup
        self.model = TossingBot().to(self.device)
        self.load_weights()

        if self.training_enabled:
            self.model.train()
            self.optimizer = optim.Adam(self.model.parameters(), lr=self.LEARNING_RATE)
            self.loss_fn = nn.BCEWithLogitsLoss()
            self.buffer = ReplayBuffer(capacity=2000)
        else:
            self.model.eval() # Inference Mode

        # 4. State & UI
        self.step_count = 0
        self.current_loss = 0.0
        cv2.namedWindow("TossingBot View", cv2.WINDOW_NORMAL)

    def load_weights(self):
        if os.path.exists(self.save_path):
            rospy.loginfo(f"Loading weights: {self.save_path}")
            self.model.load_state_dict(torch.load(self.save_path))
        else:
            rospy.logwarn("No weights found. Starting fresh.")

    def save_weights(self):
        if self.training_enabled:
            torch.save(self.model.state_dict(), self.save_path)
            rospy.loginfo("Checkpoint Saved.")

    def get_heatmap(self):
        """Standard Forward Pass"""
        state = self.env.get_observation()
        if state is None: return None, None
        
        # Prepare Input
        state_gpu = state.unsqueeze(0).to(self.device)
        
        # Forward (No Grad if deploying)
        with torch.set_grad_enabled(self.training_enabled):
            logits = self.model(state_gpu)
            heatmap = logits[0, 0, :, :] # (H, W)
            
        return state, heatmap

    def execute_and_store(self, state, u, v):
        """Common Logic: Execute -> Get Label -> Store in Buffer"""
        # Execute
        label = self.env.step(u, v) # Returns 1.0 (Success) or 0.0 (Fail)
        
        # Store (Only if training)
        if self.training_enabled:
            self.buffer.push(state, u, v, label)
        
        self.env.reset()
        self.step_count += 1
        return label

    def train_burst(self, iterations=1):
        """
        Self-Supervised Update Loop.
        Trains the network to predict '1' for successful pixels and '0' for failed ones.
        """
        if not self.training_enabled or len(self.buffer) < self.BATCH_SIZE:
            return

        total_loss = 0.0
        
        for _ in range(iterations):
            # Sample Batch
            batch = self.buffer.sample(self.BATCH_SIZE)
            
            b_states = torch.stack([x[0] for x in batch]).to(self.device)
            b_u = [x[1] for x in batch]
            b_v = [x[2] for x in batch]
            b_labels = torch.tensor([x[3] for x in batch], dtype=torch.float32).to(self.device).unsqueeze(1)

            # Forward
            self.optimizer.zero_grad()
            b_logits = self.model(b_states)
            
            # Masked Selection (Only gradients for the clicked pixels)
            pred_vals = []
            for i in range(self.BATCH_SIZE):
                val = b_logits[i, 0, b_u[i], b_v[i]]
                pred_vals.append(val)
            pred_vals = torch.stack(pred_vals).unsqueeze(1)

            # Backward (BCE Loss)
            loss = self.loss_fn(pred_vals, b_labels)
            loss.backward()
            self.optimizer.step()
            total_loss += loss.item()

        self.current_loss = total_loss / iterations
        rospy.loginfo(f"TRAINED ({iterations} iters). Avg Loss: {self.current_loss:.4f}")

    def visualize(self, state, heatmap, u, v, extra_text=""):
        # RGB
        img_rgb = state.permute(1, 2, 0).cpu().numpy()
        img_rgb = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
        
        # Heatmap
        prob_map = torch.sigmoid(heatmap).detach().cpu().numpy()
        img_heat = (prob_map * 255).astype(np.uint8)
        img_heat = cv2.applyColorMap(img_heat, cv2.COLORMAP_JET)
        
        if img_heat.shape[:2] != img_rgb.shape[:2]:
            img_heat = cv2.resize(img_heat, (img_rgb.shape[1], img_rgb.shape[0]), interpolation=cv2.INTER_NEAREST)

        # Crosshair
        cv2.drawMarker(img_rgb, (v, u), (0, 255, 0), markerType=cv2.MARKER_CROSS, markerSize=15, thickness=2)
        cv2.drawMarker(img_heat, (v, u), (255, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=15, thickness=2)

        # Stack
        display = np.hstack([img_rgb, img_heat])
        display = cv2.resize(display, (0,0), fx=3.0, fy=3.0, interpolation=cv2.INTER_NEAREST)
        
        # Text
        info = f"Steps: {self.step_count} | Loss: {self.current_loss:.3f} | {extra_text}"
        cv2.putText(display, info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        cv2.imshow("TossingBot View", display)
        cv2.waitKey(1)