import torch
import torch.optim as optim
import torch.nn as nn
import torchvision.transforms.functional as TF
import numpy as np
import random
import os
import cv2

from tossingbot import config as cfg
from tossingbot.learning.network import TossingBot_Modular
from tossingbot.learning.buffer import RankBasedReplayBuffer
from tossingbot.learning.utils import RotationTransformer

class TossingAgent:
    def __init__(self, load_weights='continue', load_buffer=True):
        """
        Args:
            load_weights: 'continue' (load latest), 'new' (fresh start), or path to specific checkpoint
            load_buffer: whether to load saved replay buffer (default: True)
        """
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        # Model
        self.model = TossingBot_Modular(input_channels=4).to(self.device)
        self.optimizer = optim.SGD(self.model.parameters(), lr=cfg.LEARNING_RATE, 
                                   momentum=cfg.MOMENTUM, weight_decay=cfg.WEIGHT_DECAY)
        self.loss_fn = nn.BCEWithLogitsLoss(reduction='none')
        
        # Memory & Utils
        self.buffer = RankBasedReplayBuffer(capacity=cfg.BUFFER_CAPACITY)
        self.transformer = RotationTransformer(device=self.device)
        
        # Training metadata
        self.training_start_time = None
        self.checkpoint_name = None
                
        # Load weights based on strategy
        if load_weights != 'new':
            self.load_weights(load_weights if load_weights != 'continue' else None)
        
        # Load replay buffer if requested
        if load_buffer and cfg.SAVE_BUFFER:
            self.load_buffer()

    def get_action(self, state_tensor, epsilon=0.0, failed_attempts=[]):
        """
        Returns: (rot_idx, u, v, debug_data)
        """
        state_gpu = state_tensor.to(self.device)

        with torch.no_grad():
            # forward_multi_view handles the 4 rotations internally
            full_vol, rotated_inputs, raw_heatmaps = self._forward_multi_view(state_gpu)
            
            # Apply Inhibition (Don't repeat mistakes)
            masked_vol = full_vol.clone()
            H, W = masked_vol.shape[2:]
            
            for (f_rot, f_u, f_v) in failed_attempts:
                rad = 2 # Inhibition radius
                masked_vol[0, f_rot, max(0, f_u-rad):min(H, f_u+rad), max(0, f_v-rad):min(W, f_v+rad)] = -1e9

            # Selection Strategy
            if random.random() < epsilon:
                action_type = "RANDOM"
                rot_idx = random.randint(0, cfg.NUM_ROTATIONS - 1)
                u = random.randint(15, H - 16)
                v = random.randint(15, W - 16)
                conf = 0.0
            else:
                action_type = "NETWORK"
                flat_idx = torch.argmax(masked_vol).item()
                rot_idx = flat_idx // (H * W)
                rem = flat_idx % (H * W)
                u = rem // W
                v = rem % W
                conf = torch.sigmoid(full_vol[0, rot_idx, u, v]).item()

        # Debug/Vis Data
        debug = {
            'inputs': rotated_inputs,
            'heatmaps': raw_heatmaps,
            'state_viz': state_gpu,
            'type': action_type,
            'conf': conf
        }
        return rot_idx, u, v, debug

    def train(self):
        if len(self.buffer) < cfg.BATCH_SIZE: return 0.0
        
        # 1. Sample
        batch, indices = self.buffer.sample(cfg.BATCH_SIZE)
        
        # Unpack...
        b_states = torch.stack([x[0] for x in batch]).to(self.device)
        b_u = [x[1] for x in batch]
        b_v = [x[2] for x in batch]
        b_rot = [x[3] for x in batch]
        b_rew = torch.tensor([x[4] for x in batch], dtype=torch.float32).to(self.device).unsqueeze(1)

        # Rotate images using the new unified method...
        rotated_inputs = []
        for i in range(cfg.BATCH_SIZE):
            angle = (cfg.TOTAL_DEG / cfg.NUM_ROTATIONS) * b_rot[i]
            # Use the new, centralized and robust rotation function
            rot_img = self.transformer.rotate_single_with_padding(b_states[i], angle)
            rotated_inputs.append(rot_img)
        
        # Forward
        tensor_in = torch.stack(rotated_inputs)
        self.optimizer.zero_grad()
        
        # "logits" are raw scores (-inf to +inf)
        logits = self.model(tensor_in)
        
        # Extract specific pixel logits
        pred_logits = []
        for i in range(cfg.BATCH_SIZE):
            pred_logits.append(logits[i, 0, b_u[i], b_v[i]])
        
        pred_logits = torch.stack(pred_logits).unsqueeze(1)
        
        # 2. Compute Loss (BCEWithLogitsLoss prefers raw logits for stability)
        loss = self.loss_fn(pred_logits, b_rew).mean()
        
        loss.backward()
        self.optimizer.step()
        
        # 3. Update Priorities (CRITICAL FIX)
        # We must convert Logits -> Probability (0..1) to compare with Reward (0 or 1)
        with torch.no_grad():
            pred_probs = torch.sigmoid(pred_logits) # Convert to 0.0 - 1.0
            errors = torch.abs(pred_probs - b_rew).cpu().squeeze().numpy()
        
        # Handle scalar edge case
        if errors.ndim == 0: errors = np.array([errors])
            
        self.buffer.update_priorities(indices, errors)
        
        return loss.item()

    def _forward_multi_view(self, state_tensor):
        # Use the new, unified rotation method from our transformer
        batch_rotated = []
        for i in range(cfg.NUM_ROTATIONS):
            angle = (cfg.TOTAL_DEG / cfg.NUM_ROTATIONS) * i
            rot_img = self.transformer.rotate_single_with_padding(state_tensor, angle)
            batch_rotated.append(rot_img)
        
        stack = torch.stack(batch_rotated)
        out = self.model(stack) # [4, 1, H, W]
        return out.permute(1, 0, 2, 3), stack, out # Returns [1, 4, H, W]

    def save_snapshot(self, step):
        """Save checkpoint with timestamp or custom name"""
        import datetime
        if not os.path.exists(cfg.WEIGHTS_DIR): 
            os.makedirs(cfg.WEIGHTS_DIR)
        
        # Generate checkpoint filename
        if self.checkpoint_name:
            filename = f"tossingbot_auto_{self.checkpoint_name}_step{step}.pth"
        else:
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"tossingbot_auto_{timestamp}_step{step}.pth"
        
        checkpoint_path = os.path.join(cfg.WEIGHTS_DIR, filename)
        latest_path = os.path.join(cfg.WEIGHTS_DIR, "tossingbot_auto_latest.pth")
        
        # Save checkpoint with metadata
        checkpoint = {
            'model': self.model.state_dict(),
            'opt': self.optimizer.state_dict(),
            'step': step,
            'timestamp': datetime.datetime.now().isoformat(),
            'checkpoint_name': self.checkpoint_name
        }
        
        if self.training_start_time:
            checkpoint['training_start_time'] = self.training_start_time
        
        torch.save(checkpoint, checkpoint_path)
        
        # Create/update symlink to latest
        if os.path.exists(latest_path) or os.path.islink(latest_path):
            os.remove(latest_path)
        os.symlink(os.path.basename(checkpoint_path), latest_path)
        
        print(f"Saved checkpoint: {filename}")
        
        # Save replay buffer if enabled
        if cfg.SAVE_BUFFER:
            self.save_buffer()

    def load_weights(self, path=None):
        """
        Load model weights from checkpoint.
        Args:
            path: Specific checkpoint path, or None to load latest
        """
        if path is None:
            # Try to load latest
            latest_path = os.path.join(cfg.WEIGHTS_DIR, "tossingbot_auto_latest.pth")
            if not os.path.exists(latest_path):
                # Fallback to old naming convention
                path = cfg.SAVE_PATH
                if not os.path.exists(path):
                    print("No existing weights found. Starting with random initialization.")
                    return
            else:
                path = latest_path
        
        if not os.path.exists(path):
            print(f"Warning: Checkpoint not found at {path}. Starting with random initialization.")
            return
        
        try:
            ckpt = torch.load(path, map_location=self.device)
            self.model.load_state_dict(ckpt['model'])
            self.optimizer.load_state_dict(ckpt['opt'])
            
            # Load metadata if available
            if 'checkpoint_name' in ckpt and ckpt['checkpoint_name']:
                self.checkpoint_name = ckpt['checkpoint_name']
            if 'training_start_time' in ckpt:
                self.training_start_time = ckpt['training_start_time']
            
            step_info = f" (step {ckpt['step']})" if 'step' in ckpt else ""
            print(f"Loaded weights from: {os.path.basename(path)}{step_info}")
        except Exception as e:
            print(f"Error loading checkpoint: {e}")
            print("Starting with random initialization.")
    
    def save_buffer(self):
        """Save replay buffer to disk"""
        try:
            import pickle
            with open(cfg.BUFFER_PATH, 'wb') as f:
                pickle.dump(self.buffer, f)
            print(f"Saved replay buffer ({len(self.buffer)} experiences)")
        except Exception as e:
            print(f"Warning: Could not save replay buffer: {e}")
    
    def load_buffer(self):
        """Load replay buffer from disk"""
        if not os.path.exists(cfg.BUFFER_PATH):
            print("No saved replay buffer found. Starting with empty buffer.")
            return
        
        try:
            import pickle
            with open(cfg.BUFFER_PATH, 'rb') as f:
                self.buffer = pickle.load(f)
            print(f"Loaded replay buffer ({len(self.buffer)} experiences)")
        except Exception as e:
            print(f"Warning: Could not load replay buffer: {e}")
            print("Starting with empty buffer.")