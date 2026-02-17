import torch
import torch.optim as optim
import torch.nn as nn
import torchvision.transforms.functional as TF
import numpy as np
import random
import os
import cv2
import pickle

from tossingbot import config as cfg
from tossingbot.learning.network import TossingBot_Modular
from tossingbot.learning.buffer import RankBasedReplayBuffer
from tossingbot.learning.utils import RotationTransformer

class TossingAgent:
    def __init__(self, session, load_buffer=True, load_weights=True):
        """
        Args:
            session: Session object from SessionManager
            load_buffer: whether to load saved replay buffer (default: True)
            load_weights: whether to load saved weights (default: True)
        """
        self.session = session
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        # Model
        self.model = TossingBot_Modular(input_channels=4).to(self.device)
        self.optimizer = optim.SGD(self.model.parameters(), lr=cfg.LEARNING_RATE, 
                                   momentum=cfg.MOMENTUM, weight_decay=cfg.WEIGHT_DECAY)
        self.loss_fn = nn.BCEWithLogitsLoss(reduction='none')
        
        # Memory & Utils
        self.buffer = RankBasedReplayBuffer(capacity=cfg.BUFFER_CAPACITY)
        self.transformer = RotationTransformer(device=self.device)
        
        # Load weights if requested
        if load_weights:
            self.load_weights()
        
        # Load replay buffer if requested
        if load_buffer:
            self.load_buffer()
        
        # Save hyperparameters to session
        self.save_hyperparameters()

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

    def save_hyperparameters(self):
        """Save current hyperparameters to session"""
        hyperparams = {
            'learning_rate': cfg.LEARNING_RATE,
            'momentum': cfg.MOMENTUM,
            'weight_decay': cfg.WEIGHT_DECAY,
            'batch_size': cfg.BATCH_SIZE,
            'buffer_capacity': cfg.BUFFER_CAPACITY,
            'num_rotations': cfg.NUM_ROTATIONS,
            'explore_start': cfg.EXPLORE_START,
            'explore_end': cfg.EXPLORE_END,
            'explore_steps': cfg.EXPLORE_STEPS,
            'save_interval': cfg.SAVE_INTERVAL
        }
        self.session.save_hyperparameters(hyperparams)
    
    def save_snapshot(self, step, success_rate=0.0):
        """
        Save checkpoint with session management.
        
        Args:
            step: Current training step
            success_rate: Current success rate (for best checkpoint tracking)
        """
        import datetime
        
        # Generate checkpoint filename
        filename = f"checkpoint_step_{step}.pth"
        checkpoint_path = self.session.get_checkpoint_path(filename)
        latest_path = self.session.get_checkpoint_path("checkpoint_latest.pth")
        
        # Save checkpoint
        checkpoint = {
            'model': self.model.state_dict(),
            'opt': self.optimizer.state_dict(),
            'step': step,
            'success_rate': success_rate,
            'timestamp': datetime.datetime.now().isoformat()
        }
        
        torch.save(checkpoint, checkpoint_path)
        
        # Update latest symlink
        if os.path.exists(latest_path) or os.path.islink(latest_path):
            os.remove(latest_path)
        os.symlink(filename, latest_path)
        
        print(f"Saved checkpoint: {filename}")
        
        # Update session metadata
        checkpoints = self.session.metadata.get('checkpoints', [])
        if filename not in checkpoints:
            checkpoints.append(filename)
        
        self.session.update_metadata(
            total_steps=step,
            success_rate=success_rate,
            buffer_size=len(self.buffer),
            checkpoints=checkpoints
        )
        
        # Track best checkpoint
        best = self.session.metadata.get('best_checkpoint')
        if best is None or success_rate > best.get('success_rate', 0):
            # Save as best checkpoint
            best_path = self.session.get_checkpoint_path("checkpoint_best.pth")
            torch.save(checkpoint, best_path)
            
            self.session.update_metadata(
                best_checkpoint={
                    'file': filename,
                    'success_rate': success_rate,
                    'step': step,
                    'timestamp': checkpoint['timestamp']
                }
            )
            print(f"New best checkpoint! Success rate: {success_rate*100:.1f}%")
        
        # Save replay buffer
        self.save_buffer()

    def load_weights(self, checkpoint_name="checkpoint_latest.pth"):
        """
        Load model weights from session checkpoint directory.
        
        Args:
            checkpoint_name: Name of checkpoint file to load
        """
        checkpoint_path = self.session.get_checkpoint_path(checkpoint_name)
        
        if not os.path.exists(checkpoint_path):
            print(f"No checkpoint found at: {checkpoint_path}")
            print("Starting with random initialization.")
            return
        
        try:
            ckpt = torch.load(checkpoint_path, map_location=self.device)
            self.model.load_state_dict(ckpt['model'])
            self.optimizer.load_state_dict(ckpt['opt'])
            
            step_info = f" (step {ckpt['step']})" if 'step' in ckpt else ""
            success_info = f" [{ckpt['success_rate']*100:.1f}% success]" if 'success_rate' in ckpt else ""
            print(f"Loaded weights: {checkpoint_name}{step_info}{success_info}")
        except Exception as e:
            print(f"Error loading checkpoint: {e}")
            print("Starting with random initialization.")
    
    def save_buffer(self):
        """Save replay buffer to session"""
        buffer_path = self.session.get_buffer_path()
        try:
            with open(buffer_path, 'wb') as f:
                pickle.dump(self.buffer, f)
            print(f"Saved replay buffer ({len(self.buffer)} experiences)")
        except Exception as e:
            print(f"Warning: Could not save replay buffer: {e}")
    
    def load_buffer(self):
        """Load replay buffer from session"""
        buffer_path = self.session.get_buffer_path()
        
        if not os.path.exists(buffer_path):
            print("No saved replay buffer found. Starting with empty buffer.")
            return
        
        try:
            with open(buffer_path, 'rb') as f:
                self.buffer = pickle.load(f)
            print(f"Loaded replay buffer ({len(self.buffer)} experiences)")
        except Exception as e:
            print(f"Warning: Could not load replay buffer: {e}")
            print("Starting with empty buffer.")