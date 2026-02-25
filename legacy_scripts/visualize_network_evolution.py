#!/usr/bin/env python3.8
"""
Visualize network evolution across training checkpoints.

Shows how the network's predictions improve over training by loading
multiple checkpoints and running them on the same scene.

Usage:
    python visualize_network_evolution.py final_small \
        --checkpoints 500,1000,1500,2000 \
        --output ~/slides/figures/
"""

import os
import sys
import argparse
import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib
from matplotlib.gridspec import GridSpec
import torch
import rospy
from pathlib import Path

# Python 3 ROS compatibility
sys.path.insert(0, '/geometry2_ws/devel/lib/python3/dist-packages')

# Add tossingbot to path
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(script_dir, '..'))

from tossingbot import config as cfg
from tossingbot.perception.ros_camera import RosCamera
from tossingbot.perception.vision import VisionProcessor
from tossingbot.learning.agent import TossingAgent
from tossingbot.learning.utils import RotationTransformer

matplotlib.use('Agg')
plt.rcParams.update({
    'font.size': 10,
    'axes.labelsize': 11,
    'axes.titlesize': 12,
    'figure.dpi': 150,
    'savefig.dpi': 300,
    'savefig.bbox': 'tight',
})

class NetworkEvolutionVisualizer:
    def __init__(self, experiment_dir):
        self.experiment_dir = experiment_dir
        self.checkpoint_dir = os.path.join(experiment_dir, 'train/checkpoints')
        
        # Initialize ROS node if not already running
        try:
            rospy.init_node('network_evolution_viz', anonymous=True, disable_signals=True)
        except rospy.exceptions.ROSException:
            pass
        
        # Initialize perception
        self.camera = RosCamera()
        self.vision = VisionProcessor()
        self.transformer = RotationTransformer()
        
        print("Waiting for camera data...")
        while self.camera.get_latest_cloud()[0] is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
        print("Camera ready!")
    
    def get_current_observation(self):
        """Capture current scene from camera."""
        pts, cols = self.camera.get_latest_cloud()
        if pts is None:
            return None, None
        
        # Process to get heightmap
        heightmap = self.vision.process(pts, cols)
        
        # Get RGB image for visualization (use color channel)
        rgb = heightmap[0:3, :, :].cpu().numpy().transpose(1, 2, 0)
        rgb = (rgb * 255).astype(np.uint8)
        
        return heightmap, rgb
    
    def load_checkpoint_and_predict(self, checkpoint_path, heightmap):
        """Load checkpoint and run inference on heightmap."""
        if not os.path.exists(checkpoint_path):
            print(f"Warning: Checkpoint not found: {checkpoint_path}")
            return None, None, None
        
        # Create temporary agent and load weights
        # We need to pass a session-like object, create a minimal one
        class DummySession:
            def __init__(self, checkpoint_dir):
                self.checkpoint_dir = checkpoint_dir
                self.metadata = {}
            def get_checkpoint_path(self, name):
                return checkpoint_path
            def save_hyperparameters(self, hyperparams):
                pass  # No-op for inference
            def save_metadata(self):
                pass  # No-op for inference
        
        dummy_session = DummySession(self.checkpoint_dir)
        agent = TossingAgent(dummy_session, load_buffer=False, load_weights=False)
        
        # Manually load checkpoint
        checkpoint = torch.load(checkpoint_path, map_location=agent.device)
        agent.model.load_state_dict(checkpoint['model'])
        agent.model.eval()
        
        # Run inference
        with torch.no_grad():
            heightmap_tensor = heightmap.unsqueeze(0).to(agent.device)
            q_maps = agent.model(heightmap_tensor)  # [1, R, H, W]
            q_maps = q_maps.squeeze(0).cpu().numpy()  # [R, H, W]
        
        # Get best action across all rotations
        max_q_map = np.max(q_maps, axis=0)  # [H, W]
        best_rot = np.argmax(q_maps, axis=0)  # [H, W] - rotation index for each pixel
        
        # Find global best action
        max_val = np.max(max_q_map)
        best_pixel = np.unravel_index(np.argmax(max_q_map), max_q_map.shape)
        best_rotation_idx = best_rot[best_pixel]
        
        return max_q_map, best_pixel, best_rotation_idx, max_val
    
    def visualize_evolution(self, checkpoints, output_path):
        """Generate grid visualization of network evolution."""
        print(f"\nGenerating network evolution visualization...")
        print(f"Checkpoints: {checkpoints}")
        
        # Capture current scene
        heightmap, rgb_image = self.get_current_observation()
        if heightmap is None:
            print("Error: Could not capture scene")
            return False
        
        # Create figure with grid
        n_checkpoints = len(checkpoints)
        n_cols = 2 if n_checkpoints <= 4 else 3
        n_rows = (n_checkpoints + n_cols - 1) // n_cols
        
        fig = plt.figure(figsize=(n_cols * 5, n_rows * 5))
        gs = GridSpec(n_rows, n_cols, figure=fig, hspace=0.3, wspace=0.3)
        
        for idx, checkpoint_step in enumerate(checkpoints):
            row = idx // n_cols
            col = idx % n_cols
            
            # Load checkpoint
            checkpoint_name = f"checkpoint_{checkpoint_step}.pth"
            checkpoint_path = os.path.join(self.checkpoint_dir, checkpoint_name)
            
            # Try alternative naming
            if not os.path.exists(checkpoint_path):
                checkpoint_path = os.path.join(self.checkpoint_dir, f"checkpoint_step_{checkpoint_step}.pth")
            
            max_q_map, best_pixel, best_rot, max_val = self.load_checkpoint_and_predict(checkpoint_path, heightmap)
            
            if max_q_map is None:
                print(f"  Skipping checkpoint {checkpoint_step} (not found)")
                continue
            
            print(f"  Checkpoint {checkpoint_step}: max_q={max_val:.3f}, pixel={best_pixel}, rot={best_rot}")
            
            # Create subplot
            ax = fig.add_subplot(gs[row, col])
            
            # Overlay heatmap on RGB
            # Normalize heatmap
            heatmap_norm = (max_q_map - max_q_map.min()) / (max_q_map.max() - max_q_map.min() + 1e-8)
            heatmap_colored = cv2.applyColorMap((heatmap_norm * 255).astype(np.uint8), cv2.COLORMAP_JET)
            heatmap_colored = cv2.cvtColor(heatmap_colored, cv2.COLOR_BGR2RGB)
            
            # Blend with RGB
            alpha = 0.6
            blended = cv2.addWeighted(rgb_image, 1-alpha, heatmap_colored, alpha, 0)
            
            # Draw grasp point
            v, u = best_pixel  # heightmap is [H, W]
            circle_color = (255, 255, 0)  # Yellow
            cv2.circle(blended, (u, v), 5, circle_color, -1)
            cv2.circle(blended, (u, v), 6, (0, 0, 0), 2)
            
            # Draw rotation indicator (small line)
            angle = best_rot * (180.0 / cfg.NUM_ROTATIONS)
            length = 15
            angle_rad = np.deg2rad(angle)
            u2 = int(u + length * np.cos(angle_rad))
            v2 = int(v + length * np.sin(angle_rad))
            cv2.line(blended, (u, v), (u2, v2), (255, 255, 0), 2)
            
            ax.imshow(blended)
            ax.set_title(f'Step {checkpoint_step}\nQ={max_val:.2f}, Rot={angle:.0f}°', fontsize=11)
            ax.axis('off')
        
        plt.suptitle('Network Evolution During Training', fontsize=16, fontweight='bold', y=0.98)
        
        # Save
        plt.savefig(output_path, format='pdf', bbox_inches='tight')
        plt.savefig(output_path.replace('.pdf', '.png'), format='png', bbox_inches='tight')
        print(f"\n✓ Saved: {output_path}")
        plt.close()
        
        return True

def main():
    parser = argparse.ArgumentParser(description='Visualize network evolution across checkpoints')
    parser.add_argument('experiment_id', help='Experiment ID (e.g., final_small)')
    parser.add_argument('--checkpoints', default='500,1000,1500,2000', 
                       help='Comma-separated checkpoint steps to visualize')
    parser.add_argument('--output', default='./presentation_figures',
                       help='Output directory for figures')
    parser.add_argument('--base-dir', default=None,
                       help='Base directory for experiments (auto-detect if not provided)')
    
    args = parser.parse_args()
    
    # Parse checkpoint steps
    checkpoint_steps = [int(x.strip()) for x in args.checkpoints.split(',')]
    
    # Auto-detect base directory
    if args.base_dir is None:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        args.base_dir = os.path.join(script_dir, '../../sessions/experiments')
    
    exp_dir = os.path.join(args.base_dir, args.experiment_id)
    if not os.path.exists(exp_dir):
        print(f"Error: Experiment directory not found: {exp_dir}")
        return 1
    
    # Create output directory
    os.makedirs(args.output, exist_ok=True)
    
    output_path = os.path.join(args.output, 'network_evolution_grid.pdf')
    
    print("="*70)
    print("NETWORK EVOLUTION VISUALIZATION")
    print("="*70)
    print(f"Experiment: {args.experiment_id}")
    print(f"Checkpoints: {checkpoint_steps}")
    print(f"Output: {output_path}")
    print("="*70)
    
    print("\nIMPORTANT: Make sure Gazebo is running with a scene spawned!")
    print("The script will capture the current camera view.\n")
    
    input("Press Enter when ready to capture scene...")
    
    # Create visualizer and generate figure
    visualizer = NetworkEvolutionVisualizer(exp_dir)
    success = visualizer.visualize_evolution(checkpoint_steps, output_path)
    
    if success:
        print("\n✓ Network evolution visualization complete!")
        return 0
    else:
        print("\n✗ Failed to generate visualization")
        return 1

if __name__ == '__main__':
    sys.exit(main())
