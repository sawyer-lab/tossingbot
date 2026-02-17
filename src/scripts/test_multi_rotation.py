#!/usr/bin/env python3.8
"""
Test network performance across multiple object rotations.

Spawns the same object at different rotations and evaluates grasping success.
Generates visualization showing rotation invariance.

Usage:
    python test_multi_rotation.py final_small \
        --object T_shape_small \
        --rotations 8 \
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
from tossingbot.environment.tossing_env import TossingEnv
from tossingbot.learning.agent import TossingAgent

matplotlib.use('Agg')
plt.rcParams.update({
    'font.size': 9,
    'axes.labelsize': 10,
    'axes.titlesize': 11,
    'figure.dpi': 150,
    'savefig.dpi': 300,
    'savefig.bbox': 'tight',
})

class MultiRotationTester:
    def __init__(self, experiment_dir, checkpoint_name='checkpoint_best.pth'):
        self.experiment_dir = experiment_dir
        self.checkpoint_path = os.path.join(experiment_dir, 'train/checkpoints', checkpoint_name)
        
        if not os.path.exists(self.checkpoint_path):
            raise FileNotFoundError(f"Checkpoint not found: {self.checkpoint_path}")
        
        # Initialize ROS node if not already running
        try:
            rospy.init_node('multi_rotation_test', anonymous=True, disable_signals=True)
        except rospy.exceptions.ROSException:
            pass
        
        print(f"Loading checkpoint: {checkpoint_name}")
    
    def test_rotation(self, object_type, rotation_angle_deg):
        """
        Test grasping at specific rotation.
        Returns: (rgb_image, heatmap, grasp_pixel, grasp_rotation, confidence, success)
        """
        # Initialize environment with single object type
        env = TossingEnv(allowed_objects=[object_type])
        
        # Force environment to use single instance spawning
        env._instances_per_type_override = 1
        
        # Spawn object at specific rotation
        env.sim.manager.despawn_all()
        rospy.sleep(0.5)
        
        # Spawn with controlled rotation
        # Convert angle to quaternion (rotation around Z-axis)
        angle_rad = np.deg2rad(rotation_angle_deg)
        qz = np.sin(angle_rad / 2)
        qw = np.cos(angle_rad / 2)
        
        from geometry_msgs.msg import Pose, Point, Quaternion
        pose = Pose(
            position=Point(0.6, 0.15, 0.76),
            orientation=Quaternion(0, 0, qz, qw)
        )
        
        # Spawn object
        success = env.sim.manager.spawn(
            object_type, 
            f"{object_type}_test",
            pose=pose
        )
        
        if not success:
            print(f"Failed to spawn object at {rotation_angle_deg}°")
            return None, None, None, None, None, False
        
        rospy.sleep(1.5)
        
        # Get observation
        obs = env.get_observation()
        if obs is None:
            return None, None, None, None, None, False
        
        # Load agent and run inference
        class DummySession:
            def __init__(self, checkpoint_path):
                self.checkpoint_path_val = checkpoint_path
                self.metadata = {}
            def get_checkpoint_path(self, name):
                return self.checkpoint_path_val
            def save_hyperparameters(self, hyperparams):
                pass  # No-op for inference
            def save_metadata(self):
                pass  # No-op for inference
        
        dummy_session = DummySession(self.checkpoint_path)
        agent = TossingAgent(dummy_session, load_buffer=False, load_weights=False)
        
        checkpoint = torch.load(self.checkpoint_path, map_location=agent.device)
        agent.model.load_state_dict(checkpoint['model'])
        agent.model.eval()
        
        # Run inference
        with torch.no_grad():
            heightmap_tensor = obs.unsqueeze(0).to(agent.device)
            q_maps = agent.model(heightmap_tensor)
            q_maps = q_maps.squeeze(0).cpu().numpy()
        
        # Get best action
        max_q_map = np.max(q_maps, axis=0)
        best_rot_map = np.argmax(q_maps, axis=0)
        
        max_val = np.max(max_q_map)
        best_pixel = np.unravel_index(np.argmax(max_q_map), max_q_map.shape)
        best_rotation_idx = best_rot_map[best_pixel]
        
        # Get RGB for visualization
        rgb = obs[0:3, :, :].cpu().numpy().transpose(1, 2, 0)
        rgb = (rgb * 255).astype(np.uint8)
        
        # Execute grasp
        u, v = best_pixel[1], best_pixel[0]  # Convert to pixel coordinates
        result = env.step(u, v, best_rotation_idx)
        grasp_success = result[2]  # reward > 0
        
        # Clean up
        env.sim.manager.despawn_all()
        
        return rgb, max_q_map, best_pixel, best_rotation_idx, max_val, grasp_success
    
    def visualize_multi_rotation(self, object_type, num_rotations, output_path):
        """Generate grid visualization of multi-rotation test."""
        print(f"\nTesting {object_type} at {num_rotations} rotations...")
        
        angles = np.linspace(0, 360, num_rotations, endpoint=False)
        results = []
        
        for angle in angles:
            print(f"  Testing rotation: {angle:.0f}°", end='... ')
            result = self.test_rotation(object_type, angle)
            results.append({
                'angle': angle,
                'rgb': result[0],
                'heatmap': result[1],
                'pixel': result[2],
                'rotation_idx': result[3],
                'confidence': result[4],
                'success': result[5]
            })
            print(f"{'SUCCESS' if result[5] else 'FAIL'} (Q={result[4]:.2f})")
        
        # Create visualization grid
        n_cols = 4 if num_rotations >= 8 else 3
        n_rows = (num_rotations + n_cols - 1) // n_cols
        
        fig = plt.figure(figsize=(n_cols * 4, n_rows * 4))
        gs = GridSpec(n_rows, n_cols, figure=fig, hspace=0.35, wspace=0.25)
        
        for idx, result in enumerate(results):
            row = idx // n_cols
            col = idx % n_cols
            
            if result['rgb'] is None:
                continue
            
            ax = fig.add_subplot(gs[row, col])
            
            # Overlay heatmap
            heatmap = result['heatmap']
            heatmap_norm = (heatmap - heatmap.min()) / (heatmap.max() - heatmap.min() + 1e-8)
            heatmap_colored = cv2.applyColorMap((heatmap_norm * 255).astype(np.uint8), cv2.COLORMAP_JET)
            heatmap_colored = cv2.cvtColor(heatmap_colored, cv2.COLOR_BGR2RGB)
            
            blended = cv2.addWeighted(result['rgb'], 0.4, heatmap_colored, 0.6, 0)
            
            # Draw grasp point
            v, u = result['pixel']
            color = (0, 255, 0) if result['success'] else (255, 0, 0)
            cv2.circle(blended, (u, v), 5, color, -1)
            cv2.circle(blended, (u, v), 6, (255, 255, 255), 2)
            
            # Draw rotation indicator
            angle_net = result['rotation_idx'] * (180.0 / cfg.NUM_ROTATIONS)
            length = 15
            angle_rad = np.deg2rad(angle_net)
            u2 = int(u + length * np.cos(angle_rad))
            v2 = int(v + length * np.sin(angle_rad))
            cv2.line(blended, (u, v), (u2, v2), (255, 255, 0), 2)
            
            ax.imshow(blended)
            
            # Title with result
            status = '✓' if result['success'] else '✗'
            title = f"{status} {result['angle']:.0f}° | Q={result['confidence']:.2f}"
            color_title = 'green' if result['success'] else 'red'
            ax.set_title(title, fontsize=10, color=color_title, fontweight='bold')
            ax.axis('off')
        
        # Overall title with success rate
        n_success = sum(1 for r in results if r['success'])
        success_rate = n_success / len(results) * 100
        title = f'Multi-Rotation Analysis: {object_type.replace("_small", "").replace("_", " ").title()}\n'
        title += f'Success Rate: {n_success}/{len(results)} ({success_rate:.0f}%)'
        plt.suptitle(title, fontsize=14, fontweight='bold', y=0.98)
        
        # Save
        plt.savefig(output_path, format='pdf', bbox_inches='tight')
        plt.savefig(output_path.replace('.pdf', '.png'), format='png', bbox_inches='tight')
        print(f"\n✓ Saved: {output_path}")
        plt.close()
        
        return success_rate

def main():
    parser = argparse.ArgumentParser(description='Test network across multiple rotations')
    parser.add_argument('experiment_id', help='Experiment ID (e.g., final_small)')
    parser.add_argument('--object', default='T_shape_small',
                       help='Object type to test')
    parser.add_argument('--rotations', type=int, default=8,
                       help='Number of rotations to test')
    parser.add_argument('--checkpoint', default='checkpoint_best.pth',
                       help='Checkpoint to use')
    parser.add_argument('--output', default='./presentation_figures',
                       help='Output directory for figures')
    parser.add_argument('--base-dir', default=None,
                       help='Base directory for experiments')
    
    args = parser.parse_args()
    
    # Auto-detect base directory
    if args.base_dir is None:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        args.base_dir = os.path.join(script_dir, '../../sessions/experiments')
    
    exp_dir = os.path.join(args.base_dir, args.experiment_id)
    if not os.path.exists(exp_dir):
        print(f"Error: Experiment directory not found: {exp_dir}")
        return 1
    
    os.makedirs(args.output, exist_ok=True)
    
    output_path = os.path.join(args.output, f'multi_rotation_{args.object}.pdf')
    
    print("="*70)
    print("MULTI-ROTATION ANALYSIS")
    print("="*70)
    print(f"Experiment: {args.experiment_id}")
    print(f"Object: {args.object}")
    print(f"Rotations: {args.rotations}")
    print(f"Checkpoint: {args.checkpoint}")
    print(f"Output: {output_path}")
    print("="*70)
    
    print("\nIMPORTANT: Make sure Gazebo and ROS are running!")
    print("This will spawn objects and test grasps automatically.\n")
    
    input("Press Enter to start multi-rotation test...")
    
    try:
        tester = MultiRotationTester(exp_dir, args.checkpoint)
        success_rate = tester.visualize_multi_rotation(args.object, args.rotations, output_path)
        
        print(f"\n{'='*70}")
        print(f"RESULTS: {success_rate:.0f}% success rate across {args.rotations} rotations")
        print(f"{'='*70}\n")
        
        return 0
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == '__main__':
    sys.exit(main())
