#!/usr/bin/env python3.8
"""
Visualize network evolution from a saved heightmap image.
"""
import sys
import os
import torch
import numpy as np
import cv2
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

sys.path.insert(0, '/geometry2_ws/devel/lib/python3/dist-packages')
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(script_dir, '..'))

from tossingbot import config as cfg
from tossingbot.learning.agent import TossingAgent

plt.rcParams.update({'font.size': 10, 'figure.dpi': 150, 'savefig.dpi': 300})

def load_checkpoint_and_predict(checkpoint_path, heightmap, device):
    """Load checkpoint and run inference."""
    if not os.path.exists(checkpoint_path):
        return None, None, None, None
    
    # Minimal session
    class DummySession:
        def __init__(self):
            self.metadata = {}
        def get_checkpoint_path(self, name):
            return checkpoint_path
        def save_hyperparameters(self, h):
            pass
        def save_metadata(self):
            pass
    
    agent = TossingAgent(DummySession(), load_buffer=False, load_weights=False)
    checkpoint = torch.load(checkpoint_path, map_location=device)
    agent.model.load_state_dict(checkpoint['model'])
    agent.model.eval()
    
    with torch.no_grad():
        q_maps = agent.model(heightmap.unsqueeze(0).to(device))  # Raw network output
        q_maps = torch.sigmoid(q_maps)  # Apply sigmoid like training display
        q_maps = q_maps.squeeze(0).cpu().numpy()
    
    max_q_map = np.max(q_maps, axis=0)
    best_pixel = np.unravel_index(np.argmax(max_q_map), max_q_map.shape)
    best_rot = np.argmax(q_maps, axis=0)[best_pixel]
    
    return max_q_map, best_pixel, best_rot, np.max(max_q_map)

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('image', help='Saved heightmap (.pt file)')
    parser.add_argument('--checkpoints', default='500,1000,1500,2000')
    parser.add_argument('--exp-dir', required=True)
    parser.add_argument('--output-dir', default=None, help='Output directory (default: exp_dir/analysis)')
    args = parser.parse_args()
    
    # Default output to experiment analysis directory
    if args.output_dir is None:
        args.output_dir = os.path.join(args.exp_dir, 'analysis')
    os.makedirs(args.output_dir, exist_ok=True)
    
    heightmap = torch.load(args.image)
    rgb = (heightmap[0:3].numpy().transpose(1,2,0) * 255).astype(np.uint8)
    
    checkpoints = [int(x) for x in args.checkpoints.split(',')]
    ckpt_dir = os.path.join(args.exp_dir, 'train/checkpoints')
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    
    print(f"Generating network evolution images...")
    
    for step in checkpoints:
        ckpt_path = os.path.join(ckpt_dir, f'checkpoint_step_{step}.pth')
        if not os.path.exists(ckpt_path):
            ckpt_path = os.path.join(ckpt_dir, f'checkpoint_{step}.pth')
        
        heatmap, pixel, rot, maxq = load_checkpoint_and_predict(ckpt_path, heightmap, device)
        if heatmap is None:
            print(f"Skipping step {step}")
            continue
        
        print(f"Step {step}: Q={maxq:.2f}")
        
        # Save just the inference heatmap
        output_path = os.path.join(args.output_dir, f'network_step_{step}.png')
        plt.figure(figsize=(5, 5))
        plt.imshow(heatmap, cmap='jet', vmin=0, vmax=1)
        plt.title(f'Network Inference - Step {step} | Max Q={maxq:.2f}', fontsize=12)
        plt.colorbar(label='Grasp Quality')
        plt.axis('off')
        plt.tight_layout()
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        plt.close()
        print(f"  Saved: {output_path}")
    
    print(f"\n✓ All images saved to: {args.output_dir}")

if __name__ == '__main__':
    main()
