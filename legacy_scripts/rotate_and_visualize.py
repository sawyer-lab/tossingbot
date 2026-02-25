#!/usr/bin/env python3.8
"""
Multi-rotation analysis by rotating the saved image (TossingBot approach).
"""
import sys
import os
import torch
import numpy as np
import cv2
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import scipy.ndimage as ndimage

sys.path.insert(0, '/geometry2_ws/devel/lib/python3/dist-packages')
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(script_dir, '..'))

from tossingbot import config as cfg
from tossingbot.learning.agent import TossingAgent

plt.rcParams.update({'font.size': 9, 'figure.dpi': 150, 'savefig.dpi': 300})

def rotate_image_tensor(image_tensor, angle_deg):
    """Rotate heightmap tensor by angle (degrees)."""
    # image_tensor: [C, H, W]
    rotated = []
    for c in range(image_tensor.shape[0]):
        channel = image_tensor[c].numpy()
        rotated_channel = ndimage.rotate(channel, angle_deg, reshape=False, order=1)
        rotated.append(torch.from_numpy(rotated_channel))
    return torch.stack(rotated)

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('image', help='Saved heightmap (.pt file)')
    parser.add_argument('--exp-dir', required=True)
    parser.add_argument('--checkpoint', default='checkpoint_best.pth')
    parser.add_argument('--output-dir', default=None, help='Output directory (default: exp_dir/analysis)')
    args = parser.parse_args()
    
    # Default output to experiment analysis directory
    if args.output_dir is None:
        args.output_dir = os.path.join(args.exp_dir, 'analysis')
    os.makedirs(args.output_dir, exist_ok=True)
    
    # Fixed rotation angles: 0, 45, 90, 135
    angles = [0, 45, 90, 135]
    
    # Load checkpoint
    ckpt_path = os.path.join(args.exp_dir, 'train/checkpoints', args.checkpoint)
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    
    class DummySession:
        def __init__(self):
            self.metadata = {}
        def get_checkpoint_path(self, n):
            return ckpt_path
        def save_hyperparameters(self, h):
            pass
        def save_metadata(self):
            pass
    
    agent = TossingAgent(DummySession(), load_buffer=False, load_weights=False)
    checkpoint = torch.load(ckpt_path, map_location=device)
    agent.model.load_state_dict(checkpoint['model'])
    agent.model.eval()
    
    # Load image
    heightmap = torch.load(args.image)
    rgb_original = (heightmap[0:3].numpy().transpose(1,2,0) * 255).astype(np.uint8)
    
    print(f"Testing {len(angles)} rotations: {angles}...")
    
    for angle in angles:
        print(f"  Rotation {angle:.0f}°...", end=' ')
        
        # Rotate heightmap for network inference
        rotated_heightmap = rotate_image_tensor(heightmap, angle)
        
        # Run inference on rotated input
        with torch.no_grad():
            q_maps = agent.model(rotated_heightmap.unsqueeze(0).to(device))
            q_maps = torch.sigmoid(q_maps)
            q_maps = q_maps.squeeze(0).cpu().numpy()
        
        max_q_map = np.max(q_maps, axis=0)
        maxq = np.max(max_q_map)
        
        print(f"Q={maxq:.2f}")
        
        # Extract rotated RGB for display
        rotated_rgb = (rotated_heightmap[0:3].numpy().transpose(1,2,0) * 255).astype(np.uint8)
        
        # Save rotated input image
        input_path = os.path.join(args.output_dir, f'rotated_input_{angle:03d}.png')
        plt.figure(figsize=(5, 5))
        plt.imshow(rotated_rgb)
        plt.title(f'{angle:.0f}° Rotated Input', fontsize=12)
        plt.axis('off')
        plt.tight_layout()
        plt.savefig(input_path, dpi=150, bbox_inches='tight')
        plt.close()
        
        # Save inference heatmap
        output_path = os.path.join(args.output_dir, f'rotation_{angle:03d}.png')
        plt.figure(figsize=(5, 5))
        plt.imshow(max_q_map, cmap='jet', vmin=0, vmax=1)
        plt.title(f'{angle:.0f}° Inference | Max Q={maxq:.2f}', fontsize=12)
        plt.colorbar(label='Grasp Quality')
        plt.axis('off')
        plt.tight_layout()
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        plt.close()
        print(f"  Saved: {input_path} & {output_path}")
    
    print(f"\n✓ All images saved to: {args.output_dir}")

if __name__ == '__main__':
    main()
