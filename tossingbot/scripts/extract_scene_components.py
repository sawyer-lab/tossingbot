#!/usr/bin/env python3.8
"""
Extract RGB and depth components from saved heightmap scene.
"""
import sys
import torch
import numpy as np
import matplotlib.pyplot as plt

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('scene', help='Saved heightmap (.pt file)')
    parser.add_argument('--output-dir', required=True, help='Output directory')
    args = parser.parse_args()
    
    import os
    os.makedirs(args.output_dir, exist_ok=True)
    
    # Load heightmap
    heightmap = torch.load(args.scene)
    
    # Extract RGB (first 3 channels)
    rgb = (heightmap[0:3].numpy().transpose(1,2,0) * 255).astype(np.uint8)
    
    # Extract depth (4th channel)
    depth = heightmap[3].numpy()
    
    # Save RGB
    rgb_path = os.path.join(args.output_dir, 'scene_rgb.png')
    plt.figure(figsize=(5, 5))
    plt.imshow(rgb)
    plt.title('RGB Input', fontsize=12)
    plt.axis('off')
    plt.tight_layout()
    plt.savefig(rgb_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"✓ RGB saved: {rgb_path}")
    
    # Save depth
    depth_path = os.path.join(args.output_dir, 'scene_depth.png')
    plt.figure(figsize=(5, 5))
    plt.imshow(depth, cmap='gray')
    plt.title('Depth Input', fontsize=12)
    plt.colorbar(label='Height (m)')
    plt.axis('off')
    plt.tight_layout()
    plt.savefig(depth_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"✓ Depth saved: {depth_path}")
    
    print(f"\n✓ All components saved to: {args.output_dir}")

if __name__ == '__main__':
    main()
