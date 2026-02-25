#!/usr/bin/env python3.8
"""
All-in-one script: Capture scene + Network evolution + Multi-rotation analysis
"""
import sys
import os
import torch
import numpy as np
import cv2
import matplotlib.pyplot as plt
import scipy.ndimage as ndimage

sys.path.insert(0, '/geometry2_ws/devel/lib/python3/dist-packages')
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(script_dir, '..'))

from tossingbot import config as cfg
from tossingbot.learning.agent import TossingAgent
from tossingbot.perception.ros_camera import RosCamera
from tossingbot.perception.vision import VisionProcessor

plt.rcParams.update({'font.size': 10, 'figure.dpi': 150, 'savefig.dpi': 300})

def capture_scene():
    """Capture current scene from ROS camera."""
    print("Capturing scene from camera...")
    import rospy
    try:
        rospy.init_node('capture_scene', anonymous=True, disable_signals=True)
    except:
        pass
    
    camera = RosCamera()
    vision = VisionProcessor()
    
    print("Waiting for camera data...")
    while camera.get_latest_cloud()[0] is None and not rospy.is_shutdown():
        rospy.sleep(0.1)
    
    pts, cols = camera.get_latest_cloud()
    heightmap = vision.process(pts, cols)
    
    print("✓ Scene captured")
    return heightmap

def rotate_image_tensor(image_tensor, angle_deg):
    """Rotate heightmap tensor by angle (degrees)."""
    rotated = []
    for c in range(image_tensor.shape[0]):
        channel = image_tensor[c].numpy()
        rotated_channel = ndimage.rotate(channel, angle_deg, reshape=False, order=1)
        rotated.append(torch.from_numpy(rotated_channel))
    return torch.stack(rotated)

def load_checkpoint_and_predict(checkpoint_path, heightmap, device):
    """Load checkpoint and run inference."""
    if not os.path.exists(checkpoint_path):
        return None, None, None, None
    
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
        q_maps = agent.model(heightmap.unsqueeze(0).to(device))
        q_maps = torch.sigmoid(q_maps)
        q_maps = q_maps.squeeze(0).cpu().numpy()
    
    max_q_map = np.max(q_maps, axis=0)
    best_pixel = np.unravel_index(np.argmax(max_q_map), max_q_map.shape)
    best_rot = np.argmax(q_maps, axis=0)[best_pixel]
    
    return max_q_map, best_pixel, best_rot, np.max(max_q_map)

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--exp-dir', required=True, help='Experiment directory')
    parser.add_argument('--checkpoints', default='500,1000,1500,2000', help='Checkpoints for evolution')
    parser.add_argument('--checkpoint', default='checkpoint_best.pth', help='Checkpoint for rotation')
    parser.add_argument('--output-dir', default=None, help='Output directory (default: exp_dir/analysis)')
    args = parser.parse_args()
    
    # Setup output directory
    if args.output_dir is None:
        args.output_dir = os.path.join(args.exp_dir, 'analysis')
    os.makedirs(args.output_dir, exist_ok=True)
    
    # Step 1: Capture scene
    heightmap = capture_scene()
    scene_path = os.path.join(args.output_dir, 'scene.pt')
    torch.save(heightmap, scene_path)
    print(f"✓ Scene saved: {scene_path}\n")
    
    rgb_original = (heightmap[0:3].numpy().transpose(1,2,0) * 255).astype(np.uint8)
    depth = heightmap[3].numpy()
    
    # Save RGB and depth components
    rgb_path = os.path.join(args.output_dir, 'scene_rgb.png')
    plt.figure(figsize=(5, 5))
    plt.imshow(rgb_original)
    plt.title('RGB Input', fontsize=12)
    plt.axis('off')
    plt.tight_layout()
    plt.savefig(rgb_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"✓ RGB saved: {rgb_path}")
    
    depth_path = os.path.join(args.output_dir, 'scene_depth.png')
    plt.figure(figsize=(5, 5))
    plt.imshow(depth, cmap='gray')
    plt.title('Depth Input', fontsize=12)
    plt.colorbar(label='Height (m)')
    plt.axis('off')
    plt.tight_layout()
    plt.savefig(depth_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"✓ Depth saved: {depth_path}\n")
    
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    
    # Step 2: Network Evolution
    print("=" * 60)
    print("NETWORK EVOLUTION")
    print("=" * 60)
    checkpoints = [int(x) for x in args.checkpoints.split(',')]
    ckpt_dir = os.path.join(args.exp_dir, 'train/checkpoints')
    
    for step in checkpoints:
        ckpt_path = os.path.join(ckpt_dir, f'checkpoint_step_{step}.pth')
        if not os.path.exists(ckpt_path):
            ckpt_path = os.path.join(ckpt_dir, f'checkpoint_{step}.pth')
        
        heatmap, pixel, rot, maxq = load_checkpoint_and_predict(ckpt_path, heightmap, device)
        if heatmap is None:
            print(f"Skipping step {step}")
            continue
        
        print(f"Step {step}: Q={maxq:.2f}")
        
        # Save inference heatmap
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
    
    # Step 3: Multi-Rotation Analysis
    print("\n" + "=" * 60)
    print("MULTI-ROTATION ANALYSIS")
    print("=" * 60)
    angles = [0, 45, 90, 135]
    
    ckpt_path = os.path.join(args.exp_dir, 'train/checkpoints', args.checkpoint)
    
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
    
    print("\n" + "=" * 60)
    print(f"✓ ALL FIGURES SAVED TO: {args.output_dir}")
    print("=" * 60)

if __name__ == '__main__':
    main()
