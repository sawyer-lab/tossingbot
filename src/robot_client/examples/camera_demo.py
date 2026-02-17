#!/usr/bin/env python3
"""
RGBD Camera Demo

Demonstrates point cloud capture and streaming capabilities.
Shows how to:
- Request point clouds on-demand
- Enable/disable streaming mode
- Visualize point cloud data with matplotlib
- Project depth to 2D image

Requirements:
    - Gazebo simulator running with RGBD cameras
    - ZMQ server running in container
    - matplotlib (optional, for visualization)
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from robot_client import RGBDCameraClient
import time
import numpy as np

try:
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    print("Warning: matplotlib not available, skipping visualizations")


def print_section(title):
    """Print section header."""
    print("\n" + "="*70)
    print(f"  {title}")
    print("="*70)


def visualize_point_cloud_3d(cloud, subsample=1000):
    """
    Visualize point cloud using matplotlib 3D scatter plot.
    
    Args:
        cloud: Point cloud dict with 'points' and 'colors' keys
        subsample: Number of points to display (for performance)
    """
    if not MATPLOTLIB_AVAILABLE:
        print("   Matplotlib not available, skipping 3D visualization")
        return
    
    points = cloud['points']
    colors = cloud['colors']
    
    if len(points) > subsample:
        indices = np.random.choice(len(points), subsample, replace=False)
        points = points[indices]
        colors = colors[indices]
    
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], 
               c=colors, s=1, alpha=0.6)
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(f'Point Cloud ({len(points)} points shown)')
    
    ax.set_box_aspect([1, 1, 0.5])
    
    plt.tight_layout()
    plt.show(block=False)
    plt.pause(0.1)


def visualize_depth_projection(cloud, resolution=(480, 640)):
    """
    Project point cloud to depth image and visualize.
    
    Args:
        cloud: Point cloud dict with 'points' and 'colors' keys
        resolution: Output image resolution (height, width)
    """
    if not MATPLOTLIB_AVAILABLE:
        print("   Matplotlib not available, skipping depth projection")
        return
    
    points = cloud['points']
    colors = cloud['colors']
    
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    
    x_min, x_max = x.min(), x.max()
    y_min, y_max = y.min(), y.max()
    
    h, w = resolution
    u = ((x - x_min) / (x_max - x_min) * (w - 1)).astype(int)
    v = ((y - y_min) / (y_max - y_min) * (h - 1)).astype(int)
    
    u = np.clip(u, 0, w - 1)
    v = np.clip(v, 0, h - 1)
    
    depth_image = np.zeros((h, w), dtype=np.float32)
    color_image = np.zeros((h, w, 3), dtype=np.float32)
    count_image = np.zeros((h, w), dtype=np.int32)
    
    for i in range(len(points)):
        depth_image[v[i], u[i]] += z[i]
        color_image[v[i], u[i]] += colors[i]
        count_image[v[i], u[i]] += 1
    
    mask = count_image > 0
    depth_image[mask] /= count_image[mask]
    color_image[mask] /= count_image[mask, None]
    
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    
    im1 = axes[0].imshow(depth_image, cmap='viridis')
    axes[0].set_title('Depth Projection')
    axes[0].axis('off')
    plt.colorbar(im1, ax=axes[0], label='Z depth (m)')
    
    axes[1].imshow(color_image)
    axes[1].set_title('Color Projection')
    axes[1].axis('off')
    
    plt.tight_layout()
    plt.show(block=False)
    plt.pause(0.1)


def main():
    print_section("RGBD Camera Demo")

    print("\n[1] Connecting to RGBD camera...")
    camera = RGBDCameraClient(protocol='zmq', host='localhost')
    print("    Connected")

    print_section("On-Demand Point Cloud Capture")
    
    print("\n   Capturing point cloud...")
    cloud = camera.get_point_cloud()
    
    if cloud:
        points = cloud['points']
        colors = cloud['colors']
        timestamp = cloud['timestamp']
        
        print(f"   Point cloud captured:")
        print(f"     Points: {len(points)} points")
        print(f"     Shape:  {points.shape}")
        print(f"     Range:  X=[{points[:,0].min():.3f}, {points[:,0].max():.3f}]")
        print(f"             Y=[{points[:,1].min():.3f}, {points[:,1].max():.3f}]")
        print(f"             Z=[{points[:,2].min():.3f}, {points[:,2].max():.3f}]")
        print(f"     Colors: {colors.shape}, range=[{colors.min():.3f}, {colors.max():.3f}]")
        print(f"     Time:   {timestamp:.3f}")
        
        if MATPLOTLIB_AVAILABLE:
            print("\n   Visualizing point cloud...")
            print("   (Close matplotlib windows to continue)")
            
            print("\n   3D scatter plot (subsampled to 1000 points)...")
            visualize_point_cloud_3d(cloud, subsample=1000)
            
            print("   2D depth/color projection...")
            visualize_depth_projection(cloud, resolution=(480, 640))
            
            input("\n   Press Enter to close visualizations and continue...")
            plt.close('all')
        else:
            print("\n   Install matplotlib for visualizations: pip install matplotlib")
    else:
        print("   No point cloud data available")
        print("   (Check that RGBD cameras are active in Gazebo)")

    print_section("Streaming Mode")
    
    print("\n   Streaming is disabled by default to save bandwidth.")
    print("   When enabled, point clouds appear in state broadcast on port 5556.")
    print("   Warning: This adds ~1-20 MB per frame at 100Hz!")
    
    print("\n   Enable streaming? (y/n): ", end='')
    choice = input().strip().lower()
    
    if choice == 'y':
        print("\n   Enabling streaming...")
        success = camera.enable_streaming()
        print(f"   Status: {'enabled' if success else 'failed'}")
        
        print("\n   Streaming for 3 seconds...")
        print("   (Point clouds now included in state broadcast)")
        time.sleep(3)
        
        print("\n   Disabling streaming...")
        camera.disable_streaming()
        print("   Streaming stopped")
    else:
        print("\n   Skipping streaming demo")

    print_section("Convenience Methods")
    
    print("\n   capture_scene() is an alias for get_point_cloud():")
    scene = camera.capture_scene()
    if scene:
        print(f"   Captured scene with {len(scene['points'])} points")
    else:
        print("   No scene data")

    print_section("Demo Complete")
    print("\n   RGBD camera functionality demonstrated.")
    
    if MATPLOTLIB_AVAILABLE:
        print("   Visualizations displayed using matplotlib.")
    else:
        print("   For visualization, install: pip install matplotlib")
        print("   Or export point cloud to PLY and use Open3D/CloudCompare.")
    print()

    camera.close()
    
    if MATPLOTLIB_AVAILABLE:
        plt.close('all')


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n[Interrupted] Demo stopped")
    except Exception as e:
        print(f"\n\n[ERROR] Demo failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
