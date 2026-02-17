#!/usr/bin/env python3
"""
Live Camera Stream Viewer

Real-time visualization of RGBD camera using OpenCV.
Shows depth and color projections side-by-side.

Requirements:
    - Gazebo simulator with RGBD cameras
    - ZMQ server running
    - opencv-python (cv2)
    - numpy
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from robot_client import RGBDCameraClient
import numpy as np
import time

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    print("ERROR: OpenCV not available")
    print("Install with: pip install opencv-python")
    sys.exit(1)


def project_to_image(cloud, resolution=(480, 640)):
    """
    Project point cloud to 2D depth and color images.
    
    Args:
        cloud: Point cloud dict with 'points' and 'colors'
        resolution: Output image size (height, width)
    
    Returns:
        tuple: (depth_image, color_image) as numpy arrays
    """
    points = cloud['points']
    colors = cloud['colors']
    
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    
    x_min, x_max = x.min(), x.max()
    y_min, y_max = y.min(), y.max()
    z_min, z_max = z.min(), z.max()
    
    h, w = resolution
    
    u = ((x - x_min) / (x_max - x_min + 1e-6) * (w - 1)).astype(int)
    v = ((y - y_min) / (y_max - y_min + 1e-6) * (h - 1)).astype(int)
    
    u = np.clip(u, 0, w - 1)
    v = np.clip(v, 0, h - 1)
    
    depth_accum = np.zeros((h, w), dtype=np.float32)
    color_accum = np.zeros((h, w, 3), dtype=np.float32)
    count = np.zeros((h, w), dtype=np.int32)
    
    for i in range(len(points)):
        depth_accum[v[i], u[i]] += z[i]
        color_accum[v[i], u[i]] += colors[i]
        count[v[i], u[i]] += 1
    
    mask = count > 0
    depth_img = np.zeros((h, w), dtype=np.float32)
    color_img = np.zeros((h, w, 3), dtype=np.float32)
    
    depth_img[mask] = depth_accum[mask] / count[mask]
    color_img[mask] = color_accum[mask] / count[mask, None]
    
    depth_normalized = ((depth_img - z_min) / (z_max - z_min + 1e-6) * 255).astype(np.uint8)
    depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
    
    color_img_bgr = (color_img[:, :, ::-1] * 255).astype(np.uint8)
    
    return depth_colored, color_img_bgr


def main():
    print("="*70)
    print("  Live RGBD Camera Stream")
    print("="*70)
    print("\nConnecting to camera...")
    
    camera = RGBDCameraClient(protocol='zmq', host='localhost')
    print("Connected")
    
    print("\nControls:")
    print("  Press 'q' to quit")
    print("  Press 's' to save snapshot")
    print("  Press SPACE to pause/resume")
    print("\nStarting stream...")
    
    paused = False
    frame_count = 0
    last_cloud = None
    
    fps_start = time.time()
    fps_frames = 0
    current_fps = 0.0
    
    while True:
        if not paused:
            cloud = camera.get_point_cloud()
            
            if cloud:
                last_cloud = cloud
                fps_frames += 1
                
                if time.time() - fps_start >= 1.0:
                    current_fps = fps_frames / (time.time() - fps_start)
                    fps_frames = 0
                    fps_start = time.time()
        
        if last_cloud:
            depth_img, color_img = project_to_image(last_cloud, resolution=(480, 640))
            
            combined = np.hstack([depth_img, color_img])
            
            status_text = f"FPS: {current_fps:.1f} | Points: {len(last_cloud['points'])}"
            if paused:
                status_text = "PAUSED | " + status_text
            
            cv2.putText(combined, status_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(combined, "Depth", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            cv2.putText(combined, "Color", (650, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            
            cv2.imshow('RGBD Camera Stream', combined)
        else:
            blank = np.zeros((480, 1280, 3), dtype=np.uint8)
            cv2.putText(blank, "Waiting for camera data...", (400, 240),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            cv2.imshow('RGBD Camera Stream', blank)
        
        key = cv2.waitKey(30) & 0xFF
        
        if key == ord('q'):
            print("\nQuitting...")
            break
        elif key == ord('s') and last_cloud:
            filename = f"snapshot_{frame_count:04d}.png"
            cv2.imwrite(filename, combined)
            print(f"Saved: {filename}")
            frame_count += 1
        elif key == ord(' '):
            paused = not paused
            print("Paused" if paused else "Resumed")
    
    cv2.destroyAllWindows()
    camera.close()
    print("Stream closed")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nInterrupted")
        cv2.destroyAllWindows()
    except Exception as e:
        print(f"\n\nERROR: {e}")
        import traceback
        traceback.print_exc()
        cv2.destroyAllWindows()
        sys.exit(1)
