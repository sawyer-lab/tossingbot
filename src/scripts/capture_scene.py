#!/usr/bin/env python3.8
"""
Capture and save heightmap from current scene.
Run this while training/demo is active to capture a scene snapshot.
"""
import sys
import os
import torch
import rospy

sys.path.insert(0, '/geometry2_ws/devel/lib/python3/dist-packages')
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(script_dir, '..'))

from tossingbot.perception.ros_camera import RosCamera
from tossingbot.perception.vision import VisionProcessor

def main():
    output_file = sys.argv[1] if len(sys.argv) > 1 else "scene_snapshot.pt"
    
    try:
        rospy.init_node('capture_scene', anonymous=True, disable_signals=True)
    except:
        pass
    
    camera = RosCamera()
    vision = VisionProcessor()
    
    print("Waiting for camera data...")
    while camera.get_latest_cloud()[0] is None and not rospy.is_shutdown():
        rospy.sleep(0.1)
    
    print("Capturing scene...")
    pts, cols = camera.get_latest_cloud()
    heightmap = vision.process(pts, cols)
    
    torch.save(heightmap, output_file)
    print(f"✓ Saved heightmap to: {output_file}")
    print(f"  Shape: {heightmap.shape}")

if __name__ == '__main__':
    main()
