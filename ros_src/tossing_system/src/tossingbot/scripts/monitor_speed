#!/usr/bin/env python3.8
import sys
import os

# 1. PYTHON 3 COMPATIBILITY HACKS
sys.path.insert(0, '/geometry2_ws/devel/lib/python3/dist-packages')
sys.path.insert(0, '/cv_bridge_ws/devel/lib/python3/dist-packages')

import rospy
import tf2_ros
import sys

def monitor():
    rospy.init_node('monitor_tip_pose', anonymous=True)
    
    # 1. Setup TF Listener
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    
    # 2. Config
    target_frame = "right_gripper_tip"
    source_frame = "base"
    
    rate = rospy.Rate(10) # Update 10 times per second
    
    print(f"--- MONITORING POSE: {source_frame} -> {target_frame} ---")
    print("Waiting for transform...")

    while not rospy.is_shutdown():
        try:
            # Get latest transform
            trans = tf_buffer.lookup_transform(source_frame, target_frame, rospy.Time(0))
            
            pos = trans.transform.translation
            rot = trans.transform.rotation
            
            # Format output
            # \r overwrites the current line
            # \033[K clears the rest of the line
            sys.stdout.write(
                f"\rPOS: [X={pos.x:.3f}, Y={pos.y:.3f}, Z={pos.z:.3f}]  "
                f"ROT: [x={rot.x:.2f}, y={rot.y:.2f}, z={rot.z:.2f}, w={rot.w:.2f}]\033[K"
            )
            sys.stdout.flush()
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
            
        rate.sleep()

if __name__ == '__main__':
    monitor()