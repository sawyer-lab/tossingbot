#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import sys

# Assume PerceptionHandler is in a file named perception_handler.py
# If it's in the same directory, this import works.
from perception_handler import PerceptionHandler

def main():
    rospy.init_node("perception_monitor_node")
    
    # 1. Initialize the Class
    handler = PerceptionHandler()
    
    # Allow some time for TF buffer to fill and connections to establish
    rospy.sleep(1.0)
    
    rate = rospy.Rate(15) # 15 Hz visualization
    
    print("--------------------------------------------------")
    print("Perception Monitor Started.")
    print("Press 'q' in the OpenCV window to exit.")
    print("--------------------------------------------------")

    while not rospy.is_shutdown():
        # 2. Get the 6-Channel Tensor
        # Shape: (H, W, 6) -> [X, Y, Z, R, G, B]
        tensor = handler.get_last_tensor()
        
        # Check if data is valid
        if tensor is None:
            rospy.loginfo_throttle(2, "Waiting for point cloud data...")
            rate.sleep()
            continue
            
        # Check if tensor is effectively empty (all zeros)
        if np.max(tensor) == 0:
            rospy.loginfo_throttle(2, "ROI is empty (no points detected).")
            # Create a black image just to keep window open
            h, w, _ = tensor.shape
            display_img = np.zeros((h, w*2, 3), dtype=np.uint8)
        else:
            # =================================================
            # VISUALIZATION LOGIC
            # =================================================
            
            # --- A. Process RGB Channels (3, 4, 5) ---
            # Extract normalized floats (0.0 - 1.0)
            rgb_floats = tensor[:, :, 3:6]
            
            # Convert to uint8 (0 - 255)
            rgb_uint8 = (rgb_floats * 255).astype(np.uint8)
            
            # OpenCV uses BGR, not RGB. We must swap channels.
            bgr_img = cv2.cvtColor(rgb_uint8, cv2.COLOR_RGB2BGR)
            
            # --- B. Process Depth Channel (2) ---
            # Extract Z meters
            z_meters = tensor[:, :, 2]
            
            # To visualize depth, we need to normalize it to 0-255.
            # We assume objects aren't taller than 0.4 meters for better contrast.
            max_visual_height = 0.4 
            z_norm = np.clip(z_meters / max_visual_height, 0.0, 1.0)
            z_uint8 = (z_norm * 255).astype(np.uint8)
            
            # Apply a colormap (JET makes low=blue, high=red)
            depth_colormap = cv2.applyColorMap(z_uint8, cv2.COLORMAP_JET)
            
            # --- C. Debug Info for Coordinate Channels (0, 1) ---
            # Let's inspect the center pixel to see real metric coordinates
            cy, cx = tensor.shape[0] // 2, tensor.shape[1] // 2
            metric_x = tensor[cy, cx, 0]
            metric_y = tensor[cy, cx, 1]
            
            # Draw a crosshair on the RGB image
            cv2.drawMarker(bgr_img, (cx, cy), (0, 255, 0), markerType=cv2.MARKER_CROSS, markerSize=10)
            
            # --- D. Stack Images Side-by-Side ---
            # Left: RGB, Right: Depth
            display_img = np.hstack((bgr_img, depth_colormap))
            
            # Upscale for easier viewing on screen (Optional)
            scale = 4
            h, w = display_img.shape[:2]
            display_img = cv2.resize(display_img, (w * scale, h * scale), interpolation=cv2.INTER_NEAREST)

            # Add Text Overlay
            cv2.putText(display_img, "RGB View", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(display_img, "Z-Height View", (10 + w*scale//2, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Display stats in console occasionally
            sys.stdout.write("\rCenter Pixel Metric Coords -> X: {:.3f}m, Y: {:.3f}m, Z: {:.3f}m   ".format(
                metric_x, metric_y, tensor[cy, cx, 2]))
            sys.stdout.flush()

        # 3. Show Window
        cv2.imshow("Neural Network Input Tensor", display_img)
        
        # Press 'q' to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
        rate.sleep()

    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass