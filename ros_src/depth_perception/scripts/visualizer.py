#!/usr/bin/env python
#encoding: utf-8
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from depth_perception.msg import XYZRGBArray

class XYZRGBVisualizer(object):
    def __init__(self):
        rospy.init_node("xyzrgb_visualizer")
        
        self.visualize = rospy.get_param("~visualize", True)
        
        # Grid dimensions in pixels
        self.W = rospy.get_param("~width", 400)
        self.H = rospy.get_param("~height", 400)
        
        # Spatial bounds in meters
        self.xmin = rospy.get_param("~xmin", -1.0)
        self.xmax = rospy.get_param("~xmax",  1.0)
        self.ymin = rospy.get_param("~ymin", -1.0)
        self.ymax = rospy.get_param("~ymax",  1.0)

        self.bridge = CvBridge()

        # Publishers
        self.pub_heightmap = rospy.Publisher("heightmap_image", Image, queue_size=1)
        self.pub_color = rospy.Publisher("heightmap_color", Image, queue_size=1)
        
        # Create resizable windows if visualizing
        if self.visualize:
            cv2.namedWindow("Heightmap", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Color Heightmap", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Side-by-side", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Overlay RGB + Height", cv2.WINDOW_NORMAL)


        rospy.Subscriber("xyzrgb_cloud", XYZRGBArray, self.cb, queue_size=1)
        rospy.loginfo("XYZRGBVisualizer started: grid=%dx%d, bounds=[%.2f,%.2f]x[%.2f,%.2f]", 
                      self.W, self.H, self.xmin, self.xmax, self.ymin, self.ymax)

    def cb(self, msg):
        if len(msg.x) == 0:
            return

        # Convert message arrays to numpy
        xs = np.array(msg.x, dtype=np.float32)
        ys = np.array(msg.y, dtype=np.float32)
        zs = np.array(msg.z, dtype=np.float32)
        
        # In ROS Python 2, uint8[] comes as bytes/str, need to convert
        rs = np.frombuffer(msg.r, dtype=np.uint8)
        gs = np.frombuffer(msg.g, dtype=np.uint8)
        bs = np.frombuffer(msg.b, dtype=np.uint8)

        # Stack into Nx6 array
        pts = np.column_stack((xs, ys, zs, rs, gs, bs))

        # Build heightmap and colormap
        heightmap = np.full((self.H, self.W), -np.inf)
        colormap = np.zeros((self.H, self.W, 3), dtype=np.uint8)

        # Insert points (max Z per cell)
        for x, y, z, r, g, b in pts:
            ix = int(x - self.xmin) 
            iy = int(y - self.ymin) 
            
            if ix < 0 or iy < 0 or ix >= self.W or iy >= self.H:
                continue
                
            if z > heightmap[iy, ix]:
                heightmap[iy, ix] = z
                colormap[iy, ix] = [r, g, b]

        # Replace -inf with nan
        heightmap[heightmap == -np.inf] = np.nan

        # # Publish as ROS Image messages
        # self.publish_heightmap(heightmap, colormap)

        # Visualize with OpenCV
        if self.visualize:
            self.display(heightmap, colormap)

    def publish_heightmap(self, heightmap, colormap):
        hm = heightmap.copy()
        hm[np.isnan(hm)] = 0

        # Normalize height for visual image
        if np.nanmax(heightmap) > np.nanmin(heightmap):
            hm_norm = (255 * (hm - np.nanmin(heightmap)) /
                      (np.nanmax(heightmap) - np.nanmin(heightmap))).astype(np.uint8)
        else:
            hm_norm = np.zeros_like(hm, dtype=np.uint8)

        # Publish heightmap
        img_msg = self.bridge.cv2_to_imgmsg(hm_norm, encoding="mono8")
        img_msg.header.stamp = rospy.Time.now()
        self.pub_heightmap.publish(img_msg)

        # Publish colormap
        color_msg = self.bridge.cv2_to_imgmsg(colormap, encoding="rgb8")
        color_msg.header.stamp = rospy.Time.now()
        self.pub_color.publish(color_msg)

    def display(self, heightmap, colormap):
        hm = heightmap.copy()
        hm[np.isnan(hm)] = 0

        # Normalize for visualization
        if np.nanmax(heightmap) > np.nanmin(heightmap):
            hm_norm = (255 * (hm - np.nanmin(heightmap)) /
                      (np.nanmax(heightmap) - np.nanmin(heightmap))).astype(np.uint8)
        else:
            hm_norm = np.zeros_like(hm, dtype=np.uint8)

        # A) Two separate windows
        cv2.imshow("Heightmap", hm_norm)
        cv2.imshow("Color Heightmap", colormap)

        # B) Side-by-side window
        side = np.hstack([
            cv2.cvtColor(hm_norm, cv2.COLOR_GRAY2BGR),
            colormap
        ])
        cv2.imshow("Side-by-side", side)

        # C) Overlay heightmap → RGB (heatmap on top of color)
        heat = cv2.applyColorMap(hm_norm, cv2.COLORMAP_JET)
        overlay = cv2.addWeighted(colormap, 0.6, heat, 0.4, 0)
        cv2.imshow("Overlay RGB + Height", overlay)

        cv2.waitKey(1)

if __name__ == "__main__":
    XYZRGBVisualizer()
    rospy.spin()