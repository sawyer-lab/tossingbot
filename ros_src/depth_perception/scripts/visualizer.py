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

        # Pixel-based grid
        self.W = rospy.get_param("~width", 640)   # match camera
        self.H = rospy.get_param("~height", 480)

        self.bridge = CvBridge()

        if self.visualize:
            cv2.namedWindow("Heightmap", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Color Heightmap", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Side-by-side", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Overlay", cv2.WINDOW_NORMAL)

        rospy.Subscriber("xyzrgb_cloud", XYZRGBArray, self.cb, queue_size=1)

        rospy.loginfo("Visualizer started: %dx%d pixel grid", self.W, self.H)

    def cb(self, msg):
        if len(msg.x) == 0:
            return

        # Convert lists to numpy
        xs = np.array(msg.x, dtype=np.int32)
        ys = np.array(msg.y, dtype=np.int32)
        zs = np.array(msg.z, dtype=np.float32)

        rs = np.array(msg.r, dtype=np.uint8)
        gs = np.array(msg.g, dtype=np.uint8)
        bs = np.array(msg.b, dtype=np.uint8)

        heightmap = np.full((self.H, self.W), -np.inf)
        colormap  = np.zeros((self.H, self.W, 3), dtype=np.uint8)

        for x, y, z, r, g, b in zip(xs, ys, zs, rs, gs, bs):

            if x < 0 or y < 0 or x >= self.W or y >= self.H:
                continue

            if z > heightmap[y, x]:
                heightmap[y, x] = z
                colormap[y, x] = [r, g, b]

        heightmap[heightmap == -np.inf] = np.nan

        if self.visualize:
            self.display(heightmap, colormap)

    def display(self, heightmap, colormap):

        hm = heightmap.copy()
        hm[np.isnan(hm)] = 0

        if np.nanmax(heightmap) > np.nanmin(heightmap):
            hm_norm = (255 * (hm - np.nanmin(heightmap)) /
                       (np.nanmax(heightmap) - np.nanmin(heightmap))).astype(np.uint8)
        else:
            hm_norm = np.zeros_like(hm, dtype=np.uint8)

        cv2.imshow("Heightmap", hm_norm)
        cv2.imshow("Color Heightmap", colormap)

        side = np.hstack([
            cv2.cvtColor(hm_norm, cv2.COLOR_GRAY2BGR),
            colormap
        ])
        cv2.imshow("Side-by-side", side)

        heat = cv2.applyColorMap(hm_norm, cv2.COLORMAP_JET)
        overlay = cv2.addWeighted(colormap, 0.6, heat, 0.4, 0)
        cv2.imshow("Overlay", overlay)

        cv2.waitKey(1)


if __name__ == "__main__":
    XYZRGBVisualizer()
    rospy.spin()
