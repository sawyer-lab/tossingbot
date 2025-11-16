#!/usr/bin/env python
#encoding: utf-8
import rospy
import numpy as np
import struct
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge
import cv2

import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs as tf2_sm

class HeightmapGeneratorTF:
    def __init__(self):
        rospy.init_node("heightmap_generator_tf")

        self.bridge = CvBridge()

        # ---- Parameters ----
        self.resolution = rospy.get_param("~resolution", 0.02)  # 2cm grid
        self.xmin = rospy.get_param("~xmin", -1.0)
        self.xmax = rospy.get_param("~xmax",  1.0)
        self.ymin = rospy.get_param("~ymin", -1.0)
        self.ymax = rospy.get_param("~ymax",  1.0)
        self.target_frame = rospy.get_param("~target_frame", "base")

        # ---- TF Buffer ----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # ---- Publisher ----
        self.pub_heightmap = rospy.Publisher("heightmap_image", Image, queue_size=1)
        self.pub_color = rospy.Publisher("heightmap_color", Image, queue_size=1)

        # ---- Subscriber ----
        rospy.Subscriber("voxel_filtered_points", PointCloud2, self.cloud_cb)

        rospy.loginfo("Heightmap generator with TF running.")

    # -----------------------------------------------------------
    # Callback: receive voxel filtered cloud
    # -----------------------------------------------------------
    def cloud_cb(self, cloud_msg):
        # --- Transform the cloud to base frame ---
        try:
            trans_cloud = tf2_sm.do_transform_cloud(
                cloud_msg,
                self.tf_buffer.lookup_transform(
                    self.target_frame,
                    cloud_msg.header.frame_id,
                    rospy.Time(0),
                    rospy.Duration(1.0)
                )
            )
        except Exception as e:
            rospy.logwarn("TF transform failed: %s", str(e))
            return

        # --- Convert cloud to numpy ---
        points_np = self.cloud_to_xyzrgb(trans_cloud)

        if points_np is None or len(points_np) == 0:
            rospy.logwarn("Empty pointcloud after TF.")
            return

        # --- Build heightmap ---
        heightmap, colormap = self.build_heightmap(points_np)

        # --- Publish heightmap ---
        self.publish_heightmap(heightmap, colormap)

    # -----------------------------------------------------------
    # Convert cloud fields -> numpy array [x,y,z,r,g,b]
    # -----------------------------------------------------------
    def cloud_to_xyzrgb(self, cloud_msg):
        points = pc2.read_points(cloud_msg, skip_nans=True,
                                 field_names=("x", "y", "z", "rgb"))

        lst = []
        for x, y, z, rgb in points:
            r, g, b = self.unpack_rgb(rgb)
            lst.append([x, y, z, r, g, b])

        return np.array(lst)

    # -----------------------------------------------------------
    # Build 2.5D heightmap
    # -----------------------------------------------------------
    def build_heightmap(self, points):
        W = int((self.xmax - self.xmin) / self.resolution)
        H = int((self.ymax - self.ymin) / self.resolution)

        heightmap = np.full((H, W), -np.inf)
        colormap = np.zeros((H, W, 3), dtype=np.uint8)

        for x, y, z, r, g, b in points:
            ix = int((x - self.xmin) / self.resolution)
            iy = int((y - self.ymin) / self.resolution)

            if ix < 0 or iy < 0 or ix >= W or iy >= H:
                continue

            # max height wins
            if z > heightmap[iy, ix]:
                heightmap[iy, ix] = z
                colormap[iy, ix] = [r, g, b]

        # Replace -inf with nan
        heightmap[heightmap == -np.inf] = np.nan
        return heightmap, colormap

    # -----------------------------------------------------------
    # Publish heightmap
    # -----------------------------------------------------------
    def publish_heightmap(self, heightmap, colormap):

        # ------------  PUBLISH  ------------
        hm = heightmap.copy()
        hm[np.isnan(hm)] = 0

        # Normalize height for visual image
        if np.nanmax(heightmap) > np.nanmin(heightmap):
            hm_norm = (255 * (hm - np.nanmin(hm)) /
                    (np.nanmax(hm) - np.nanmin(hm))).astype(np.uint8)
        else:
            hm_norm = np.zeros_like(hm, dtype=np.uint8)

        img_msg = self.bridge.cv2_to_imgmsg(hm_norm, encoding="mono8")
        img_msg.header.stamp = rospy.Time.now()

        color_msg = self.bridge.cv2_to_imgmsg(colormap, encoding="rgb8")
        color_msg.header.stamp = rospy.Time.now()

        self.pub_heightmap.publish(img_msg)
        self.pub_color.publish(color_msg)

        # ------------  VISUALIZE ------------
        

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


    # -----------------------------------------------------------
    # Helper: unpack PCL float32 RGB
    # -----------------------------------------------------------
    def unpack_rgb(self, rgb_float):
        s = struct.pack('>f', rgb_float)
        i = struct.unpack('>I', s)[0]
        r = (i >> 16) & 0xFF
        g = (i >> 8)  & 0xFF
        b =  i        & 0xFF
        return r, g, b


if __name__ == "__main__":
    HeightmapGeneratorTF()
    rospy.spin()
