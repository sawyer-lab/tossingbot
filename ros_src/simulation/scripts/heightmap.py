#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import rospy
import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs as tf2sm
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import cv2


def cloud_to_heightmap(cloud, resolution=0.02):
    """
    Convert PointCloud2 (already in 'base' frame) into a 2.5D heightmap numpy array.
    """
    pts = list(pc2.read_points(cloud, field_names=["x", "y", "z"], skip_nans=True))
    if not pts:
        return None, None, None

    points = np.array(pts, dtype=np.float32)
    xs, ys, zs = points[:, 0], points[:, 1], points[:, 2]

    x_min, x_max = xs.min(), xs.max()
    y_min, y_max = ys.min(), ys.max()

    # Build grid bins
    x_bins = np.arange(x_min, x_max, resolution)
    y_bins = np.arange(y_min, y_max, resolution)
    heightmap = np.full((len(x_bins), len(y_bins)), np.nan, dtype=np.float32)

    # Fill heightmap (take max z per cell)
    for i in range(points.shape[0]):
        xi = int((xs[i] - x_min) / resolution)
        yi = int((ys[i] - y_min) / resolution)
        if 0 <= xi < len(x_bins) and 0 <= yi < len(y_bins):
            if np.isnan(heightmap[xi, yi]) or zs[i] > heightmap[xi, yi]:
                heightmap[xi, yi] = zs[i]

    # Normalize for visualization
    h_norm = np.nan_to_num(heightmap)
    h_norm[np.isnan(h_norm)] = 0.0
    if np.ptp(h_norm) > 1e-6:
        h_norm = (h_norm - np.nanmin(h_norm)) / (np.nanmax(h_norm) - np.nanmin(h_norm))
    h_img = (h_norm * 255).astype(np.uint8)


    return h_img, x_bins, y_bins


class HeightmapViewer(object):
    def __init__(self):
        rospy.init_node("heightmap_viewer")
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)
        self.target_frame = rospy.get_param("~target_frame", "base")
        self.resolution = rospy.get_param("~resolution", 0.02)

        self.sub = rospy.Subscriber("/voxel_filtered_points", PointCloud2, self.callback, queue_size=1)
        rospy.loginfo("Heightmap viewer subscribed to /voxel_filtered_points, frame=%s" % self.target_frame)

        cv2.namedWindow("2.5D Heightmap (base frame)", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("2.5D Heightmap (base frame)", 800, 800)


    def callback(self, cloud_msg):
        # Try to transform point cloud to the 'base' frame
        try:
            trans = self.tf_buf.lookup_transform(self.target_frame,
                                                 cloud_msg.header.frame_id,
                                                 rospy.Time(0),
                                                 rospy.Duration(0.5))
            cloud_base = tf2sm.do_transform_cloud(cloud_msg, trans)
        except Exception as e:
            rospy.logwarn("TF lookup failed: %s" % e)
            return

        h_img, _, _ = cloud_to_heightmap(cloud_base, self.resolution)
        if h_img is None:
            return

        h_color = cv2.applyColorMap(h_img, cv2.COLORMAP_JET)
        cv2.imshow("2.5D Heightmap (base frame)", h_color)
        cv2.waitKey(1)


if __name__ == "__main__":
    viewer = HeightmapViewer()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
