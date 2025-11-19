#!/usr/bin/env python

import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2
import tf2_sensor_msgs.tf2_sensor_msgs as tf2_sm
import sensor_msgs.point_cloud2 as pc2
import struct
import numpy as np
import threading


class PerceptionHandler(object):
    def __init__(self):
        rospy.loginfo("Initializing PerceptionHandler...")

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Sync
        self.lock = threading.Lock()
        self.points = None  # Nx6 array (x,y,z,r,g,b)

        # Sub
        rospy.Subscriber("voxel_filtered_points",
                         PointCloud2,
                         self.cb,
                         queue_size=1)

        rospy.loginfo("PerceptionHandler initialized.")


    def cb(self, msg):
        # Try transform
        try:
            trans = self.tf_buffer.lookup_transform(
                "base",
                msg.header.frame_id,
                msg.header.stamp,          # use timestamp for safety
                rospy.Duration(0.5)
            )
            cloud_tf = tf2_sm.do_transform_cloud(msg, trans)
        except Exception as e:
            rospy.logwarn("TF transform failed: %s", str(e))
            return

        # Check RGB field exists
        field_names = [f.name for f in cloud_tf.fields]
        if "rgb" not in field_names:
            rospy.logwarn("PointCloud missing RGB field")
            return

        xyz = []
        rgb = []

        # Read points efficiently
        for p in pc2.read_points(cloud_tf,
                                 skip_nans=True,
                                 field_names=("x", "y", "z", "rgb")):
            x, y, z, rgb_f = p
            r, g, b = self.unpack_rgb(rgb_f)
            xyz.append([x, y, z])
            rgb.append([r, g, b])

        if not xyz:
            return

        # Convert to NumPy efficiently
        xyz = np.array(xyz, dtype=np.float32)
        rgb = np.array(rgb, dtype=np.uint8)
        points = np.hstack((xyz, rgb))

        # Thread-safe update
        with self.lock:
            self.points = points

    def get_last_image(self):
        with self.lock:
            if self.points is None:
                return None
            return self.points.copy()  # avoid race conditions


    def unpack_rgb(self, rgb_float):
        # Convert float->uint32 using little-endian
        i = struct.unpack('<I', struct.pack('<f', rgb_float))[0]
        r = (i >> 16) & 255
        g = (i >> 8) & 255
        b = i & 255
        return r, g, b



if __name__ == "__main__":
    rospy.init_node("perception_handler")
    handler = PerceptionHandler()

    SAVE_PATH = "/home/kid/ros_ws/src/depth_perception/img/"
    import os
    if not os.path.exists(SAVE_PATH):
        os.makedirs(SAVE_PATH)


    rate = rospy.Rate(10)

    saved = False

    while not rospy.is_shutdown():
        pts = handler.get_last_image()

        if pts is not None and not saved:
            # pts is Nx6 (x,y,z,r,g,b)
            xyz = pts[:, :3]
            rgb = pts[:, 3:]

            # --- Create simple projection images ---
            H, W = 480, 640
            xs, ys, zs = xyz[:, 0], xyz[:, 1], xyz[:, 2]

            # Normalize XY for 2D projection
            x_norm = (xs - xs.min()) / (xs.max() - xs.min() + 1e-9)
            y_norm = (ys - ys.min()) / (ys.max() - ys.min() + 1e-9)

            u = (x_norm * (W - 1)).astype(np.int32)
            v = (y_norm * (H - 1)).astype(np.int32)

            import cv2
            rgb_img = np.zeros((H, W, 3), dtype=np.uint8)
            depth_img = np.zeros((H, W), dtype=np.float32)

            for i in range(len(pts)):
                rgb_img[v[i], u[i]] = rgb[i]
                depth_img[v[i], u[i]] = zs[i]

            # Normalize depth to visible grayscale
            depth_norm = cv2.normalize(depth_img, None, 0, 255, cv2.NORM_MINMAX)
            depth_norm = depth_norm.astype(np.uint8)

            # Save
            cv2.imwrite(SAVE_PATH + "rgb_test.png", rgb_img)
            cv2.imwrite(SAVE_PATH + "depth_test.png", depth_norm)

            rospy.loginfo("Saved RGB and depth images to:")
            rospy.loginfo(SAVE_PATH + "rgb_test.png")
            rospy.loginfo(SAVE_PATH + "depth_test.png")

            saved = True  # avoid saving more than once

        rate.sleep()
