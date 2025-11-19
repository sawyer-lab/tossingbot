#!/usr/bin/env python
import rospy
import struct
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from depth_perception.msg import XYZRGBArray

import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs as tf2_sm

class XYZRGBPublisher(object):
    def __init__(self):
        rospy.init_node("transform_and_publish_xyzrgb")
        self.target_frame = rospy.get_param("~target_frame", "base")
        self.pub = rospy.Publisher("xyzrgb_cloud", XYZRGBArray, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber("voxel_filtered_points", PointCloud2, self.cb, queue_size=1)
        rospy.loginfo("XYZRGBPublisher started, target_frame=%s", self.target_frame)

        self.saved_images = False

    def cb(self, cloud_msg):
        # Lookup transform and transform the cloud to target_frame
        try:
            trans = self.tf_buffer.lookup_transform(
                self.target_frame,
                cloud_msg.header.frame_id,
                rospy.Time(0),
                rospy.Duration(1.0)
            )
            cloud_tf = tf2_sm.do_transform_cloud(cloud_msg, trans)
        except Exception as e:
            rospy.logwarn("TF transform failed: %s", str(e))
            return

        points = []

        for p in pc2.read_points(cloud_tf, skip_nans=True, field_names=("x","y","z","rgb")):
            x, y, z, rgb_f = p
            r, g, b = self.unpack_rgb(rgb_f)
            points.append((x, y, z, r, g, b))

        if not points:
            return

        # Convert to columns
        xs, ys, zs, rs, gs, bs = zip(*points)

        if len(xs) == 0:
            rospy.logdebug("No points in transformed cloud.")
            return

        msg = XYZRGBArray()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.target_frame
        msg.x = list(xs)
        msg.y = list(ys)
        msg.z = list(zs)
        msg.r = list(rs)
        msg.g = list(gs)
        msg.b = list(bs)

        self.pub.publish(msg)


        if not self.saved_images:
            self.saved_images = True

            import cv2
            import numpy as np

            # Convert lists to numpy arrays
            xs_np = np.array(xs)
            ys_np = np.array(ys)
            zs_np = np.array(zs)
            rs_np = np.array(rs)
            gs_np = np.array(gs)
            bs_np = np.array(bs)

            # ----------- SIMPLE HEIGHTMAP (top-down grid) ------------
            # Define resolution
            res = 0.01  # meters per pixel
            x_min, x_max = xs_np.min(), xs_np.max()
            y_min, y_max = ys_np.min(), ys_np.max()

            H = int((y_max - y_min) / res) + 1
            W = int((x_max - x_min) / res) + 1

            heightmap = np.zeros((H, W), dtype=np.float32)

            # Fill heightmap (z values)
            for x, y, z in zip(xs_np, ys_np, zs_np):
                ix = int((x - x_min) / res)
                iy = int((y - y_min) / res)
                heightmap[iy, ix] = z  # overwrite ok

            # Normalize for viewing
            heightmap_norm = cv2.normalize(heightmap, None, 0, 255, cv2.NORM_MINMAX)
            heightmap_img = heightmap_norm.astype(np.uint8)

            

            cv2.imwrite("/home/kid/ros_ws/src/depth_perception/img/heightmap.png", heightmap_img)
            rospy.loginfo("Saved ../tmp/heightmap.png")

            # ----------- SAVE RGB IMAGE (point projection) ------------
            rgb_img = np.zeros((H, W, 3), dtype=np.uint8)

            for x, y, r, g, b in zip(xs_np, ys_np, rs_np, gs_np, bs_np):
                ix = int((x - x_min) / res)
                iy = int((y - y_min) / res)
                rgb_img[iy, ix] = (b, g, r)  # OpenCV uses BGR

            cv2.imwrite("/home/kid/ros_ws/src/depth_perception/img/rgb_image.png", rgb_img)
            rospy.loginfo("Saved ../tmp/rgb_image.png")

    def unpack_rgb(self, rgb_float):
        # PCL packs RGB in a float; unpack to uint8 r,g,b
        s = struct.pack('>f', rgb_float)
        i = struct.unpack('>I', s)[0]
        r = (i >> 16) & 0xFF
        g = (i >> 8) & 0xFF
        b = i & 0xFF
        return r, g, b

if __name__ == "__main__":
    XYZRGBPublisher()
    rospy.spin()