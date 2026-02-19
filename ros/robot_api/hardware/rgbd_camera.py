#!/usr/bin/env python3
"""RGBDCamera - Single RGBD camera subscriber (color + depth + point cloud).

Subscribes to the three standard topics published by cameras like
Intel RealSense or Kinect:

    {ns}/color/image_raw   - RGB image          (sensor_msgs/Image)
    {ns}/depth/image_raw   - depth map           (sensor_msgs/Image, float32 m or uint16 mm)
    {ns}/depth/points      - point cloud         (sensor_msgs/PointCloud2)

All three streams are independent; each is updated whenever a new message
arrives on its topic.  Data is accessed through thread-safe getters.
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import tf2_ros

try:
    import ros_numpy
    _HAS_ROS_NUMPY = True
except ImportError:
    _HAS_ROS_NUMPY = False
    rospy.logwarn("RGBDCamera: ros_numpy not found — point cloud support disabled")

from .base_camera import BaseCameraSubscriber


class RGBDCamera(BaseCameraSubscriber):
    """Thread-safe subscriber for one RGBD camera."""

    def __init__(self, namespace, base_frame='base'):
        """
        Args:
            namespace:   ROS topic namespace, e.g. '/rgbd_camera_left'
            base_frame:  TF frame that point clouds are transformed into
        """
        super().__init__()

        self.namespace = namespace
        self.base_frame = base_frame
        self._bridge = CvBridge()

        self._color_image = None        # BGR uint8 numpy array
        self._depth_image = None        # float32 metres numpy array
        self._cloud_points = None       # (N,3) float32 XYZ
        self._cloud_colors = None       # (N,3) float32 RGB [0–1]

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber(
            f"{namespace}/rgb/image_raw", Image,
            self._color_callback, queue_size=1, buff_size=2**24
        )
        rospy.Subscriber(
            f"{namespace}/depth/image_raw", Image,
            self._depth_callback, queue_size=1, buff_size=2**24
        )
        rospy.Subscriber(
            f"{namespace}/depth/points", PointCloud2,
            self._cloud_callback, queue_size=1
        )

        rospy.loginfo(f"RGBDCamera: subscribed to {namespace}")

    # ── Color ─────────────────────────────────────────────────────────────────

    def get_color_image(self):
        """Return latest color image as BGR numpy array, or None."""
        with self._lock:
            if self._color_image is None:
                rospy.logwarn_throttle(2.0, f"RGBDCamera ({self.namespace}): no color image yet")
                return None
            return self._color_image.copy()

    def get_color_image_compressed(self):
        """Return latest color image as JPEG bytes, or None."""
        img = self.get_color_image()
        if img is None:
            return None
        _, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 85])
        return buf.tobytes()

    # ── Depth ─────────────────────────────────────────────────────────────────

    def get_depth_image(self):
        """Return latest depth map as float32 numpy array in metres, or None."""
        with self._lock:
            if self._depth_image is None:
                rospy.logwarn_throttle(2.0, f"RGBDCamera ({self.namespace}): no depth image yet")
                return None
            return self._depth_image.copy()

    def get_depth_image_compressed(self):
        """
        Return depth image as lossless 16-bit PNG bytes (uint16, millimetres).

        Decode on the client with:
            nparr = np.frombuffer(bytes.fromhex(hex_str), np.uint8)
            depth_m = cv2.imdecode(nparr, cv2.IMREAD_ANYDEPTH) / 1000.0
        """
        depth = self.get_depth_image()
        if depth is None:
            return None
        depth_mm = np.clip(depth * 1000.0, 0, 65535).astype(np.uint16)
        _, buf = cv2.imencode('.png', depth_mm)
        return buf.tobytes()

    # ── Point cloud ───────────────────────────────────────────────────────────

    def get_point_cloud(self):
        """
        Return latest point cloud as dict, or None.

        Keys:
            points:    (N,3) float32 XYZ in base_frame
            colors:    (N,3) float32 RGB [0–1]
            timestamp: float seconds
        """
        with self._lock:
            if self._cloud_points is None:
                return None
            return {
                'points':    self._cloud_points.copy(),
                'colors':    self._cloud_colors.copy(),
                'timestamp': self._latest_timestamp,
            }

    # ── has_data ──────────────────────────────────────────────────────────────

    def has_data(self):
        with self._lock:
            return (
                self._color_image is not None
                or self._depth_image is not None
                or self._cloud_points is not None
            )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _color_callback(self, msg):
        if not self._active:
            return
        try:
            img = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
            with self._lock:
                self._color_image = img
                self._latest_timestamp = msg.header.stamp.to_sec()
        except Exception as e:
            rospy.logerr(f"RGBDCamera ({self.namespace}): color conversion error: {e}")

    def _depth_callback(self, msg):
        if not self._active:
            return
        try:
            if msg.encoding == '32FC1':
                depth = self._bridge.imgmsg_to_cv2(msg, '32FC1')
            else:
                # 16UC1 — millimetres → metres
                depth_mm = self._bridge.imgmsg_to_cv2(msg, '16UC1')
                depth = depth_mm.astype(np.float32) / 1000.0
            with self._lock:
                self._depth_image = depth
        except Exception as e:
            rospy.logerr(f"RGBDCamera ({self.namespace}): depth conversion error: {e}")

    def _cloud_callback(self, msg):
        if not self._active or not _HAS_ROS_NUMPY:
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, msg.header.frame_id,
                rospy.Time(0), rospy.Duration(0.1)
            )
            pc = ros_numpy.numpify(msg).flatten()

            points = np.zeros((len(pc), 3), dtype=np.float32)
            points[:, 0] = pc['x']
            points[:, 1] = pc['y']
            points[:, 2] = pc['z']

            rgb = pc['rgb'].view(np.uint32)
            r = ((rgb >> 16) & 0xFF) / 255.0
            g = ((rgb >> 8)  & 0xFF) / 255.0
            b = (rgb & 0xFF)         / 255.0
            colors = np.stack([r, g, b], axis=-1).astype(np.float32)

            points = self._apply_transform(points, transform)

            with self._lock:
                self._cloud_points = points
                self._cloud_colors = colors
                self._latest_timestamp = msg.header.stamp.to_sec()
        except Exception as e:
            rospy.logwarn(f"RGBDCamera ({self.namespace}): point cloud error: {e}")

    # ── Helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _apply_transform(points, transform_msg):
        t = np.array([
            transform_msg.transform.translation.x,
            transform_msg.transform.translation.y,
            transform_msg.transform.translation.z,
        ])
        x = transform_msg.transform.rotation.x
        y = transform_msg.transform.rotation.y
        z = transform_msg.transform.rotation.z
        w = transform_msg.transform.rotation.w
        R = np.array([
            [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
            [    2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
            [    2*x*z - 2*y*w,     2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y],
        ])
        return points @ R.T + t
