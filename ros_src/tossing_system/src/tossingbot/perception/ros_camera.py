#!/usr/bin/env python3.8
import rospy
import message_filters
import ros_numpy
import numpy as np
import tf2_ros
from sensor_msgs.msg import PointCloud2

class RosCamera:
    """
    Hardware Interface.
    Responsibilities:
    1. Listen to ROS Topics.
    2. Synchronize Left/Right cameras.
    3. Transform raw data into the Robot Base Frame.
    """
    def __init__(self, 
                 left_topic="/rgbd_camera_left/depth/points",
                 right_topic="/rgbd_camera_right/depth/points",
                 base_frame="base"):
        
        self.base_frame = base_frame
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # State: Raw Numpy Arrays (N, 3)
        self._latest_points = None
        self._latest_colors = None
        
        # Setup ROS Subscribers
        left_sub = message_filters.Subscriber(left_topic, PointCloud2)
        right_sub = message_filters.Subscriber(right_topic, PointCloud2)
        
        # Sync (Approximate Time)
        self.ts = message_filters.ApproximateTimeSynchronizer([left_sub, right_sub], queue_size=10, slop=0.2)
        self.ts.registerCallback(self._cb)
        
        rospy.loginfo("RosCamera: Listening...")

    def get_latest_cloud(self):
        """
        Returns:
            points (np.array): Shape (N, 3) in Base Frame
            colors (np.array): Shape (N, 3) Normalized 0-1
        OR (None, None) if not ready.
        """
        if self._latest_points is None: 
            return None, None
        return self._latest_points, self._latest_colors

    def _cb(self, msg_l, msg_r):
        # 1. Process Left
        pts_l, col_l = self._msg_to_numpy(msg_l)
        if pts_l is None: return

        # 2. Process Right
        pts_r, col_r = self._msg_to_numpy(msg_r)
        if pts_r is None: return

        # 3. Stack (Merge)
        self._latest_points = np.vstack((pts_l, pts_r))
        self._latest_colors = np.vstack((col_l, col_r))

    def _msg_to_numpy(self, msg):
        try:
            # 1. Get Transform (Base <- Camera)
            trans = self.tf_buffer.lookup_transform(
                self.base_frame, 
                msg.header.frame_id, 
                rospy.Time(0), 
                rospy.Duration(0.1)
            )
            
            # 2. Unpack Binary Data (Fastest method)
            pc = ros_numpy.numpify(msg).flatten()
            
            # Extract XYZ
            points = np.zeros((len(pc), 3), dtype=np.float32)
            points[:,0] = pc['x']
            points[:,1] = pc['y']
            points[:,2] = pc['z']
            
            # Extract RGB
            rgb = pc['rgb'].view(np.uint32)
            r = ((rgb >> 16) & 0xFF) / 255.0
            g = ((rgb >> 8) & 0xFF) / 255.0
            b = (rgb & 0xFF) / 255.0
            colors = np.stack([r, g, b], axis=-1)

            # 3. Apply Transform
            points_transformed = self._apply_transform(points, trans)
            
            return points_transformed, colors

        except Exception as e:
            return None, None

    def _apply_transform(self, points, trans_msg):
        # Extract Translation
        t = np.array([
            trans_msg.transform.translation.x,
            trans_msg.transform.translation.y,
            trans_msg.transform.translation.z
        ])
        
        # Extract Rotation (Quaternion -> Matrix)
        q = [
            trans_msg.transform.rotation.x,
            trans_msg.transform.rotation.y,
            trans_msg.transform.rotation.z,
            trans_msg.transform.rotation.w
        ]
        
        # Manual Rotation Matrix (No Scipy dependency)
        x, y, z, w = q
        R = np.array([
            [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
            [    2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
            [    2*x*z - 2*y*w,     2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
        ])

        # Apply: P_new = R * P_old + T
        return points @ R.T + t