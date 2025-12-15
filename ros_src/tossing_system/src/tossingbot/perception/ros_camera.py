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
    """
    def __init__(self, 
                 left_topic="/rgbd_camera_left/depth/points",
                 right_topic="/rgbd_camera_right/depth/points",
                 base_frame="base"):
        
        self.base_frame = base_frame
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self._latest_points = None
        self._latest_colors = None
        
        left_sub = message_filters.Subscriber(left_topic, PointCloud2)
        right_sub = message_filters.Subscriber(right_topic, PointCloud2)
        
        self.ts = message_filters.ApproximateTimeSynchronizer([left_sub, right_sub], queue_size=10, slop=0.2)
        self.ts.registerCallback(self._cb)
        
        rospy.loginfo("RosCamera: Listening...")

    def get_latest_cloud(self):
        if self._latest_points is None: 
            return None, None
        return self._latest_points, self._latest_colors

    def _cb(self, msg_l, msg_r):
        pts_l, col_l = self._msg_to_numpy(msg_l)
        if pts_l is None: return

        pts_r, col_r = self._msg_to_numpy(msg_r)
        if pts_r is None: return

        self._latest_points = np.vstack((pts_l, pts_r))
        self._latest_colors = np.vstack((col_l, col_r))

    def _msg_to_numpy(self, msg):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.base_frame, 
                msg.header.frame_id, 
                rospy.Time(0), 
                rospy.Duration(0.1)
            )
            
            pc = ros_numpy.numpify(msg).flatten()
            
            points = np.zeros((len(pc), 3), dtype=np.float32)
            points[:,0] = pc['x']
            points[:,1] = pc['y']
            points[:,2] = pc['z']
            
            # --- COLOR CORRECTION ---
            rgb = pc['rgb'].view(np.uint32)
            # Bit shift to extract channels
            # Usually: 0x00RRGGBB
            c1 = ((rgb >> 16) & 0xFF) / 255.0
            c2 = ((rgb >> 8) & 0xFF) / 255.0
            c3 = (rgb & 0xFF) / 255.0
            

            colors = np.stack([c3, c2, c1], axis=-1)

            points_transformed = self._apply_transform(points, trans)
            
            return points_transformed, colors

        except Exception:
            return None, None

    def _apply_transform(self, points, trans_msg):
        t = np.array([
            trans_msg.transform.translation.x,
            trans_msg.transform.translation.y,
            trans_msg.transform.translation.z
        ])
        
        q = [
            trans_msg.transform.rotation.x,
            trans_msg.transform.rotation.y,
            trans_msg.transform.rotation.z,
            trans_msg.transform.rotation.w
        ]
        
        x, y, z, w = q
        R = np.array([
            [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
            [    2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
            [    2*x*z - 2*y*w,     2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
        ])

        return points @ R.T + t