#!/usr/bin/env python3
"""
RGBD Camera Manager

Manages RGBD camera data from ROS topics.
Synchronizes dual camera point clouds and provides numpy arrays.
"""

import rospy
import threading
import numpy as np
import message_filters
import ros_numpy
import tf2_ros
from sensor_msgs.msg import PointCloud2


class RGBDCameraManager:
    """
    Manages RGBD camera data from synchronized point cloud topics.
    
    Subscribes to left and right camera point clouds, synchronizes them,
    transforms to base frame, and provides thread-safe access to combined
    point cloud data.
    """
    
    def __init__(self, 
                 left_topic="/rgbd_camera_left/depth/points",
                 right_topic="/rgbd_camera_right/depth/points",
                 base_frame="base"):
        """
        Initialize RGBD camera manager.
        
        Args:
            left_topic: ROS topic for left camera point cloud
            right_topic: ROS topic for right camera point cloud
            base_frame: Target frame for point cloud transformation
        """
        self._lock = threading.Lock()
        self.base_frame = base_frame
        self.active = True
        
        # Latest point cloud data
        self._latest_points = None
        self._latest_colors = None
        self._latest_timestamp = None
        
        # TF buffer for coordinate transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Synchronized subscribers for left and right cameras
        left_sub = message_filters.Subscriber(left_topic, PointCloud2)
        right_sub = message_filters.Subscriber(right_topic, PointCloud2)
        
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [left_sub, right_sub], 
            queue_size=10, 
            slop=0.2
        )
        self.synchronizer.registerCallback(self._point_cloud_callback)
        
        rospy.loginfo(f"RGBDCameraManager initialized: {left_topic}, {right_topic}")
    
    def _point_cloud_callback(self, msg_left, msg_right):
        """
        Synchronized callback for left and right camera point clouds.
        
        Args:
            msg_left: PointCloud2 message from left camera
            msg_right: PointCloud2 message from right camera
        """
        if not self.active:
            return
        
        # Convert left camera
        pts_left, colors_left = self._msg_to_numpy(msg_left)
        if pts_left is None:
            return
        
        # Convert right camera
        pts_right, colors_right = self._msg_to_numpy(msg_right)
        if pts_right is None:
            return
        
        # Combine left and right point clouds
        with self._lock:
            self._latest_points = np.vstack((pts_left, pts_right))
            self._latest_colors = np.vstack((colors_left, colors_right))
            self._latest_timestamp = rospy.Time.now().to_sec()
        
        rospy.logdebug(f"Point cloud updated: {len(self._latest_points)} points")
    
    def _msg_to_numpy(self, msg):
        """
        Convert PointCloud2 message to numpy arrays.
        
        Args:
            msg: PointCloud2 ROS message
        
        Returns:
            tuple: (points, colors) as numpy arrays, or (None, None) on error
                points: (N, 3) float32 array of x,y,z coordinates
                colors: (N, 3) float32 array of r,g,b values [0-1]
        """
        try:
            # Lookup transform from camera frame to base frame
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                msg.header.frame_id,
                rospy.Time(0),
                rospy.Duration(0.1)
            )
            
            # Convert ROS message to numpy structured array
            pc = ros_numpy.numpify(msg).flatten()
            
            # Extract XYZ coordinates
            points = np.zeros((len(pc), 3), dtype=np.float32)
            points[:, 0] = pc['x']
            points[:, 1] = pc['y']
            points[:, 2] = pc['z']
            
            # Extract RGB colors
            # Point cloud RGB is packed as uint32: 0x00RRGGBB
            rgb = pc['rgb'].view(np.uint32)
            r = ((rgb >> 16) & 0xFF) / 255.0
            g = ((rgb >> 8) & 0xFF) / 255.0
            b = (rgb & 0xFF) / 255.0
            colors = np.stack([r, g, b], axis=-1).astype(np.float32)
            
            # Apply transform to points
            points_transformed = self._apply_transform(points, transform)
            
            return points_transformed, colors
            
        except Exception as e:
            rospy.logwarn(f"Failed to convert point cloud: {e}")
            return None, None
    
    def _apply_transform(self, points, transform_msg):
        """
        Apply TF transform to point cloud.
        
        Args:
            points: (N, 3) numpy array of points
            transform_msg: TransformStamped message
        
        Returns:
            numpy array: Transformed points (N, 3)
        """
        # Extract translation
        t = np.array([
            transform_msg.transform.translation.x,
            transform_msg.transform.translation.y,
            transform_msg.transform.translation.z
        ])
        
        # Extract rotation quaternion
        q = [
            transform_msg.transform.rotation.x,
            transform_msg.transform.rotation.y,
            transform_msg.transform.rotation.z,
            transform_msg.transform.rotation.w
        ]
        
        # Convert quaternion to rotation matrix
        x, y, z, w = q
        R = np.array([
            [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
            [    2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
            [    2*x*z - 2*y*w,     2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
        ])
        
        # Apply transform: R * points + t
        return points @ R.T + t
    
    def enable(self):
        """Enable point cloud processing."""
        with self._lock:
            self.active = True
            rospy.loginfo("RGBD camera enabled")
    
    def disable(self):
        """Disable point cloud processing to save CPU."""
        with self._lock:
            self.active = False
            rospy.loginfo("RGBD camera disabled")
    
    def get_point_cloud(self):
        """
        Get the latest point cloud data.
        
        Returns:
            dict: Point cloud data with keys:
                - points: (N, 3) numpy array of xyz coordinates
                - colors: (N, 3) numpy array of rgb values [0-1]
                - timestamp: float (seconds since epoch)
            None if no data available
        """
        with self._lock:
            if self._latest_points is None:
                return None
            
            return {
                'points': self._latest_points.copy(),
                'colors': self._latest_colors.copy(),
                'timestamp': self._latest_timestamp
            }
    
    def get_point_cloud_raw(self):
        """
        Get raw point cloud arrays without copying.
        
        Use with caution - arrays may be updated by callback thread.
        
        Returns:
            tuple: (points, colors, timestamp) or (None, None, None)
        """
        with self._lock:
            return self._latest_points, self._latest_colors, self._latest_timestamp
    
    def is_active(self):
        """
        Check if camera processing is active.
        
        Returns:
            bool: True if active, False otherwise
        """
        with self._lock:
            return self.active
    
    def has_data(self):
        """
        Check if point cloud data is available.
        
        Returns:
            bool: True if data available, False otherwise
        """
        with self._lock:
            return self._latest_points is not None
