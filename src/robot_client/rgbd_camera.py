"""
RGBDCameraClient - RGBD camera control and point cloud capture

Interfaces with RGBD cameras to capture point clouds and depth images.
"""

import numpy as np
from .base_client import BaseClient


class RGBDCameraClient(BaseClient):
    """
    RGBD camera client for point cloud capture.
    
    Provides access to synchronized dual RGBD camera data including
    point clouds (xyz + rgb) from Gazebo simulation.
    """
    
    def get_point_cloud(self):
        """
        Request a single point cloud capture.
        
        Returns:
            dict: Point cloud data with keys:
                - points: numpy array (N, 3) of xyz coordinates
                - colors: numpy array (N, 3) of rgb values [0-1]
                - timestamp: Capture time (seconds)
            None if no data available
        """
        result = self._client.camera_get_point_cloud()
        if not result or result.get('status') != 'ok':
            return None
        
        return {
            'points': np.array(result['points'], dtype=np.float32),
            'colors': np.array(result['colors'], dtype=np.float32),
            'timestamp': result['timestamp']
        }
    
    def enable_streaming(self):
        """
        Enable point cloud streaming in state broadcast.
        
        When enabled, point cloud data will be included in the
        100Hz state stream on port 5556. Warning: this adds
        significant bandwidth (~1-20 MB per frame).
        
        Returns:
            bool: True if successful
        """
        return self._client.camera_enable_streaming()
    
    def disable_streaming(self):
        """
        Disable point cloud streaming.
        
        Point clouds will no longer be included in state broadcast.
        Use get_point_cloud() for on-demand capture.
        
        Returns:
            bool: True if successful
        """
        return self._client.camera_disable_streaming()
    
    def capture_scene(self):
        """
        Convenience method to capture current scene.
        
        Alias for get_point_cloud() with clearer naming.
        
        Returns:
            dict: Point cloud data (see get_point_cloud)
        """
        return self.get_point_cloud()
