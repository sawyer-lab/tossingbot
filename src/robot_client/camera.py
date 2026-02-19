"""
CameraClient - Camera control
"""

from .base_client import BaseClient


class CameraClient(BaseClient):
    """
    Camera control client.

    Controls camera streaming and image capture.
    Camera names:
        CameraClient.HEAD  — head-mounted camera
        CameraClient.HAND  — wrist / end-effector camera
    """

    HEAD = 'head'
    HAND = 'hand'

    def start(self, camera=None) -> bool:
        """Start camera streaming."""
        return self._client.camera_start(camera=camera)

    def stop(self, camera=None) -> bool:
        """Stop camera streaming."""
        return self._client.camera_stop(camera=camera)

    def get_image(self, camera=None):
        """
        Get current camera image.

        Returns:
            numpy array (H, W, 3) BGR format, or None if unavailable
        """
        return self._client.camera_get_image(camera=camera)

    def get_state(self, camera=None):
        """
        Get camera state (streaming flag, has_image, topic).

        Returns:
            dict with 'streaming', 'has_image', 'topic', or None on error
        """
        return self._client.camera_get_state(camera=camera)

    def get_camera_names(self):
        """Get available camera names."""
        return self._client.params_camera_names()
