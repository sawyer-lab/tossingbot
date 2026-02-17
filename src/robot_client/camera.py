"""
CameraClient - Camera control
"""

from .base_client import BaseClient


class CameraClient(BaseClient):
    """
    Camera control client.

    Controls camera streaming and image capture.
    """

    def start(self) -> bool:
        """Start camera streaming."""
        return self._client.camera_start()

    def stop(self) -> bool:
        """Stop camera streaming."""
        return self._client.camera_stop()

    def get_image(self):
        """
        Get current camera image.

        Returns:
            numpy array (H, W, 3) BGR format, or None if unavailable
        """
        return self._client.camera_get_image()

    def get_camera_names(self):
        """Get available camera names."""
        return self._client.params_camera_names()
