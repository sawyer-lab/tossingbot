"""
HeadClient - Head control (pan) and display
"""

from .base_client import BaseClient


class HeadClient(BaseClient):
    """
    Head control client.

    Controls head pan angle and head display screen.
    """

    def pan(self) -> float:
        """Get current head pan angle in radians."""
        return self._client.head_pan()

    def set_pan(self, angle: float, speed: float = 1.0) -> bool:
        """Set head pan angle."""
        return self._client.head_set_pan(angle, speed)

    def display_image(self, image):
        """
        Display image on head screen.

        Args:
            image: numpy array (H, W, 3) BGR format
        """
        return self._client.display_image(image)

    def display_clear(self) -> bool:
        """Clear head display."""
        return self._client.display_clear()
