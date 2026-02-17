"""
GripperClient - Gripper control
"""

from typing import Dict
from .base_client import BaseClient


class GripperClient(BaseClient):
    """
    Gripper control client.

    Controls the robot gripper (open, close, set position).
    """

    def open(self) -> bool:
        """Open the gripper fully."""
        return self._client.gripper_open()

    def close(self) -> bool:
        """Close the gripper (or until grasping object)."""
        return self._client.gripper_close()

    def set_position(self, position: float) -> bool:
        """
        Set gripper to specific position.

        Args:
            position: Gripper width in meters (0.0 = closed, 0.041667 = open)
        """
        return self._client.gripper_set_position(position)

    def get_state(self) -> Dict:
        """Get gripper state (position, is_grasping)."""
        return self._client.gripper_get_state()
