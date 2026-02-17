"""
LightsClient - Robot lights control
"""

from typing import List
from .base_client import BaseClient


class LightsClient(BaseClient):
    """
    Lights control client.

    Controls robot lights (navigator, cuff, head).
    """

    def list_all(self) -> List[str]:
        """List all available lights."""
        return self._client.lights_list()

    def set(self, name: str, on: bool = True) -> bool:
        """Set light state (on=True, off=False)."""
        return self._client.lights_set(name, on)

    def get(self, name: str) -> bool:
        """Get light state."""
        return self._client.lights_get(name)
