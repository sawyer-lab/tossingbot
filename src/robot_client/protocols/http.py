"""
HTTP Protocol Implementation

Simple REST API for robot control (~10-50ms latency).
Good for debugging and simple scripts.
"""

import requests
import time
from typing import List, Dict, Optional


class HTTPClient:
    """
    HTTP client for robot control.

    Simple REST API - easy to debug with curl.
    """

    def __init__(self, host='localhost', port=5000):
        """
        Initialize HTTP client.

        Args:
            host: Container hostname or IP
            port: HTTP server port
        """
        self.base_url = f"http://{host}:{port}"
        self.session = requests.Session()

        # Test connection
        try:
            response = self.session.get(f"{self.base_url}/ping", timeout=2)
            if response.status_code == 200:
                print(f"✓ HTTP Client connected to {self.base_url}")
            else:
                print(f"⚠ HTTP Server responded with status {response.status_code}")
        except requests.RequestException as e:
            print(f"⚠ Could not connect to {self.base_url}: {e}")

    def _post(self, endpoint: str, data: dict = None) -> Dict:
        """
        Send POST request to server.

        Args:
            endpoint: API endpoint (e.g., '/move_robot')
            data: Request data (JSON)

        Returns:
            Response dict
        """
        try:
            response = self.session.post(
                f"{self.base_url}{endpoint}",
                json=data,
                timeout=60
            )
            response.raise_for_status()
            return response.json()
        except requests.RequestException as e:
            return {'status': 'error', 'message': str(e)}

    def _get(self, endpoint: str) -> Dict:
        """
        Send GET request to server.

        Args:
            endpoint: API endpoint (e.g., '/get_angles')

        Returns:
            Response dict
        """
        try:
            response = self.session.get(
                f"{self.base_url}{endpoint}",
                timeout=10
            )
            response.raise_for_status()
            return response.json()
        except requests.RequestException as e:
            return {'status': 'error', 'message': str(e)}

    # ===== Robot Commands =====

    def move_to_joints(self, angles: List[float], timeout: float = 30.0) -> bool:
        """Move robot to target joint positions."""
        data = {'angles': angles, 'timeout': timeout}
        response = self._post('/move_robot', data)
        return response.get('status') == 'ok'

    def get_joint_angles(self) -> List[float]:
        """Get current joint angles."""
        response = self._get('/get_angles')
        if response.get('status') == 'ok':
            return response.get('angles', [])
        return []

    def get_endpoint_pose(self) -> Optional[Dict]:
        """Get end-effector pose."""
        response = self._get('/get_endpoint_pose')
        if response.get('status') == 'ok':
            return response.get('pose')
        return None

    def get_robot_state(self) -> Dict:
        """Get complete robot state."""
        response = self._get('/get_robot_state')
        if response.get('status') == 'ok':
            return response.get('state', {})
        return {}

    # ===== Gripper Commands =====

    def gripper_open(self) -> bool:
        """Open gripper."""
        response = self._post('/gripper_open')
        return response.get('status') == 'ok'

    def gripper_close(self) -> bool:
        """Close gripper."""
        response = self._post('/gripper_close')
        return response.get('status') == 'ok'

    def gripper_set_position(self, position: float) -> bool:
        """Set gripper position."""
        data = {'position': position}
        response = self._post('/gripper_set_position', data)
        return response.get('status') == 'ok'

    def gripper_get_state(self) -> Dict:
        """Get gripper state."""
        response = self._get('/gripper_get_state')
        if response.get('status') == 'ok':
            return response.get('state', {})
        return {}

    # ===== System Commands =====

    def get_full_state(self) -> Dict:
        """Get complete system state (robot + gripper)."""
        response = self._get('/get_full_state')
        if response.get('status') == 'ok':
            return response.get('state', {})
        return {}

    def ping(self) -> float:
        """
        Test connection latency.

        Returns:
            Round-trip time in milliseconds
        """
        start = time.time()
        response = self._get('/ping')
        latency_ms = (time.time() - start) * 1000

        if response.get('status') == 'ok':
            return latency_ms
        return -1.0

    def close(self):
        """Close connection."""
        self.session.close()
        print("✓ HTTP Client closed")


if __name__ == "__main__":
    """Simple test of HTTP client"""
    import numpy as np

    client = HTTPClient(host='localhost', port=5000)

    # Test ping
    print("\n[1] Testing connection...")
    latency = client.ping()
    print(f"    Latency: {latency:.2f} ms")

    # Get current state
    print("\n[2] Getting current state...")
    angles = client.get_joint_angles()
    print(f"    Joint angles: {np.round(angles, 3)}")

    # Move robot
    print("\n[3] Moving robot...")
    target = [0.0, 0.3, 0.5, 0.0, 0.0, -0.2, 0.0]
    success = client.move_to_joints(target, timeout=30.0)
    print(f"    Success: {success}")

    client.close()
    print("\n[PASS] HTTP client test complete")
