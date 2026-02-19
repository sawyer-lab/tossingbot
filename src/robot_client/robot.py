"""
RobotClient - Robot arm control
"""

from typing import List, Dict, Optional
from .base_client import BaseClient


class RobotClient(BaseClient):
    """
    Robot arm control client.

    Controls the robot arm (joint positions, endpoint pose, etc.)
    """

    def move_to_joints(self, angles: List[float], timeout: float = 30.0) -> bool:
        """Move robot to target joint positions."""
        return self._client.move_to_joints(angles, timeout)

    def get_joint_angles(self) -> List[float]:
        """Get current joint angles."""
        return self._client.get_joint_angles()

    def get_endpoint_pose(self) -> Optional[Dict]:
        """Get end-effector pose."""
        return self._client.get_endpoint_pose()

    def get_state(self) -> Dict:
        """Get complete robot state."""
        return self._client.get_robot_state()

    def get_robot_name(self) -> str:
        """Get robot name (e.g., 'sawyer')."""
        return self._client.params_robot_name()

    def get_limb_names(self) -> List[str]:
        """Get limb names."""
        return self._client.params_limb_names()

    def get_joint_names(self, limb='right') -> List[str]:
        """Get joint names for limb."""
        return self._client.params_joint_names(limb)

    def get_joint_velocities(self) -> List[float]:
        """Get current joint velocities."""
        return self._client.get_joint_velocities()

    def get_endpoint_velocity(self) -> Optional[Dict]:
        """Get end-effector velocity (linear and angular)."""
        return self._client.get_endpoint_velocity()

    def execute_trajectory(self, waypoints: List[Dict], rate_hz: float = 100.0) -> bool:
        """
        Send a full trajectory for the container to execute at the given rate.

        Args:
            waypoints: List of dicts with 'position', 'velocity', 'acceleration' keys.
                       Each value is a list of 7 floats.
            rate_hz: Execution rate in Hz (default: 100).

        Returns:
            bool: Success
        """
        return self._client.execute_trajectory(waypoints, rate_hz)

    def execute_toss_trajectory(self, Q, Qd, Qdd, release_index=None) -> bool:
        """
        Execute toss trajectory at 100Hz with async gripper release.

        Runs the 100Hz command loop inside the container and opens the gripper
        at the specified step without interrupting the trajectory.

        Args:
            Q:   (N, 7) joint positions (list or numpy array).
            Qd:  (N, 7) joint velocities.
            Qdd: (N, 7) joint accelerations.
            release_index: Step at which to open the gripper asynchronously.
                           Pass None to skip gripper release.

        Returns:
            bool: Success
        """
        return self._client.execute_toss_trajectory(Q, Qd, Qdd, release_index)

    def gripper_open(self) -> bool:
        """Open the gripper fully."""
        return self._client.gripper_open()

    def gripper_close(self) -> bool:
        """Close the gripper."""
        return self._client.gripper_close()

    def gripper_set_position(self, position: float) -> bool:
        """Set gripper position (0.0=closed, 0.041667=open)."""
        return self._client.gripper_set_position(position)

    def gripper_get_state(self) -> Dict:
        """Get gripper state dict."""
        return self._client.gripper_get_state()
