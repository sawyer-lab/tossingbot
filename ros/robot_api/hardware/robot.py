#!/usr/bin/env python3
"""
Robot - Clean interface for Sawyer robot control

Intera SDK-style API that hides ROS complexity.
This class wraps the SawyerInterface for use by communication servers.
"""

import rospy
import sys
import os
from typing import List, Dict, Optional

from .sawyer import SawyerInterface


class Robot:
    """
    Clean robot interface - Intera SDK style

    Provides simple methods for robot control without exposing ROS internals.
    Designed to be used by communication servers (ZMQ, HTTP, WebSocket).
    """

    def __init__(self):
        """Initialize robot interface. Blocks until robot is ready."""
        rospy.loginfo("Robot: Initializing...")
        self._sawyer = SawyerInterface()
        rospy.loginfo("Robot: Ready")

    def move_to_joints(self, angles: List[float], timeout: float = 30.0) -> bool:
        """
        Move robot to target joint positions.

        Args:
            angles: List of 7 joint angles in radians [j0, j1, ..., j6]
            timeout: Maximum time to wait for motion to complete (seconds)

        Returns:
            True if motion completed successfully, False if timeout
        """
        if len(angles) != 7:
            rospy.logerr(f"Robot.move_to_joints: Expected 7 angles, got {len(angles)}")
            return False

        try:
            self._sawyer.move_to_joint_positions(angles, timeout=timeout)
            return True
        except Exception as e:
            rospy.logerr(f"Robot.move_to_joints: Error - {e}")
            return False

    def get_joint_angles(self) -> List[float]:
        """
        Get current joint angles.

        Returns:
            List of 7 joint angles in radians [j0, j1, ..., j6]
        """
        return self._sawyer.get_joint_positions()

    def get_joint_velocities(self) -> List[float]:
        """
        Get current joint velocities.

        Returns:
            List of 7 joint velocities in rad/s
        """
        return self._sawyer.get_joint_velocities()

    def get_joint_efforts(self) -> List[float]:
        """
        Get current joint efforts (torques).

        Returns:
            List of 7 joint efforts in Nm
        """
        return self._sawyer.get_joint_efforts()

    def get_endpoint_pose(self) -> Optional[Dict]:
        """
        Get end-effector pose (position + orientation).

        Returns:
            Dict with 'position': [x, y, z] and 'orientation': [x, y, z, w]
            or None if not available
        """
        return self._sawyer.get_endpoint_pose()

    def get_endpoint_velocity(self) -> Optional[Dict]:
        """
        Get end-effector velocity (linear + angular).

        Returns:
            Dict with 'linear': [vx, vy, vz] and 'angular': [wx, wy, wz]
            or None if not available
        """
        return self._sawyer.get_endpoint_velocity()

    def get_endpoint_effort(self) -> Optional[Dict]:
        """
        Get end-effector effort (force + torque).

        Returns:
            Dict with 'force': [fx, fy, fz] and 'torque': [tx, ty, tz]
            or None if not available
        """
        return self._sawyer.get_endpoint_effort()

    def execute_trajectory(self, waypoints: List[Dict], rate_hz: float = 100.0) -> bool:
        """
        Execute a full trajectory at the given rate using TRAJECTORY mode.

        Each waypoint must have 'position', 'velocity', 'acceleration' keys
        with lists of 7 floats each.

        Args:
            waypoints: List of waypoint dicts.
            rate_hz: Execution rate (currently fixed at 100Hz by hardware).

        Returns:
            True if trajectory completed, False on error.
        """
        from .sawyer import RobotCommand, ControlMode

        if not waypoints:
            rospy.logwarn("Robot.execute_trajectory: Empty waypoint list")
            return False

        try:
            stream = [
                RobotCommand(
                    position=wp['position'],
                    velocity=wp.get('velocity', []),
                    acceleration=wp.get('acceleration', []),
                )
                for wp in waypoints
            ]
            return self._sawyer.execute_stream(stream, ControlMode.TRAJECTORY)
        except Exception as e:
            rospy.logerr(f"Robot.execute_trajectory: Error - {e}")
            return False

    def get_state(self) -> Dict:
        """
        Get complete robot state.

        Returns:
            Dict containing all robot state information
        """
        return {
            'joint_angles': self.get_joint_angles(),
            'joint_velocities': self.get_joint_velocities(),
            'joint_efforts': self.get_joint_efforts(),
            'endpoint_pose': self.get_endpoint_pose(),
            'endpoint_velocity': self.get_endpoint_velocity(),
            'endpoint_effort': self.get_endpoint_effort(),
        }


if __name__ == "__main__":
    """Simple test of Robot interface"""
    rospy.init_node("robot_test")

    robot = Robot()

    # Get current state
    print("Current joint angles:", robot.get_joint_angles())
    print("Endpoint pose:", robot.get_endpoint_pose())

    # Move to a target position
    target = [0.0, 0.3, 0.5, 0.0, 0.0, -0.2, 0.0]
    print(f"\nMoving to: {target}")
    success = robot.move_to_joints(target)

    if success:
        print("Movement completed successfully")
    else:
        print("Movement timed out or failed")

    print("\nFinal joint angles:", robot.get_joint_angles())
