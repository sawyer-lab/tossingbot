#!/usr/bin/env python3
"""
Gripper - Clean interface for Sawyer gripper control

Intera SDK-style API that hides ROS complexity.
This class wraps the GripperInterface for use by communication servers.
"""

import rospy
import json
import sensor_msgs.msg
from intera_core_msgs.msg import IOComponentCommand


class GripperInterface:
    MAX_POSITION = 0.041667
    MIN_POSITION = 0.0
    LEFT_FINGER  = 'right_gripper_l_finger_joint'
    RIGHT_FINGER = 'right_gripper_r_finger_joint'

    def __init__(self):
        self._cmd_topic = '/io/end_effector/right_gripper/command'
        self._pub = rospy.Publisher(self._cmd_topic, IOComponentCommand, queue_size=1)
        self._sub = rospy.Subscriber('/robot/joint_states', sensor_msgs.msg.JointState, self._joint_callback)
        self._current_width = -1.0
        self._cmd = IOComponentCommand()
        self._cmd.op = 'set'
        rospy.loginfo("GripperInterface: Ready (JointState Mode).")

    def _joint_callback(self, msg):
        try:
            if self.LEFT_FINGER in msg.name and self.RIGHT_FINGER in msg.name:
                idx_l = msg.name.index(self.LEFT_FINGER)
                idx_r = msg.name.index(self.RIGHT_FINGER)
                pos_l = msg.position[idx_l]
                pos_r = msg.position[idx_r]
                self._current_width = abs(pos_l) + abs(pos_r)
        except ValueError:
            pass

    def open(self):
        self._send_position(self.MAX_POSITION)

    def close(self):
        self._send_position(self.MIN_POSITION)

    def set_position(self, position: float):
        pos = max(self.MIN_POSITION, min(self.MAX_POSITION, position))
        self._send_position(pos)

    def _send_position(self, pos_value):
        cmd_struct = {
            "signals": {
                "position_m": {
                    "format": {"type": "float"},
                    "data": [pos_value]
                }
            }
        }
        self._cmd.time = rospy.Time.now()
        self._cmd.args = json.dumps(cmd_struct)
        self._pub.publish(self._cmd)

    def is_grasping(self) -> bool:
        width = self.get_current_position()
        if width < 0: return False
        if width > 0.002 and width < (self.MAX_POSITION - 0.005):
            return True
        return False

    def get_current_position(self) -> float:
        return self._current_width


class Gripper:
    """
    Clean gripper interface - Intera SDK style

    Provides simple methods for gripper control without exposing ROS internals.
    Designed to be used by communication servers (ZMQ, HTTP, WebSocket).
    """

    def __init__(self):
        """Initialize gripper interface."""
        rospy.loginfo("Gripper: Initializing...")
        self._gripper = GripperInterface()
        rospy.sleep(0.5)  # Allow time for first state callback
        rospy.loginfo("Gripper: Ready")

    def open(self) -> bool:
        """
        Open the gripper fully.

        Returns:
            True if command sent successfully
        """
        try:
            self._gripper.open()
            rospy.sleep(1.0)  # Allow time for gripper to open
            return True
        except Exception as e:
            rospy.logerr(f"Gripper.open: Error - {e}")
            return False

    def close(self) -> bool:
        """
        Close the gripper fully (or until it grasps an object).

        Returns:
            True if command sent successfully
        """
        try:
            self._gripper.close()
            rospy.sleep(1.0)  # Allow time for gripper to close
            return True
        except Exception as e:
            rospy.logerr(f"Gripper.close: Error - {e}")
            return False

    def set_position(self, position: float) -> bool:
        """
        Set gripper to a specific position.

        Args:
            position: Gripper width in meters (0.0 = closed, 0.041667 = fully open)

        Returns:
            True if command sent successfully
        """
        try:
            self._gripper.set_position(position)
            rospy.sleep(1.0)  # Allow time for gripper to move
            return True
        except Exception as e:
            rospy.logerr(f"Gripper.set_position: Error - {e}")
            return False

    def is_grasping(self) -> bool:
        """
        Check if gripper is currently holding an object.

        Returns:
            True if gripper appears to be grasping something
        """
        return self._gripper.is_grasping()

    def get_position(self) -> float:
        """
        Get current gripper position.

        Returns:
            Current gripper width in meters (-1.0 if unknown)
        """
        return self._gripper.get_current_position()

    def get_state(self) -> dict:
        """
        Get complete gripper state.

        Returns:
            Dict with position and grasping status
        """
        return {
            'position': self.get_position(),
            'is_grasping': self.is_grasping(),
        }


if __name__ == "__main__":
    """Simple test of Gripper interface"""
    rospy.init_node("gripper_test")

    gripper = Gripper()

    # Test open
    print("\n[1] Opening gripper...")
    gripper.open()
    print(f"    Position: {gripper.get_position():.5f} m")
    print(f"    Grasping: {gripper.is_grasping()}")

    # Test close
    print("\n[2] Closing gripper...")
    gripper.close()
    print(f"    Position: {gripper.get_position():.5f} m")
    print(f"    Grasping: {gripper.is_grasping()}")

    # Test halfway
    print("\n[3] Setting to halfway position...")
    gripper.set_position(0.02)
    print(f"    Position: {gripper.get_position():.5f} m")
    print(f"    Grasping: {gripper.is_grasping()}")

    # Open again
    print("\n[4] Opening gripper...")
    gripper.open()

    print("\n[PASS] Test complete")
