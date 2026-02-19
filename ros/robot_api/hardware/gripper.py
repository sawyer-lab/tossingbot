#!/usr/bin/env python3
"""
Gripper — unified gripper interface for Sawyer.

Two backends, selected at construction time via mode=:

  mode='real'  (default)  — ClickSmart SmartToolPlate via SimpleClickSmartGripper
  mode='sim'              — Gazebo electric parallel gripper via right_gripper topics

Both expose the same public API so the ZMQ server is unaffected by the mode.

Usage:
    gripper = Gripper()            # real hardware
    gripper = Gripper(mode='sim')  # Gazebo simulation
"""

import json

import rospy
import sensor_msgs.msg

from intera_core_msgs.msg import IOComponentCommand

from .clicksmart_plate import SimpleClickSmartGripper


# ──────────────────────────────────────────────────────────────────────────────
# Simulation backend  (Gazebo electric parallel gripper)
# ──────────────────────────────────────────────────────────────────────────────

class _SimBackend:
    """Gazebo electric parallel gripper via right_gripper topics + joint_states."""

    MAX_POSITION = 0.041667
    MIN_POSITION = 0.0
    _LEFT  = 'right_gripper_l_finger_joint'
    _RIGHT = 'right_gripper_r_finger_joint'

    def __init__(self):
        self._pub = rospy.Publisher(
            '/io/end_effector/right_gripper/command',
            IOComponentCommand, queue_size=1)
        rospy.Subscriber('/robot/joint_states',
                         sensor_msgs.msg.JointState, self._joint_cb)
        self._width = -1.0
        rospy.loginfo("Gripper(sim): ready — Gazebo electric parallel gripper")

    def _joint_cb(self, msg):
        try:
            if self._LEFT in msg.name and self._RIGHT in msg.name:
                l = msg.position[msg.name.index(self._LEFT)]
                r = msg.position[msg.name.index(self._RIGHT)]
                self._width = abs(l) + abs(r)
        except ValueError:
            pass

    def _send_position(self, pos):
        cmd = IOComponentCommand(time=rospy.Time.now(), op='set')
        cmd.args = json.dumps({'signals': {
            'position_m': {'format': {'type': 'float'}, 'data': [pos]}}})
        self._pub.publish(cmd)

    def open(self):   self._send_position(self.MAX_POSITION)
    def close(self):  self._send_position(self.MIN_POSITION)

    def set_position(self, pos):
        self._send_position(max(self.MIN_POSITION, min(self.MAX_POSITION, pos)))

    def get_position(self):
        return self._width

    def is_grasping(self):
        return 0.002 < self._width < (self.MAX_POSITION - 0.005) if self._width >= 0 else False

    def get_state(self):
        return {'position': self.get_position(), 'is_grasping': self.is_grasping(),
                'mode': 'sim'}


# ──────────────────────────────────────────────────────────────────────────────
# Real hardware backend  (ClickSmart SmartToolPlate)
# ──────────────────────────────────────────────────────────────────────────────

class _RealBackend:
    """
    ClickSmart SmartToolPlate via SimpleClickSmartGripper.

    Discovers the device serial from /io/end_effector/config, activates it,
    then controls it using EE Signal Type 'grip' (True=close, False=open).
    """

    MAX_POSITION = 0.041667   # kept for API compatibility; maps to open
    MIN_POSITION = 0.0        # maps to close

    def __init__(self, device_id: str):
        rospy.loginfo(f"Gripper(real): initialising ClickSmart device={device_id}")
        self._plate = SimpleClickSmartGripper(device_id, initialize=True)
        rospy.loginfo(f"Gripper(real): ready  endpoints={self._plate.list_endpoint_names()}")

    def _set_grip(self, value: bool):
        for ep in self._plate.list_endpoint_names():
            self._plate.set_ee_signal_value('grip', value, endpoint_id=ep)

    def open(self):   self._set_grip(True)
    def close(self):  self._set_grip(False)

    def set_position(self, pos: float):
        """Binary: pos <= 0.0 → close, pos > 0.0 → open."""
        self._set_grip(pos > 0.0)

    def get_position(self):
        """MAX_POSITION when open, 0.0 when closed, -1.0 if unknown."""
        ep = self._plate.list_endpoint_names()
        if not ep:
            return -1.0
        v = self._plate.get_ee_signal_value('grip', endpoint_id=ep[0])
        if v is None:
            return -1.0
        return self.MAX_POSITION if v else 0.0

    def is_grasping(self):
        """True when grip signals are inactive (closed = gripping)."""
        for ep in self._plate.list_endpoint_names():
            if not self._plate.get_ee_signal_value('grip', endpoint_id=ep):
                return True
        return False

    def get_state(self):
        return {
            'position':   self.get_position(),
            'is_grasping': self.is_grasping(),
            'device':     self._plate.name,
            'endpoints':  self._plate.list_endpoint_names(),
            'signals':    {k: v['data'] for k, v in self._plate.get_all_signals().items()},
            'mode':       'real',
        }


# ──────────────────────────────────────────────────────────────────────────────
# Facade
# ──────────────────────────────────────────────────────────────────────────────

class Gripper:
    """
    Unified gripper interface for real hardware and Gazebo simulation.

    Args:
        mode:      'real' (default) or 'sim'
        device_id: ClickSmart device serial (real mode only).
                   Defaults to 'stp_021709TP00448'.
    """

    def __init__(self, mode: str = 'real', device_id: str = 'stp_021709TP00448'):
        rospy.loginfo(f"Gripper: initialising (mode={mode})")
        if mode == 'sim':
            self._backend = _SimBackend()
        else:
            self._backend = _RealBackend(device_id)
        rospy.sleep(0.5)
        rospy.loginfo("Gripper: ready")

    def open(self) -> bool:
        try:
            self._backend.open()
            rospy.sleep(1.0)
            return True
        except Exception as e:
            rospy.logerr(f"Gripper.open: {e}")
            return False

    def close(self) -> bool:
        try:
            self._backend.close()
            rospy.sleep(1.0)
            return True
        except Exception as e:
            rospy.logerr(f"Gripper.close: {e}")
            return False

    def release(self) -> None:
        """Non-blocking open — used during toss trajectory for zero-latency release."""
        self._backend.open()

    def set_position(self, position: float) -> bool:
        try:
            self._backend.set_position(position)
            rospy.sleep(1.0)
            return True
        except Exception as e:
            rospy.logerr(f"Gripper.set_position: {e}")
            return False

    def is_grasping(self) -> bool:
        return self._backend.is_grasping()

    def get_position(self) -> float:
        return self._backend.get_position()

    def get_state(self) -> dict:
        return self._backend.get_state()


if __name__ == "__main__":
    import sys
    mode = sys.argv[1] if len(sys.argv) > 1 else 'real'
    rospy.init_node("gripper_test")

    gripper = Gripper(mode=mode)
    print(f"\n[init] {gripper.get_state()}")

    print("\n[1] Opening gripper...")
    gripper.open()
    print(f"    position={gripper.get_position():.5f}  grasping={gripper.is_grasping()}")

    print("\n[2] Closing gripper...")
    gripper.close()
    print(f"    position={gripper.get_position():.5f}  grasping={gripper.is_grasping()}")

    print("\n[3] Opening gripper (leave safe)...")
    gripper.open()

    print("\n[PASS] Test complete")
