#!/usr/bin/env python3
import rospy
from intera_core_msgs.msg import HeadPanCommand, HeadState


class Head:
    def __init__(self):
        self._state = {}
        self._pub_pan = rospy.Publisher('/robot/head/command_head_pan', HeadPanCommand, queue_size=10)
        self._sub_state = rospy.Subscriber('/robot/head/head_state', HeadState, self._on_head_state)
        rospy.sleep(0.5)
        rospy.loginfo("Head: Initialized")

    def _on_head_state(self, msg):
        self._state['pan'] = msg.pan
        self._state['panning'] = msg.isTurning
        self._state['blocked'] = msg.isBlocked
        self._state['pan_mode'] = msg.panMode

    def pan(self):
        return self._state.get('pan', 0.0)

    def panning(self):
        return self._state.get('panning', False)

    def blocked(self):
        return self._state.get('blocked', False)

    def set_pan(self, angle, speed=1.0):
        speed = max(HeadPanCommand.MIN_SPEED_RATIO, min(speed, HeadPanCommand.MAX_SPEED_RATIO))
        msg = HeadPanCommand(angle, speed, HeadPanCommand.SET_ACTIVE_MODE)
        self._pub_pan.publish(msg)
        return True
