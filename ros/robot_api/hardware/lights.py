#!/usr/bin/env python3
import rospy
import json
from intera_core_msgs.msg import IOComponentCommand, IONodeStatus


class Lights:
    def __init__(self):
        self.cmd_pub = rospy.Publisher('/io/robot/command', IOComponentCommand, queue_size=10)
        self.status_sub = rospy.Subscriber('/io/robot/status', IONodeStatus, self._status_callback)
        self.lights_state = {}
        rospy.sleep(0.5)
        rospy.loginfo("Lights: Initialized")

    def list_all_lights(self):
        return [name for name in self.lights_state.keys() if 'light' in name]

    def set_light_state(self, name, on=True):
        cmd = IOComponentCommand()
        cmd.time = rospy.Time.now()
        cmd.op = 'set'
        args = {
            'signals': {
                name: {
                    'format': {'type': 'bool'},
                    'data': [on]
                }
            }
        }
        cmd.args = json.dumps(args)
        self.cmd_pub.publish(cmd)
        return True

    def get_light_state(self, name):
        return self.lights_state.get(name, False)

    def _status_callback(self, msg):
        for signal in msg.signals:
            if 'light' in signal.name:
                self.lights_state[signal.name] = bool(signal.data[0]) if signal.data else False
