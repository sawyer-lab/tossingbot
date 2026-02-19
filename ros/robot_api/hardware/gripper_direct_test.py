#!/usr/bin/env python3
"""
gripper_direct_test.py

The real Sawyer end_effector_io node publishes:
  /io/end_effector/state          (node-level, device tag: down/ready)
  /io/end_effector/<serial>/config  (once active)
  /io/end_effector/<serial>/state   (once active)

Command goes to:
  /io/end_effector/command        (node-level: activate the device)
  /io/end_effector/<serial>/command  (device-level: calibrate, position, etc.)

The device serial is: stp_021709TP00448

Run inside the intera workspace:
    python gripper_direct_test.py
"""

import sys
import json
import time
import rospy

from std_msgs.msg import Bool
from intera_core_msgs.msg import (
    IOComponentCommand,
    IODeviceStatus,
    IONodeStatus,
    IONodeConfiguration,
)

NODE_CMD_TOPIC   = '/io/end_effector/command'
NODE_STATE_TOPIC = '/io/end_effector/state'
NODE_CFG_TOPIC   = '/io/end_effector/config'

MAX_POSITION = 0.041667
MIN_POSITION = 0.0

_device_name   = None   # discovered from config
_device_tag    = 'unknown'
_signals       = {}

def _node_cfg_cb(msg):
    global _device_name
    for d in msg.devices:
        _device_name = d.name
        rospy.loginfo(f'[config] device: {d.name}')

def _node_state_cb(msg):
    global _device_tag
    for d in msg.devices:
        _device_tag = d.status.tag
        rospy.loginfo_throttle(2.0, f'[state] device={d.name}  tag={d.status.tag}  detail={d.status.detail}')

def _device_state_cb(msg):
    for s in msg.signals:
        try:
            fmt  = json.loads(s.format)
            data = json.loads(s.data)
            _signals[s.name] = {'type': fmt.get('type'), 'value': data[0] if data else None}
        except Exception:
            pass

def sig(name):
    return _signals.get(name, {}).get('value')

def print_signals(label):
    print(f'\n  [{label}]')
    if not _signals:
        print('    (no signals)')
        return
    for k, v in sorted(_signals.items()):
        print(f'    {k:35s}  {v["value"]}')

def node_cmd(pub, op, args_dict):
    pub.publish(IOComponentCommand(
        time=rospy.Time.now(), op=op, args=json.dumps(args_dict)))
    rospy.loginfo(f'NODE CMD: op={op}  args={args_dict}')

def dev_cmd(pub, signal_name, dtype, value):
    args = {'signals': {signal_name: {'format': {'type': dtype}, 'data': [value]}}}
    pub.publish(IOComponentCommand(time=rospy.Time.now(), op='set', args=json.dumps(args)))
    rospy.loginfo(f'DEV CMD: {signal_name} = {value}')

def wait_sig(signal, pred, timeout=10.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        v = sig(signal)
        if v is not None and pred(v):
            return True
        rospy.sleep(0.1)
    return False

def wait_tag(tag_value, timeout=10.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if _device_tag == tag_value:
            return True
        rospy.sleep(0.2)
    return False


def main():
    rospy.init_node('gripper_direct_test', anonymous=True)

    rospy.Subscriber(NODE_STATE_TOPIC, IONodeStatus,       _node_state_cb)
    rospy.Subscriber(NODE_CFG_TOPIC,   IONodeConfiguration, _node_cfg_cb)

    enable_pub  = rospy.Publisher('robot/set_super_enable', Bool, queue_size=10)
    node_pub    = rospy.Publisher(NODE_CMD_TOPIC, IOComponentCommand, queue_size=10)

    rospy.loginfo('Waiting for topics...')
    rospy.sleep(2.0)

    rospy.loginfo('Enabling robot...')
    for _ in range(5):
        enable_pub.publish(Bool(data=True))
        rospy.sleep(0.2)
    rospy.sleep(1.0)

    device = _device_name or 'stp_021709TP00448'
    rospy.loginfo(f'Device: {device}  tag: {_device_tag}')

    dev_state_topic = f'/io/end_effector/{device}/state'
    dev_cmd_topic   = f'/io/end_effector/{device}/command'

    rospy.loginfo(f'Device state topic: {dev_state_topic}')
    rospy.loginfo(f'Device cmd topic  : {dev_cmd_topic}')

    # Subscribe to device state (may not exist yet if tag=down)
    rospy.Subscriber(dev_state_topic, IODeviceStatus, _device_state_cb)
    dev_pub = rospy.Publisher(dev_cmd_topic, IOComponentCommand, queue_size=10)

    # ── Bring device up via node-level configure command ──────────────────────
    if _device_tag != 'ready':
        rospy.loginfo(f'Device is {_device_tag} — sending configure to node cmd topic...')
        # Try 'configure' with the device name — this is what brings it from down→ready
        node_cmd(node_pub, 'configure', {'devices': [{'name': device, 'config': '{}'}]})
        rospy.sleep(2.0)
        rospy.loginfo(f'Tag after configure: {_device_tag}')

        if _device_tag != 'ready':
            # Also try activate
            node_cmd(node_pub, 'activate', {'devices': [device]})
            ok = wait_tag('ready', timeout=8.0)
            rospy.loginfo(f'Tag after activate: {_device_tag}  (ok={ok})')

    rospy.sleep(1.5)
    print_signals('After bring-up')

    # This is a ClickSmart plate — no calibrate/position_m.
    # Signals use EE Signal Types: grip_*, open_*, closed_*, power_*
    # Find all grip signal names
    grip_signals = [k for k in _signals if k.startswith('grip_')]
    open_signals = [k for k in _signals if k.startswith('open_')]
    rospy.loginfo(f'Grip signals : {grip_signals}')
    rospy.loginfo(f'Open signals : {open_signals}')

    if not grip_signals:
        rospy.logerr('No grip signals found!')
        sys.exit(1)

    def grip(value):
        for s in grip_signals:
            dev_cmd(dev_pub, s, 'bool', value)

    # ── Open ──────────────────────────────────────────────────────────────────
    rospy.loginfo('=== Open (grip=False) ===')
    grip(False)
    rospy.sleep(2.0)
    rospy.loginfo(f'open={[sig(s) for s in open_signals]}  closed={[sig(s) for s in [k for k in _signals if k.startswith("closed_")]]}')

    # ── Close ─────────────────────────────────────────────────────────────────
    rospy.loginfo('=== Close (grip=True) ===')
    grip(True)
    rospy.sleep(2.0)
    rospy.loginfo(f'open={[sig(s) for s in open_signals]}  closed={[sig(s) for s in [k for k in _signals if k.startswith("closed_")]]}')

    # ── Open to leave safe ────────────────────────────────────────────────────
    rospy.loginfo('=== Open (leave safe) ===')
    grip(False)
    rospy.sleep(2.0)

    print_signals('Final')
    rospy.loginfo('Done.')


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


import sys
import json
import time
import rospy

from std_msgs.msg import Bool
from intera_core_msgs.msg import IOComponentCommand, IODeviceStatus, IONodeStatus

GRP_CMD_TOPIC    = '/io/end_effector/right_gripper/command'
GRP_STATE_TOPIC  = '/io/end_effector/right_gripper/state'

MAX_POSITION = 0.041667
MIN_POSITION = 0.0

_signals = {}

def _grp_state_cb(msg):
    changed = False
    for s in msg.signals:
        try:
            fmt  = json.loads(s.format)
            data = json.loads(s.data)
            v = data[0] if data else None
            if _signals.get(s.name, {}).get('value') != v:
                changed = True
            _signals[s.name] = {'type': fmt.get('type'), 'value': v}
        except Exception:
            pass
    if changed:
        brief = {k: v['value'] for k, v in _signals.items()
                 if k in ('is_calibrated','is_moving','is_gripping','position_response_m','has_error')}
        rospy.loginfo(f'[gripper state] {brief}')

def sig(name):
    return _signals.get(name, {}).get('value')

def print_signals(label):
    print(f'\n  [{label}]')
    if not _signals:
        print('    (nothing received on state topic)')
        return
    for k, v in sorted(_signals.items()):
        print(f'    {k:35s}  {v["value"]}')

def send(pub, signal_name, dtype, value):
    args = {'signals': {signal_name: {'format': {'type': dtype}, 'data': [value]}}}
    pub.publish(IOComponentCommand(time=rospy.Time.now(), op='set', args=json.dumps(args)))
    rospy.loginfo(f'SENT: {signal_name} = {value}')

def wait_sig(signal, pred, timeout=10.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        v = sig(signal)
        if v is not None and pred(v):
            return True
        rospy.sleep(0.1)
    return False

def main():
    rospy.init_node('gripper_direct_test', anonymous=True)

    rospy.Subscriber(GRP_STATE_TOPIC, IODeviceStatus, _grp_state_cb)
    enable_pub  = rospy.Publisher('robot/set_super_enable', Bool, queue_size=10)
    grp_cmd_pub = rospy.Publisher(GRP_CMD_TOPIC, IOComponentCommand, queue_size=10)

    rospy.loginfo('Enabling robot...')
    rospy.sleep(1.0)
    for _ in range(5):
        enable_pub.publish(Bool(data=True))
        rospy.sleep(0.2)

    rospy.loginfo(f'Subscribing to: {GRP_STATE_TOPIC}')
    rospy.loginfo(f'Publishing to : {GRP_CMD_TOPIC}')
    rospy.sleep(1.0)

    print_signals('Before anything')

    # Try reboot first in case of error state
    rospy.loginfo('=== Sending reboot ===')
    send(grp_cmd_pub, 'reboot', 'bool', True)
    rospy.sleep(3.0)
    print_signals('After reboot')

    rospy.loginfo('=== Sending calibrate ===')
    send(grp_cmd_pub, 'calibrate', 'bool', True)
    ok = wait_sig('is_calibrated', lambda v: v is True, timeout=12.0)
    rospy.loginfo(f'is_calibrated = {sig("is_calibrated")}  (ok={ok})')
    print_signals('After calibrate')

    if not sig('is_calibrated'):
        rospy.logerr('Gripper not responding. Check:')
        rospy.logerr('  rostopic info /io/end_effector/right_gripper/state')
        rospy.logerr('  rostopic info /io/end_effector/right_gripper/command')
        rospy.logerr('  (is the gripper physically connected and the light on?)')
        sys.exit(1)

    rospy.loginfo('=== Open ===')
    send(grp_cmd_pub, 'position_m', 'float', MAX_POSITION)
    wait_sig('is_moving', lambda v: v is False, timeout=5.0)
    rospy.sleep(0.5)
    rospy.loginfo(f'position={sig("position_response_m")}  gripping={sig("is_gripping")}')

    rospy.loginfo('=== Close ===')
    send(grp_cmd_pub, 'position_m', 'float', MIN_POSITION)
    wait_sig('is_moving', lambda v: v is False, timeout=5.0)
    rospy.sleep(0.5)
    rospy.loginfo(f'position={sig("position_response_m")}  gripping={sig("is_gripping")}')

    rospy.loginfo('=== Open (leave safe) ===')
    send(grp_cmd_pub, 'position_m', 'float', MAX_POSITION)
    wait_sig('is_moving', lambda v: v is False, timeout=5.0)

    print_signals('Final')
    rospy.loginfo('Done.')

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


import sys
import json
import time
import rospy

from std_msgs.msg import Bool
from intera_core_msgs.msg import (
    IOComponentCommand,
    IODeviceStatus,
    IONodeStatus,
)

NODE_CMD_TOPIC   = '/io/end_effector/command'
NODE_STATE_TOPIC = '/io/end_effector/state'
GRP_CMD_TOPIC    = '/io/end_effector/right_gripper/command'
GRP_STATE_TOPIC  = '/io/end_effector/right_gripper/state'

MAX_POSITION = 0.041667
MIN_POSITION = 0.0

# ─────────────────────────────────────────────────────────────────────────────

_robot_enabled = None
_node_device_tag = 'unknown'
_signals = {}

def _robot_state_cb(msg):
    global _robot_enabled
    # std_msgs/Bool — published on /robot/state is actually intera_core_msgs
    # but we just use the raw subscriber to print what arrives
    pass

def _node_state_cb(msg):
    global _node_device_tag
    for d in msg.devices:
        _node_device_tag = d.status.tag
        rospy.loginfo(f'[node/state] device={d.name}  tag={d.status.tag}  detail={d.status.detail}')

def _grp_state_cb(msg):
    for s in msg.signals:
        try:
            fmt  = json.loads(s.format)
            data = json.loads(s.data)
            _signals[s.name] = {'type': fmt.get('type'), 'value': data[0] if data else None}
        except Exception:
            pass

def sig(name):
    return _signals.get(name, {}).get('value')

def print_signals(label):
    if not _signals:
        print(f'  [{label}] (no signals received)')
        return
    print(f'\n  [{label}]')
    for k, v in sorted(_signals.items()):
        print(f'    {k:35s}  {v["value"]}  ({v["type"]})')

def send_grp(pub, signal_name, dtype, value):
    args = {'signals': {signal_name: {'format': {'type': dtype}, 'data': [value]}}}
    pub.publish(IOComponentCommand(time=rospy.Time.now(), op='set', args=json.dumps(args)))
    rospy.loginfo(f'  SENT right_gripper/command: {signal_name} = {value}')

def wait_sig(signal, pred, timeout=8.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        v = sig(signal)
        if v is not None and pred(v):
            return True
        rospy.sleep(0.1)
    return False

# ─────────────────────────────────────────────────────────────────────────────

def main():
    rospy.init_node('gripper_direct_test', anonymous=True)

    rospy.Subscriber(NODE_STATE_TOPIC, IONodeStatus,  _node_state_cb)
    rospy.Subscriber(GRP_STATE_TOPIC,  IODeviceStatus, _grp_state_cb)

    enable_pub   = rospy.Publisher('robot/set_super_enable', Bool, queue_size=10)
    grp_cmd_pub  = rospy.Publisher(GRP_CMD_TOPIC, IOComponentCommand, queue_size=10)

    rospy.loginfo('Waiting 2s for topics...')
    rospy.sleep(2.0)

    # ── Step 1: enable robot ─────────────────────────────────────────────────
    rospy.loginfo('=== Step 1: Enabling robot ===')
    for _ in range(5):
        enable_pub.publish(Bool(data=True))
        rospy.sleep(0.3)
    rospy.sleep(1.0)
    rospy.loginfo(f'  gripper device tag after enable: {_node_device_tag}')

    if _node_device_tag == 'down':
        # ── Step 2: try reboot ───────────────────────────────────────────────
        rospy.loginfo('=== Step 2: Sending reboot to right_gripper ===')
        send_grp(grp_cmd_pub, 'reboot', 'bool', True)
        rospy.sleep(3.0)
        rospy.loginfo(f'  tag after reboot: {_node_device_tag}')

    # ── Step 3: check state ──────────────────────────────────────────────────
    rospy.loginfo(f'=== Step 3: Gripper state (tag={_node_device_tag}) ===')
    print_signals('right_gripper/state')

    # ── Step 4: calibrate ────────────────────────────────────────────────────
    rospy.loginfo('=== Step 4: Calibrate ===')
    send_grp(grp_cmd_pub, 'calibrate', 'bool', True)
    ok = wait_sig('is_calibrated', lambda v: v is True, timeout=10.0)
    rospy.loginfo(f'  is_calibrated = {sig("is_calibrated")}  (wait ok={ok})')

    if sig('is_calibrated') is None:
        rospy.logerr('Gripper state topic still empty after calibrate command.')
        rospy.logerr('Possible causes:')
        rospy.logerr('  1. Physical gripper cable disconnected')
        rospy.logerr('  2. Robot arm not fully enabled (press physical enable button)')
        rospy.logerr('  3. Run: rostopic echo /io/end_effector/right_gripper/state')
        rospy.logerr('     If empty: rostopic echo /io/end_effector/state  (check device tag)')
        sys.exit(1)

    # ── Step 5: open / close ─────────────────────────────────────────────────
    rospy.loginfo('=== Step 5: Open ===')
    send_grp(grp_cmd_pub, 'position_m', 'float', MAX_POSITION)
    wait_sig('is_moving', lambda v: v is False, timeout=5.0)
    rospy.sleep(0.5)
    rospy.loginfo(f'  position_response_m={sig("position_response_m")}  is_gripping={sig("is_gripping")}')

    rospy.loginfo('=== Step 6: Close ===')
    send_grp(grp_cmd_pub, 'position_m', 'float', MIN_POSITION)
    wait_sig('is_moving', lambda v: v is False, timeout=5.0)
    rospy.sleep(0.5)
    rospy.loginfo(f'  position_response_m={sig("position_response_m")}  is_gripping={sig("is_gripping")}')

    rospy.loginfo('=== Step 7: Open (leave safe) ===')
    send_grp(grp_cmd_pub, 'position_m', 'float', MAX_POSITION)
    wait_sig('is_moving', lambda v: v is False, timeout=5.0)

    print_signals('Final')
    rospy.loginfo('Done.')


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


import sys
import json
import time
import rospy

from std_msgs.msg import Bool
from intera_core_msgs.msg import (
    IOComponentCommand,
    IODeviceStatus,
    IONodeStatus,
    IONodeConfiguration,
)

NODE_CMD_TOPIC   = 'io/end_effector/command'
NODE_STATE_TOPIC = '/io/end_effector/state'
NODE_CFG_TOPIC   = '/io/end_effector/config'
GRP_CMD_TOPIC    = '/io/end_effector/right_gripper/command'
GRP_STATE_TOPIC  = '/io/end_effector/right_gripper/state'

MAX_POSITION = 0.041667
MIN_POSITION = 0.0

# ── Callbacks that print everything so we can see what's arriving ─────────────

def _node_state_cb(msg):
    rospy.loginfo(f'[node/state] node.name={msg.node.name} node.status.tag={msg.node.status.tag}')
    for d in msg.devices:
        rospy.loginfo(f'  device: name={d.name}  tag={d.status.tag}  detail={d.status.detail}')

def _node_cfg_cb(msg):
    rospy.loginfo(f'[node/config] node.name={msg.node.name}')
    for d in msg.devices:
        rospy.loginfo(f'  device cfg: name={d.name}')

_signals = {}
def _grp_state_cb(msg):
    for s in msg.signals:
        try:
            fmt  = json.loads(s.format)
            data = json.loads(s.data)
            _signals[s.name] = {'type': fmt.get('type'), 'value': data[0] if data else None}
        except Exception:
            pass

def sig(name):
    return _signals.get(name, {}).get('value')

def print_signals(label):
    if not _signals:
        print(f'  [{label}] (no signals)')
        return
    print(f'\n  [{label}]')
    for k, v in sorted(_signals.items()):
        print(f'    {k:35s}  {v["value"]}  ({v["type"]})')

def send(pub, topic_name, signal_name, dtype, value):
    args = {'signals': {signal_name: {'format': {'type': dtype}, 'data': [value]}}}
    pub.publish(IOComponentCommand(time=rospy.Time.now(), op='set', args=json.dumps(args)))
    rospy.loginfo(f'  SENT to {topic_name}: {signal_name} = {value}')

def wait_for(signal, pred, timeout=6.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        v = sig(signal)
        if v is not None and pred(v):
            return True
        rospy.sleep(0.1)
    return False

# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    rospy.init_node('gripper_direct_test', anonymous=True)

    # Subscribe to everything so we can see what's happening
    rospy.Subscriber(NODE_STATE_TOPIC, IONodeStatus,       _node_state_cb)
    rospy.Subscriber(NODE_CFG_TOPIC,   IONodeConfiguration, _node_cfg_cb)
    rospy.Subscriber(GRP_STATE_TOPIC,  IODeviceStatus,     _grp_state_cb)

    # Publishers
    enable_pub   = rospy.Publisher('robot/set_super_enable', Bool, queue_size=10)
    node_cmd_pub = rospy.Publisher(NODE_CMD_TOPIC, IOComponentCommand, queue_size=10)
    grp_cmd_pub  = rospy.Publisher(GRP_CMD_TOPIC,  IOComponentCommand, queue_size=10)

    rospy.loginfo('Waiting 2s for topics to settle...')
    rospy.sleep(2.0)

    # ── Step 1: enable robot ─────────────────────────────────────────────────
    rospy.loginfo('--- Step 1: enabling robot ---')
    for _ in range(5):
        enable_pub.publish(Bool(data=True))
        rospy.sleep(0.2)

    # ── Step 2: activate gripper via node-level topic ────────────────────────
    rospy.loginfo('--- Step 2: sending activate to node cmd topic ---')
    # Try both the device serial AND right_gripper name
    for dev_name in ['stp_021709TP00448', 'right_gripper']:
        args = {'devices': [dev_name]}
        node_cmd_pub.publish(IOComponentCommand(
            time=rospy.Time.now(), op='activate', args=json.dumps(args)))
        rospy.loginfo(f'  activate sent for: {dev_name}')
        rospy.sleep(0.5)

    rospy.sleep(2.0)

    # ── Step 3: check what we have on gripper state topic ────────────────────
    rospy.loginfo('--- Step 3: gripper state ---')
    print_signals('right_gripper/state after activate')

    # ── Step 4: try calibrate + position commands ────────────────────────────
    rospy.loginfo('--- Step 4: calibrate ---')
    send(grp_cmd_pub, 'right_gripper/command', 'calibrate', 'bool', True)
    rospy.sleep(5.0)
    print_signals('after calibrate')
    rospy.loginfo(f'  is_calibrated = {sig("is_calibrated")}')

    rospy.loginfo('--- Step 5: open ---')
    send(grp_cmd_pub, 'right_gripper/command', 'position_m', 'float', MAX_POSITION)
    rospy.sleep(3.0)
    rospy.loginfo(f'  position_response_m = {sig("position_response_m")}')

    rospy.loginfo('--- Step 6: close ---')
    send(grp_cmd_pub, 'right_gripper/command', 'position_m', 'float', MIN_POSITION)
    rospy.sleep(3.0)
    rospy.loginfo(f'  position_response_m = {sig("position_response_m")}')

    rospy.loginfo('--- Step 7: open (leave safe) ---')
    send(grp_cmd_pub, 'right_gripper/command', 'position_m', 'float', MAX_POSITION)
    rospy.sleep(3.0)

    print_signals('Final')
    rospy.loginfo('Done.')


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


import sys
import json
import time
import rospy

from std_msgs.msg import Bool
from intera_core_msgs.msg import IOComponentCommand, IODeviceStatus, IONodeStatus

NODE_CMD_TOPIC  = 'io/end_effector/command'        # node-level: activate/configure
NODE_STATE_TOPIC = 'io/end_effector/state'         # node-level: device status tags
GRP_CMD_TOPIC   = '/io/end_effector/right_gripper/command'
GRP_STATE_TOPIC = '/io/end_effector/right_gripper/state'

GRIPPER_NAME = 'right_gripper'
MAX_POSITION = 0.041667
MIN_POSITION = 0.0


# ── Robot enable (std_msgs Bool, no intera_interface needed) ─────────────────

def ensure_robot_enabled():
    enable_pub = rospy.Publisher('robot/set_super_enable', Bool, queue_size=10)
    rospy.sleep(1.0)
    rospy.loginfo('Enabling robot...')
    for _ in range(5):
        enable_pub.publish(Bool(data=True))
        rospy.sleep(0.2)
    rospy.sleep(0.5)
    rospy.loginfo('Enable command sent')


# ── Gripper activate (node-level command, powers on the gripper) ─────────────

_node_device_tag = {}

def _node_state_cb(msg):
    for dev in msg.devices:
        _node_device_tag[dev.name] = dev.status.tag

def activate_gripper(node_pub):
    rospy.Subscriber(NODE_STATE_TOPIC, IONodeStatus, _node_state_cb)
    rospy.sleep(1.0)

    tag = _node_device_tag.get(GRIPPER_NAME, 'unknown')
    rospy.loginfo(f'Gripper device tag: {tag}')

    if tag in ('down', 'unready', 'unknown'):
        rospy.loginfo('Activating gripper (sending activate to node command topic)...')
        args = {'devices': [GRIPPER_NAME]}
        msg = IOComponentCommand(
            time=rospy.Time.now(),
            op='activate',
            args=json.dumps(args),
        )
        for _ in range(3):
            node_pub.publish(msg)
            rospy.sleep(0.5)

        # Wait for tag to become 'ready'
        deadline = time.time() + 8.0
        while time.time() < deadline:
            tag = _node_device_tag.get(GRIPPER_NAME, 'unknown')
            rospy.loginfo(f'  tag = {tag}')
            if tag == 'ready':
                break
            rospy.sleep(0.5)
        rospy.loginfo(f'After activate: tag = {_node_device_tag.get(GRIPPER_NAME)}')
    else:
        rospy.loginfo(f'Gripper already in state: {tag}')


# ── Gripper command/state ─────────────────────────────────────────────────────

_signals = {}

def _state_cb(msg):
    for sig in msg.signals:
        try:
            fmt  = json.loads(sig.format)
            data = json.loads(sig.data)
            _signals[sig.name] = {
                'type':  fmt.get('type'),
                'value': data[0] if data else None,
            }
        except Exception:
            pass

def sig(name):
    return _signals.get(name, {}).get('value')

def print_state(label):
    print(f'\n  [{label}]')
    for k, v in sorted(_signals.items()):
        print(f'    {k:35s}  {v["value"]}  ({v["type"]})')

def send(pub, signal_name, dtype, value):
    args = {'signals': {signal_name: {'format': {'type': dtype}, 'data': [value]}}}
    pub.publish(IOComponentCommand(time=rospy.Time.now(), op='set', args=json.dumps(args)))
    rospy.loginfo(f'  -> {signal_name} = {value}')

def wait_for(signal, pred, timeout=5.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        v = sig(signal)
        if v is not None and pred(v):
            return True
        rospy.sleep(0.1)
    return False


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    rospy.init_node('gripper_direct_test', anonymous=True)

    ensure_robot_enabled()

    node_pub = rospy.Publisher(NODE_CMD_TOPIC, IOComponentCommand, queue_size=10)
    rospy.sleep(0.5)
    activate_gripper(node_pub)

    grp_pub = rospy.Publisher(GRP_CMD_TOPIC, IOComponentCommand, queue_size=10)
    rospy.Subscriber(GRP_STATE_TOPIC, IODeviceStatus, _state_cb)
    rospy.sleep(1.5)

    if not _signals:
        rospy.logwarn(f'No signals on {GRP_STATE_TOPIC} — gripper may still be initializing')
    else:
        print_state('Initial state')

    print('\n[1] Calibrate')
    send(grp_pub, 'calibrate', 'bool', True)
    ok = wait_for('is_calibrated', lambda v: v is True, timeout=10.0)
    print(f'    is_calibrated = {sig("is_calibrated")}  (ok={ok})')

    print('\n[2] Open')
    send(grp_pub, 'position_m', 'float', MAX_POSITION)
    wait_for('is_moving', lambda v: v is False, timeout=5.0)
    rospy.sleep(0.5)
    print(f'    position_response_m = {sig("position_response_m")}  is_gripping = {sig("is_gripping")}')

    print('\n[3] Close')
    send(grp_pub, 'position_m', 'float', MIN_POSITION)
    wait_for('is_moving', lambda v: v is False, timeout=5.0)
    rospy.sleep(0.5)
    print(f'    position_response_m = {sig("position_response_m")}  is_gripping = {sig("is_gripping")}')

    print('\n[4] Open (leave safe)')
    send(grp_pub, 'position_m', 'float', MAX_POSITION)
    wait_for('is_moving', lambda v: v is False, timeout=5.0)

    if _signals:
        print_state('Final state')
    print('\nDone.')


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


CMD_TOPIC   = '/io/end_effector/right_gripper/command'
STATE_TOPIC = '/io/end_effector/right_gripper/state'

MAX_POSITION = 0.041667
MIN_POSITION = 0.0


# ── Robot enable ─────────────────────────────────────────────────────────────

_robot_state = {}

def _robot_state_cb(msg):
    _robot_state['enabled'] = msg.enabled
    _robot_state['stopped'] = msg.stopped

def ensure_robot_enabled():
    rospy.Subscriber('robot/state', RobotAssemblyState, _robot_state_cb)
    enable_pub = rospy.Publisher('robot/set_super_enable', Bool, queue_size=10)
    rospy.sleep(1.0)

    enabled = _robot_state.get('enabled', False)
    stopped = _robot_state.get('stopped', False)
    rospy.loginfo(f'Robot state: enabled={enabled}  stopped={stopped}')

    if stopped:
        rospy.logerr('Robot is in STOPPED state — press the enable button on the robot first')
        sys.exit(1)

    if not enabled:
        rospy.loginfo('Enabling robot...')
        for _ in range(5):
            enable_pub.publish(True)
            rospy.sleep(0.2)
        rospy.sleep(1.0)
        rospy.loginfo(f'Robot enabled: {_robot_state.get("enabled")}')
    else:
        rospy.loginfo('Robot already enabled')


# ── Gripper helpers ───────────────────────────────────────────────────────────

_signals = {}

def _state_cb(msg):
    for sig in msg.signals:
        try:
            fmt  = json.loads(sig.format)
            data = json.loads(sig.data)
            _signals[sig.name] = {
                'type':  fmt.get('type'),
                'value': data[0] if data else None,
            }
        except Exception:
            pass

def sig(name):
    return _signals.get(name, {}).get('value')

def print_state(label):
    print(f'\n  [{label}]')
    for k, v in sorted(_signals.items()):
        print(f'    {k:35s}  {v["value"]}  ({v["type"]})')

def send(pub, signal_name, dtype, value):
    args = {'signals': {signal_name: {'format': {'type': dtype}, 'data': [value]}}}
    pub.publish(IOComponentCommand(time=rospy.Time.now(), op='set', args=json.dumps(args)))
    rospy.loginfo(f'  -> {signal_name} = {value}')

def wait_for(signal, pred, timeout=5.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        v = sig(signal)
        if v is not None and pred(v):
            return True
        rospy.sleep(0.1)
    return False


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    rospy.init_node('gripper_direct_test', anonymous=True)

    ensure_robot_enabled()

    pub = rospy.Publisher(CMD_TOPIC, IOComponentCommand, queue_size=10)
    rospy.Subscriber(STATE_TOPIC, IODeviceStatus, _state_cb)
    rospy.loginfo(f'Subscribed to : {STATE_TOPIC}')
    rospy.loginfo(f'Publishing to : {CMD_TOPIC}')
    rospy.sleep(1.5)

    if not _signals:
        rospy.logwarn('Still no state signals — check:')
        rospy.logwarn(f'  rostopic hz {STATE_TOPIC}')
        rospy.logwarn(f'  rostopic info {STATE_TOPIC}')
    else:
        print_state('Initial state')

    print('\n[1] Calibrate')
    send(pub, 'calibrate', 'bool', True)
    ok = wait_for('is_calibrated', lambda v: v is True, timeout=8.0)
    print(f'    is_calibrated = {sig("is_calibrated")}  (ok={ok})')

    print('\n[2] Open')
    send(pub, 'position_m', 'float', MAX_POSITION)
    wait_for('is_moving', lambda v: v is False, timeout=4.0)
    rospy.sleep(0.5)
    print(f'    position_response_m = {sig("position_response_m")}  is_gripping = {sig("is_gripping")}')

    print('\n[3] Close')
    send(pub, 'position_m', 'float', MIN_POSITION)
    wait_for('is_moving', lambda v: v is False, timeout=4.0)
    rospy.sleep(0.5)
    print(f'    position_response_m = {sig("position_response_m")}  is_gripping = {sig("is_gripping")}')

    print('\n[4] Open (leave safe)')
    send(pub, 'position_m', 'float', MAX_POSITION)
    wait_for('is_moving', lambda v: v is False, timeout=4.0)

    if _signals:
        print_state('Final state')
    print('\nDone.')


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


import sys
import json
import time
import argparse
import rospy

from intera_core_msgs.msg import (
    IOComponentCommand,
    IONodeConfiguration,
    IONodeStatus,
    IODeviceStatus,
)

# ── Node-level topics (discover actual device name) ──────────────────────────
NODE_CONFIG_TOPIC = '/io/end_effector/config'
NODE_STATE_TOPIC  = '/io/end_effector/state'

MAX_POSITION = 0.041667
MIN_POSITION = 0.0


# ════════════════════════════════════════════════════════════════════════════
# SDK MODE — use intera_interface directly (same as cuff_control example)
# ════════════════════════════════════════════════════════════════════════════

def run_sdk_mode():
    import intera_interface
    from intera_interface import CHECK_VERSION

    rospy.loginfo('SDK mode: initialising RobotEnable...')
    rs = intera_interface.RobotEnable(CHECK_VERSION)
    rospy.loginfo(f'Robot enabled: {rs.state().enabled}')

    rospy.loginfo('SDK mode: creating Gripper("right_gripper")...')
    try:
        gripper = intera_interface.Gripper('right_gripper')
    except Exception as e:
        rospy.logerr(f'Could not create gripper: {e}')
        sys.exit(1)

    rospy.loginfo(f'Gripper name        : {gripper.name}')
    rospy.loginfo(f'is_calibrated       : {gripper.is_calibrated()}')
    rospy.loginfo(f'has_error           : {gripper.has_error()}')

    if gripper.has_error():
        rospy.logwarn('Error detected — rebooting gripper...')
        gripper.reboot()
        rospy.sleep(2.0)

    if not gripper.is_calibrated():
        rospy.loginfo('Calibrating...')
        gripper.calibrate()

    rospy.loginfo(f'After calibration   : is_calibrated={gripper.is_calibrated()}')
    rospy.loginfo(f'position            : {gripper.get_position()}')

    print('\n[1] Open')
    gripper.open()
    rospy.sleep(1.5)
    print(f'    position={gripper.get_position():.5f}  is_gripping={gripper.is_gripping()}')

    print('\n[2] Close')
    gripper.close()
    rospy.sleep(1.5)
    print(f'    position={gripper.get_position():.5f}  is_gripping={gripper.is_gripping()}')

    print('\n[3] Half open')
    gripper.set_position(MAX_POSITION / 2)
    rospy.sleep(1.5)
    print(f'    position={gripper.get_position():.5f}  is_gripping={gripper.is_gripping()}')

    print('\n[4] Open (leave safe)')
    gripper.open()
    rospy.sleep(1.5)

    print('\nDone.')


# ════════════════════════════════════════════════════════════════════════════
# RAW MODE — discover device name, then pub/sub directly
# ════════════════════════════════════════════════════════════════════════════

def discover_device_name(timeout=5.0):
    """Read /io/end_effector/config to find the real device name."""
    result = {}

    def cb(msg):
        if msg.devices:
            result['name'] = msg.devices[0].name

    rospy.Subscriber(NODE_CONFIG_TOPIC, IONodeConfiguration, cb)
    deadline = time.time() + timeout
    while 'name' not in result and time.time() < deadline:
        rospy.sleep(0.1)
    return result.get('name')


def run_raw_mode():
    rospy.loginfo('Raw mode: discovering device name from /io/end_effector/config ...')
    device = discover_device_name()
    if not device:
        rospy.logerr('Could not read device name — is the gripper attached?')
        sys.exit(1)

    rospy.loginfo(f'Found device: {device}')
    cmd_topic   = f'/io/end_effector/{device}/command'
    state_topic = f'/io/end_effector/{device}/state'
    rospy.loginfo(f'Command topic : {cmd_topic}')
    rospy.loginfo(f'State topic   : {state_topic}')

    pub = rospy.Publisher(cmd_topic, IOComponentCommand, queue_size=10)

    _signals = {}

    def state_cb(msg):
        for sig in msg.signals:
            try:
                fmt  = json.loads(sig.format)
                data = json.loads(sig.data)
                _signals[sig.name] = {
                    'type':  fmt.get('type'),
                    'value': data[0] if data else None,
                }
            except Exception:
                pass

    rospy.Subscriber(state_topic, IODeviceStatus, state_cb)
    rospy.sleep(1.5)

    def sig(name):
        return _signals.get(name, {}).get('value')

    def print_state(label):
        print(f'\n  [{label}]')
        for k, v in sorted(_signals.items()):
            print(f'    {k:35s}  {v["value"]}  ({v["type"]})')

    def send(signal_name, dtype, value):
        args = {'signals': {signal_name: {'format': {'type': dtype}, 'data': [value]}}}
        pub.publish(IOComponentCommand(time=rospy.Time.now(), op='set', args=json.dumps(args)))

    def wait_for(signal, pred, timeout=5.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if pred(sig(signal)):
                return True
            rospy.sleep(0.1)
        return False

    if not _signals:
        rospy.logwarn('No state signals received — gripper may be unpowered')
    else:
        print_state('Initial state')

    print('\n[1] Calibrate')
    send('calibrate', 'bool', True)
    ok = wait_for('is_calibrated', lambda v: v is True, timeout=8.0)
    rospy.loginfo(f'calibrated: {ok}  is_calibrated={sig("is_calibrated")}')

    print('\n[2] Open')
    send('position_m', 'float', MAX_POSITION)
    wait_for('is_moving', lambda v: v is False, timeout=4.0)
    rospy.sleep(0.3)
    print_state('After open')

    print('\n[3] Close')
    send('position_m', 'float', MIN_POSITION)
    wait_for('is_moving', lambda v: v is False, timeout=4.0)
    rospy.sleep(0.3)
    print_state('After close')

    print('\n[4] Open (leave safe)')
    send('position_m', 'float', MAX_POSITION)
    wait_for('is_moving', lambda v: v is False, timeout=4.0)

    print('\nDone.')


# ════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--raw', action='store_true', help='Use raw topic mode instead of SDK')
    args, _ = parser.parse_known_args()

    rospy.init_node('gripper_direct_test', anonymous=True)

    if args.raw:
        run_raw_mode()
    else:
        run_sdk_mode()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
