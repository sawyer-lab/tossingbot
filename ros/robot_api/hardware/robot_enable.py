#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool, Empty
from intera_core_msgs.msg import AssemblyState


class RobotEnable:
    """
    Simple control/status wrapper around robot state

    enable()  - enable all joints
    disable() - disable all joints
    reset()   - reset all joints, clear faults
    stop()    - emergency stop (like e-stop button)
    """

    def __init__(self):
        self._state = None
        self._state_sub = rospy.Subscriber(
            'robot/state',
            AssemblyState,
            self._state_callback
        )

        # Wait for state
        timeout = rospy.Time.now() + rospy.Duration(5.0)
        while self._state is None and rospy.Time.now() < timeout:
            rospy.sleep(0.1)

        if self._state is None:
            rospy.logwarn("RobotEnable: Failed to get robot state")
        else:
            rospy.loginfo("RobotEnable: Initialized")

    def _state_callback(self, msg):
        self._state = msg

    def state(self):
        """Get current robot state."""
        return self._state

    def is_enabled(self):
        """Check if robot is enabled."""
        return self._state.enabled if self._state else False

    def is_stopped(self):
        """Check if robot is stopped (e-stop or error)."""
        return self._state.stopped if self._state else False

    def is_error(self):
        """Check if robot has error."""
        return self._state.error if self._state else False

    def enable(self):
        """Enable all joints."""
        if self._state and self._state.stopped:
            rospy.loginfo("Robot stopped, attempting reset...")
            self.reset()

        pub = rospy.Publisher('robot/set_super_enable', Bool, queue_size=10)
        rospy.sleep(0.1)

        for _ in range(10):
            pub.publish(True)
            rospy.sleep(0.5)
            if self._state and self._state.enabled:
                rospy.loginfo("Robot enabled")
                return True

        rospy.logwarn("Failed to enable robot")
        return False

    def disable(self):
        """Disable all joints."""
        pub = rospy.Publisher('robot/set_super_enable', Bool, queue_size=10)
        rospy.sleep(0.1)

        for _ in range(10):
            pub.publish(False)
            rospy.sleep(0.5)
            if self._state and not self._state.enabled:
                rospy.loginfo("Robot disabled")
                return True

        rospy.logwarn("Failed to disable robot")
        return False

    def reset(self):
        """Reset robot, clear faults."""
        if self._state and not self._state.stopped:
            rospy.logwarn("Robot not in error state, cannot reset")
            return False

        if self._state and self._state.estop_button == AssemblyState.ESTOP_BUTTON_PRESSED:
            rospy.logerr("E-Stop is pressed, disengage before reset")
            return False

        pub = rospy.Publisher('robot/set_super_reset', Empty, queue_size=10)
        rospy.sleep(0.1)
        rospy.loginfo("Resetting robot...")

        for _ in range(10):
            pub.publish(Empty())
            rospy.sleep(0.5)
            if self._state and not self._state.stopped and not self._state.error:
                rospy.loginfo("Robot reset successful")
                return True

        rospy.logwarn("Failed to reset robot")
        return False

    def stop(self):
        """Emergency stop (simulates e-stop button press)."""
        pub = rospy.Publisher('robot/set_super_stop', Empty, queue_size=10)
        rospy.sleep(0.1)

        for _ in range(10):
            pub.publish(Empty())
            rospy.sleep(0.5)
            if self._state and self._state.stopped:
                rospy.loginfo("Robot stopped")
                return True

        rospy.logwarn("Failed to stop robot")
        return False
