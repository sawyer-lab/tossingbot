#!/usr/bin/env python3
"""Base class for ROS camera topic subscribers."""

import threading
import rospy


class BaseCameraSubscriber:
    """
    Shared lifecycle and threading pattern for camera subscribers.

    Provides: lock, active flag, enable/disable/is_active/has_data/timestamp.
    Subclasses implement: ROS subscriptions, callbacks, and data getters.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._active = True
        self._latest_timestamp = None

    def enable(self):
        """Resume processing incoming messages."""
        with self._lock:
            self._active = True
        rospy.loginfo(f"{self.__class__.__name__}: enabled")

    def disable(self):
        """Pause processing to save CPU (callbacks fire but are ignored)."""
        with self._lock:
            self._active = False
        rospy.loginfo(f"{self.__class__.__name__}: disabled")

    def is_active(self):
        """Return True if processing is enabled."""
        with self._lock:
            return self._active

    def has_data(self):
        """Return True if at least one message has been received."""
        raise NotImplementedError

    def get_timestamp(self):
        """Return timestamp (seconds) of the last received message, or None."""
        with self._lock:
            return self._latest_timestamp
