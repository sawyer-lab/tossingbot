#!/usr/bin/env python3
"""
Contact Sensor Manager

Manages collision/contact sensor data from Gazebo.
Subscribes to contact state topics and tracks landing events.
"""

import rospy
import threading
from gazebo_msgs.msg import ContactsState


class ContactSensorManager:
    """
    Manages contact sensor data from Gazebo collision topics.
    
    Subscribes to ContactsState messages and tracks landing positions
    and object names. Provides thread-safe access to landing data.
    """
    
    def __init__(self, topic="/landing_table_contact"):
        """
        Initialize contact sensor manager.
        
        Args:
            topic: ROS topic name for contact states (default: /landing_table_contact)
        """
        self._lock = threading.Lock()
        self.topic = topic
        self.active = False
        
        # Last detected landing
        self.last_position = None
        self.last_object_name = None
        self.last_timestamp = None
        
        # Contact history (optional, for future use)
        self.contact_history = []
        self.max_history = 100
        
        # Subscribe to contact topic
        self.subscriber = rospy.Subscriber(topic, ContactsState, self._contact_callback)
        
        rospy.loginfo(f"ContactSensorManager initialized on topic: {topic}")
    
    def _contact_callback(self, msg):
        """
        ROS callback for contact state messages.
        
        Args:
            msg: ContactsState message from Gazebo
        """
        if not self.active or not msg.states:
            return
        
        # Get first contact state
        state = msg.states[0]
        
        # Extract contact position
        if not state.contact_positions:
            return
        
        contact_pos = state.contact_positions[0]
        
        # Extract object name from collision names
        # collision1_name might be "landing_table::table_top::collision"
        # collision2_name might be "cube_0::link::collision"
        c1 = state.collision1_name
        c2 = state.collision2_name
        
        if "landing_table" in c1:
            obj_name = c2.split("::")[0]
        else:
            obj_name = c1.split("::")[0]
        
        # Store landing data
        with self._lock:
            self.last_position = [contact_pos.x, contact_pos.y, contact_pos.z]
            self.last_object_name = obj_name
            self.last_timestamp = rospy.Time.now().to_sec()
            
            # Add to history
            self.contact_history.append({
                'position': self.last_position.copy(),
                'object_name': obj_name,
                'timestamp': self.last_timestamp
            })
            
            # Trim history
            if len(self.contact_history) > self.max_history:
                self.contact_history.pop(0)
        
        rospy.logdebug(f"Contact detected: {obj_name} at {self.last_position}")
    
    def enable(self):
        """Enable contact detection."""
        with self._lock:
            self.active = True
            rospy.loginfo("Contact sensor enabled")
    
    def disable(self):
        """Disable contact detection."""
        with self._lock:
            self.active = False
            rospy.loginfo("Contact sensor disabled")
    
    def clear(self):
        """Clear all landing data and history."""
        with self._lock:
            self.last_position = None
            self.last_object_name = None
            self.last_timestamp = None
            self.contact_history.clear()
            rospy.logdebug("Contact sensor data cleared")
    
    def get_last_landing(self):
        """
        Get the last detected landing event.
        
        Returns:
            dict: Landing data with keys:
                - position: [x, y, z] list of floats
                - object_name: string
                - timestamp: float (seconds since epoch)
            None if no landing detected
        """
        with self._lock:
            if self.last_position is None:
                return None
            
            return {
                'position': self.last_position.copy(),
                'object_name': self.last_object_name,
                'timestamp': self.last_timestamp
            }
    
    def get_history(self, count=None):
        """
        Get contact history.
        
        Args:
            count: Number of recent contacts to return (default: all)
        
        Returns:
            list: List of landing dicts (most recent last)
        """
        with self._lock:
            if count is None:
                return [h.copy() for h in self.contact_history]
            else:
                return [h.copy() for h in self.contact_history[-count:]]
    
    def is_active(self):
        """
        Check if contact detection is active.
        
        Returns:
            bool: True if active, False otherwise
        """
        with self._lock:
            return self.active
