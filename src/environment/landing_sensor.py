#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ContactsState

class LandingSensor:
    def __init__(self, topic="/landing_table_contact"):
        self.sub = rospy.Subscriber(topic, ContactsState, self._cb)
        self.last_contact_pos = None
        self.last_contact_time = None
        self.active = False
        self.detected_object = None

    def _cb(self, msg):
        if not self.active or not msg.states: return
        
        # Get first contact
        state = msg.states[0]
        
        # Filter: Ignore ground plane contacts if necessary, but here we only collide with table top
        # Check if we have contact positions
        if state.contact_positions:
            self.last_contact_pos = state.contact_positions[0] # Point(x,y,z)
            self.last_contact_time = rospy.Time.now()
            
            # Extract object name from collision name
            # collision1_name might be "landing_table::table_top::collision"
            # collision2_name might be "L_shape::link::collision"
            # We want the one that is NOT the table
            c1 = state.collision1_name
            c2 = state.collision2_name
            
            if "landing_table" in c1:
                self.detected_object = c2.split("::")[0]
            else:
                self.detected_object = c1.split("::")[0]

    def start_listening(self):
        """Enable monitoring"""
        self.active = True
        self.last_contact_pos = None
        self.detected_object = None

    def stop_listening(self):
        self.active = False

    def get_landing_result(self, timeout=5.0):
        """
        Wait for landing.
        Returns: (position: Point, object_name: str) or (None, None)
        """
        start = rospy.Time.now()
        while (rospy.Time.now() - start).to_sec() < timeout:
            if self.last_contact_pos:
                self.stop_listening()
                return self.last_contact_pos, self.detected_object
            rospy.sleep(0.01)
        
        self.stop_listening()
        return None, None
