#!/usr/bin/env python3.8
import rospy
import json
import sensor_msgs.msg
from intera_core_msgs.msg import IOComponentCommand

class GripperInterface:
    """
    Controls the gripper via IOComponentCommand (Control)
    but reads feedback via JointStates (Feedback), which is robust in Simulation.
    """
    
    # Constants
    MAX_POSITION = 0.041667 # Open (Meters)
    MIN_POSITION = 0.0      # Closed (Meters)
    
    # Joint names in Gazebo/URDF
    LEFT_FINGER  = 'right_gripper_l_finger_joint'
    RIGHT_FINGER = 'right_gripper_r_finger_joint'

    def __init__(self):
        # 1. Publisher for Commands (Keep this, it works for control)
        self._cmd_topic = '/io/end_effector/right_gripper/command'
        self._pub = rospy.Publisher(self._cmd_topic, IOComponentCommand, queue_size=1)
        
        # 2. Subscriber for Feedback (SWITCHED TO JOINT STATES)
        self._sub = rospy.Subscriber('/robot/joint_states', sensor_msgs.msg.JointState, self._joint_callback)
        
        # Internal State
        self._current_width = -1.0
        
        # Pre-allocate command message
        self._cmd = IOComponentCommand()
        self._cmd.op = 'set'
        
        rospy.loginfo("GripperInterface: Ready (JointState Mode).")

    def _joint_callback(self, msg):
        """
        Reads the global joint state message to find gripper fingers.
        """
        try:
            # Check if we have both fingers in this message
            if self.LEFT_FINGER in msg.name and self.RIGHT_FINGER in msg.name:
                idx_l = msg.name.index(self.LEFT_FINGER)
                idx_r = msg.name.index(self.RIGHT_FINGER)
                
                # In Sawyer, fingers move prismatically (meters).
                # Total Width = Left Position + Right Position
                # (Sometimes one is negative depending on URDF, but usually both are + in Gazebo)
                pos_l = msg.position[idx_l]
                pos_r = msg.position[idx_r]
                
                # Use abs() just in case the joint definition is inverted
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
        """
        Returns True if the gripper is 'stuck' holding an object.
        """
        width = self.get_current_position()
        if width < 0: return False # Unknown state
        
        # Logic: We commanded CLOSED (0.0), but the gripper stopped early (> 1mm)
        # AND it is not fully open (< 3cm)
        if width > 0.002 and width < (self.MAX_POSITION - 0.005):
            return True
        return False
    
    def get_current_position(self) -> float:
        return self._current_width

if __name__ == "__main__":
    rospy.init_node("test_gripper_hardware")
    
    print("--- GRIPPER INTERFACE TESTER (JOINT STATE VERSION) ---")
    gripper = GripperInterface()
    
    # Wait for first callback
    rospy.sleep(1.0)

    try:
        # 1. Test Open
        print(f"\n[1] Commanding: OPEN")
        gripper.open()
        rospy.sleep(2.0)
        print(f"    Current Position: {gripper.get_current_position():.5f} m")

        # 2. Test Close
        print(f"\n[2] Commanding: CLOSE")
        gripper.close()
        rospy.sleep(2.0)
        print(f"    Current Position: {gripper.get_current_position():.5f} m")

        # 3. Test Halfway
        mid = gripper.MAX_POSITION / 2.0
        print(f"\n[3] Commanding: POSITION {mid:.4f} m")
        gripper.set_position(mid)
        rospy.sleep(2.0)
        print(f"    Current Position: {gripper.get_current_position():.5f} m")
            
        print("\n[PASS] Test Complete.")
        gripper.open()

    except rospy.ROSInterruptException:
        pass