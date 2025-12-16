#!/usr/bin/env python3.8
import rospy
import json
from intera_core_msgs.msg import IOComponentCommand

class GripperInterface:
    """
    A simplified interface for the Right Gripper on Intera robots.
    Handles the JSON serialization required by the IOComponentCommand.
    """
    
    # Constants from Rethink documentation
    MAX_POSITION = 0.041667 # Open (Meters)
    MIN_POSITION = 0.0      # Closed (Meters)

    def __init__(self):
        self._topic = '/io/end_effector/right_gripper/command'
        self._pub = rospy.Publisher(self._topic, IOComponentCommand, queue_size=1)
        
        # Pre-allocate message skeleton
        self._cmd = IOComponentCommand()
        self._cmd.op = 'set'
        
        rospy.loginfo("GripperInterface: Ready.")

    def open(self):
        """Commands the gripper to fully open."""
        self._send_position(self.MAX_POSITION)

    def close(self):
        """Commands the gripper to fully close."""
        self._send_position(self.MIN_POSITION)

    def set_position(self, position: float):
        """Sets a specific width (0.0 to 0.041667)."""
        # Clamp value for safety
        pos = max(self.MIN_POSITION, min(self.MAX_POSITION, position))
        self._send_position(pos)

    def _send_position(self, pos_value):
        """
        Internal helper to format the Intera IO message.
        Structure matches intera_io/scripts/io_interface.py logic.
        """
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
        # TODO: Implement feedback check if sensors are available
        return False
    
    def get_current_position(self) -> float:
        # TODO: Implement feedback retrieval if sensors are available
        return -1.0  # Unknown


if __name__ == "__main__":
    rospy.init_node("test_gripper_hardware")
    
    print("--- GRIPPER INTERFACE TESTER ---")
    gripper = GripperInterface()
    
    # Allow time for connection
    rospy.sleep(1.0)

    try:
        # 1. Test Open
        print("Commanding: OPEN")
        gripper.open()
        rospy.sleep(2.0)

        # 2. Test Close
        print("Commanding: CLOSE")
        gripper.close()
        rospy.sleep(2.0)

        # 3. Test Specific Position (Halfway)
        mid_point = gripper.MAX_POSITION / 2.0
        print(f"Commanding: POSITION {mid_point:.4f}m")
        gripper.set_position(mid_point)
        rospy.sleep(2.0)

        # 4. Rapid Cycle Test
        print("Running Rapid Cycle (Open/Close) 3 times...")
        for i in range(3):
            gripper.open()
            rospy.sleep(0.5)
            gripper.close()
            rospy.sleep(0.5)
            
        print("[PASS] Gripper Test Complete. Resetting to Open.")
        gripper.open()

    except rospy.ROSInterruptException:
        pass