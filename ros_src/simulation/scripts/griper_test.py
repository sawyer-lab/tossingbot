#!/usr/bin/env python


from intera_interface import Gripper
from rospy import init_node, is_shutdown, loginfo, sleep
import sys

def main():
    init_node("gripper_test")
    gripper = Gripper()
    rate = 1.0
    while not is_shutdown():
        loginfo("Opening gripper...")
        gripper.open()
        sleep(rate)
        loginfo("Closing gripper...")
        gripper.close()
        sleep(rate)
        loginfo("Setting gripper to 50% position...")
        gripper.set_position(0.5)
        sleep(rate)
        
if __name__ == "__main__":
    sys.exit(main())