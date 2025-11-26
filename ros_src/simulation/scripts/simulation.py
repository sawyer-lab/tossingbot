#!/usr/bin/env python

import sys
import rospy

from geometry_msgs.msg import Point, Twist, Pose

import intera_interface
import numpy as np

import random
from GraspingActionClient import GraspingActionClient
from TossingActionClient import TossingActionClient
from LandingDetectionActionClient import LandingDetectionClient




def main():

    rospy.init_node("ik_pick_and_place_demo")
    
    gripper = intera_interface.Gripper()
    limb = intera_interface.Limb("right")
    
    home = {'right_j0': 0.0,
            'right_j1': -1.15,
            'right_j2': 0.0,
            'right_j3': 1.7,
            'right_j4': 0.0,
            'right_j5': 1.5,
            'right_j6': 0.0}


    tossing_client = TossingActionClient()

    
    limb.move_to_joint_positions(home)
    rospy.sleep(1.0)

    desired_speed = 1.5
    
    while not rospy.is_shutdown():

        limb.move_to_joint_positions(home)

       
        tossed =  tossing_client.toss(desired_speed)
        if tossed:
            rospy.loginfo("Toss at speed {:.2f} m/s succeeded.".format(desired_speed))
        else:
            rospy.logwarn("Toss at speed {:.2f} m/s failed.".format(desired_speed))

        desired_speed += 0.1
        if desired_speed > 2.1:
            rospy.loginfo("Completed all tosses.")
            break

    
  

    

if __name__ == '__main__':
    sys.exit(main())