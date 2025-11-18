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
    
    # home = {'right_j0': -0.041662954890248294,
    #                          'right_j1': -1.0258291091425074,
    #                          'right_j2': 0.0293680414401436,
    #                          'right_j3': 2.17518162913313,
    #                          'right_j4':  -0.06703022873354225,
    #                          'right_j5': 0.3968371433926965,
    #                          'right_j6': 1.7659649178699421}

    home = {'right_j0': 0.0,
            'right_j1': -1.15,
            'right_j2': 0.0,
            'right_j3': 1.7,
            'right_j4': 0.0,
            'right_j5': 1.5,
            'right_j6': 0.0}


    # grasping_client = GraspingActionClient()
    tossing_client = TossingActionClient()
    # landing_client = LandingClient()

    # block_position=Point(x=0.443, y=0.145, z=-0.140)
    
    limb.move_to_joint_positions(home)
    rospy.sleep(1.0)
    
    # picked = grasping_client.pick(block_position, 3)

    
    # rospy.sleep(1.0)
    
    # current_angles = limb.joint_angles()
    # print("Current Joint Angles: {}".format(current_angles))
    # limb.set_joint_position_speed(0.001)
    # current_angles['right_j6'] = 1.7659902159840577
    # limb.move_to_joint_positions(current_angles)
    # current_angles['right_j5'] = 0.0
    # limb.move_to_joint_positions(current_angles)
    # current_angles['right_j4'] = 0.0
    # limb.move_to_joint_positions(current_angles)
    # current_angles['right_j3'] = np.pi/4
    # limb.move_to_joint_positions(current_angles)
    
    # limb.set_joint_position_speed(0.001)
    # angles = [0.0, 0.42, 0.0, 0.0, 0.0, 0.0, 1.7659902159840577]
    # limb.move_to_joint_positions(dict(zip(limb.joint_names(), angles)))
    # rospy.sleep(1.0)
    # gripper.open()
    
    limb.move_to_joint_positions(home)
    # rospy.sleep(1.0)
    # block_position=Point(x=0.590, y=-0.12, z=-0.140)
    # picked = grasping_client.pick(block_position, 3)
    # angles = [ 0.0, -1.01986851, 0.0, 2.5424116, 0.0, 0.57185201, 1.76599022]
    # limb.move_to_joint_positions(dict(zip(limb.joint_names(), angles)))
    
    # rospy.sleep(1.0)
    
    desired_speed = 0.05 # m/s
    desired_angle = np.deg2rad(45)  # radians
    desired_velocity = np.array([np.cos(desired_angle), np.sin(desired_angle), 0.0]) * desired_speed
    # landing_client.start()
    tossed =  tossing_client.toss(Point(np.cos(0),np.sin(0),0), Twist(linear=Point(*desired_velocity)))
    # rospy.loginfo("Tossing {}".format("succeeded." if tossed else "failed."))
    # rospy.sleep(5.0)
    
    # landing_idx = landing_client.finish()
    # rospy.logwarn("Landing Index: {}".format(landing_idx))
    # from gazebo_msgs.srv import SpawnModel, DeleteModel, GetModelState, SetModelState
    
    # get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    # set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    
    
    # while True:
    #     cube_state = get_model_state('block2', 'base')
    #     z_pos = cube_state.pose.position.z
        
    #     if z_pos < 0.1:
    #         # remove all velocity
    #         print("resetting cube velocity")
    #         cube_state.twist.linear.x = 0
    #         cube_state.twist.linear.y = 0
    #         cube_state.twist.linear.z = 0
    #         cube_state.twist.angular.x = 0
    #         cube_state.twist.angular.y = 0
    #         cube_state.twist.angular.z = 0
            
    #         # Apply the updated state back to the simulation
    #         set_model_state(cube_state)
    #         break
    
    
    
    # while not rospy.is_shutdown(): 
    #     # block_position = objects_client.get_block_position("block")
    #     block_position=Point(x=0.45, y=0.155, z=-0.135)
    #     # idx = random.randint(0, 15)

    #     picked = grasping_client.pick(block_position, 3)
        
    #     rospy.sleep(1.0)
        

        

        # angles = [ 0.0, -1.01986851, 0.0, 2.5424116, 0.0, 0.57185201, 1.76599022]
        # limb.move_to_joint_positions(dict(zip(limb.joint_names(), angles)))

        # rospy.sleep(1.0)
  
        
        # rospy.loginfo("Grasping {}".format("succeeded." if picked else "failed."))
        # if not picked:
        #     rospy.logwarn("Grasping failed, retrying...")
        #     continue
        
        # rospy.sleep(1.0)

        # desired_speed = 2.7  # m/s
        # desired_angle = np.deg2rad(45)  # radians
        # desired_velocity = np.array([np.cos(desired_angle), np.sin(desired_angle), 0.0]) * desired_speed

    
        # tossed =  tossing_client.toss(Point(np.cos(0),np.sin(0),0), Twist(linear=Point(*desired_velocity)))
        # rospy.loginfo("Tossing {}".format("succeeded." if tossed else "failed."))
        
        # rospy.sleep(1.0)
        
        # limb.move_to_joint_positions(starting_joint_angles)
    
        # rospy.sleep(1.0)

    

    

if __name__ == '__main__':
    sys.exit(main())