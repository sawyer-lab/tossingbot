#!/usr/bin/env python3
import os
import time
from sawyer_robot import SawyerRobot
from sawyer_common.geometry import JointAngles
from tossingbot import config as cfg

def main():
    # Connect to the robot (assumes the container/server is running)
    host = os.environ.get("ROBOT_HOST", "localhost")
    print(f"Connecting to SawyerRobot at {host}...")
    
    try:
        with SawyerRobot(host=host) as robot:
            status = robot.get_robot_status()
            print(f"Robot status: {status}")
            
            # 1. Enable robot
            print("Enabling robot...")
            robot.enable()
            time.sleep(1.0)
            
            # 2. Get current joints
            curr_joints = robot.arm.get_joints()
            print(f"Current joints: {curr_joints}")
            
            # 3. Move to NEUTRAL_JOINT_POS
            neutral = JointAngles.from_list(cfg.NEUTRAL_JOINT_POS)
            print(f"Moving to Neutral: {neutral}")
            robot.arm.move(neutral)
            
            time.sleep(1.0)
            
            # 4. Move to TOSS_READY_POS
            toss_ready = JointAngles.from_list(cfg.TOSS_READY_POS)
            print(f"Moving to Toss Ready: {toss_ready}")
            robot.arm.move(toss_ready)
            
            time.sleep(1.0)
            
            # 5. Open/Close Gripper
            print("Opening gripper...")
            robot.gripper.open()
            time.sleep(1.0)
            print("Closing gripper...")
            robot.gripper.close()
            time.sleep(1.0)
            
            # 6. Back to Neutral
            print("Returning to Neutral...")
            robot.arm.move(neutral)
            
            print("Test complete.")
            
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    main()
