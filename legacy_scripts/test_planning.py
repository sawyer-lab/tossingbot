#!/usr/bin/env python3
import os
import time
import numpy as np
from sawyer_robot import SawyerRobot
from sawyer_common.geometry import JointAngles
from tossingbot import config as cfg
from tossingbot.planning.kinematics import CasadiKinematics
from tossingbot.planning.casadi_planner import CasadiPlanner

# Path to URDF for planning
URDF_PATH = os.path.join(os.path.dirname(__file__), "..", "planning", "sawyer_electric.urdf")

def main():
    # 1. Setup Robot and Planner
    host = os.environ.get("ROBOT_HOST", "localhost")
    print(f"Connecting to SawyerRobot at {host}...")
    
    # Initialize Planning components
    print(f"Loading URDF from {URDF_PATH}...")
    kinematics = CasadiKinematics(URDF_PATH, "base", "right_gripper_tip")
    planner = CasadiPlanner(kinematics)

    try:
        with SawyerRobot(host=host) as robot:
            print(f"Robot status: {robot.get_robot_status()}")
            
            # Enable robot
            print("Enabling robot...")
            robot.enable()
            time.sleep(1.0)
            
            # 2. Move to Neutral using joint move (baseline)
            neutral_q = cfg.NEUTRAL_JOINT_POS
            print(f"Moving to Neutral joints: {neutral_q}")
            robot.arm.move(JointAngles.from_list(neutral_q))
            time.sleep(1.0)
            
            # 3. Plan Cartesian Move
            # Get current pose from kinematics (or robot)
            q_curr = robot.arm.get_joints().to_list()
            current_pose = robot.arm.get_pose()
            print(f"Current Pose from robot: {current_pose}")
            
            # Define target: 10cm forward (X), 10cm up (Z)
            target_pos = [
                current_pose.position.x + 0.1,
                current_pose.position.y,
                current_pose.position.z + 0.1
            ]
            # Maintain current orientation (pointing down)
            target_quat = [
                current_pose.orientation.x,
                current_pose.orientation.y,
                current_pose.orientation.z,
                current_pose.orientation.w
            ]
            
            print(f"Planning Cartesian move to: {target_pos}")
            start_time = time.time()
            trajectory = planner.plan_cartesian(
                q_curr, 
                target_pos, 
                target_quat, 
                linear_speed=0.15,
                check_floor=False
            )
            
            if trajectory:
                plan_time = time.time() - start_time
                print(f"Planning successful! ({len(trajectory)} steps, {plan_time:.2f}s)")
                
                # 4. Execute Trajectory
                print("Executing trajectory...")
                success = robot.arm.execute_trajectory(trajectory)
                print(f"Execution {'successful' if success else 'failed'}")
            else:
                print("Planning failed.")
            
            time.sleep(1.0)
            
            # 5. Return to Neutral
            print("Returning to Neutral...")
            robot.arm.move(JointAngles.from_list(neutral_q))
            
            print("Test complete.")
            
    except Exception as e:
        print(f"An error occurred: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
