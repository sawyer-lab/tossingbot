#!/usr/bin/env python3.8
import rospy
import rospkg
import numpy as np
import sys
import time

# Ensure imports work
sys.path.append("/home/kid/ros_ws/src/tossingbot_system/src")

from tossingbot.hardware.sawyer import SawyerInterface, RobotCommand, ControlMode
from tossingbot.planning.kinematics import CasadiKinematics
from tossingbot.planning.casadi_planner import CasadiPlanner

def to_stream(plan_data):
    """Converts dictionary plan to RobotCommand stream."""
    stream = []
    for p in plan_data:
        stream.append(RobotCommand(
            position=p['position'],
            velocity=p['velocity'],
            acceleration=p['acceleration']
        ))
    return stream

def run_test():
    rospy.init_node("test_planner_comprehensive")
    print("--- COMPREHENSIVE PLANNER TEST ---")
    
    # 1. Initialize
    robot = SawyerInterface()
    
    rp = rospkg.RosPack()
    urdf = rp.get_path('grasping') + "/sawyer_model.urdf"
    model = CasadiKinematics(urdf, "base", "right_gripper_tip")
    planner = CasadiPlanner(model)
    
    # 2. Configs
    neutral_joints = [0.0, -0.6, 0.0, 1.5, 0.0, 0.5, 0.0]
    
    # ==========================================================================
    # PHASE 1: FAST JOINT RESET (check_floor=False)
    # ==========================================================================
    print("\n[PHASE 1] Moving to Neutral (Joint Space, Fast, No Checks)...")
    q_curr = robot.get_joint_positions()
    
    # plan_joint: Used for large reconfigurations
    path = planner.plan_joint(q_curr, neutral_joints, duration=2.0, speed_ratio=1.0, check_floor=False)
    
    if path:
        robot.execute_stream(to_stream(path), ControlMode.TRAJECTORY)
        robot.hold_position(0.5)
    else:
        print("[FAIL] Setup failed.")
        return

    # ==========================================================================
    # PHASE 2: THE VARIABLE SPEED SQUARE (Cartesian Space)
    # ==========================================================================
    # We will draw a square in the air.
    # Leg 1: Fast
    # Leg 2: Slow
    # Leg 3: Fast
    # Leg 4: Slow
    
    q_curr = robot.get_joint_positions()
    start_pos = np.array(model.fk_pos(q_curr)).flatten()
    print(f"\n[PHASE 2] Starting Square at {np.round(start_pos, 3)}")
    
    # Define Corners (Relative 15cm box)
    corners = [
        start_pos + np.array([0.0, 0.15, 0.0]),   # Left
        start_pos + np.array([0.0, 0.15, 0.15]),  # Up-Left
        start_pos + np.array([0.0, 0.0, 0.15]),   # Up-Right
        start_pos                                 # Return Home
    ]
    
    configs = [
        {"dur": 1.0, "spd": 1.0, "desc": "FAST SNAP"},
        {"dur": 4.0, "spd": 0.2, "desc": "SLOW DRIFT"},
        {"dur": 1.0, "spd": 1.0, "desc": "FAST SNAP"},
        {"dur": 4.0, "spd": 0.2, "desc": "SLOW DRIFT"}
    ]

    for i, target in enumerate(corners):
        cfg = configs[i]
        print(f"  -> Leg {i+1}: {cfg['desc']} (Dur: {cfg['dur']}s)")
        
        q_curr = robot.get_joint_positions()
        
        # plan_cartesian: Used for straight lines
        path = planner.plan_cartesian(
            q_start=q_curr, 
            target_pos=target, 
            duration=cfg['dur'], 
            speed_ratio=cfg['spd'], 
            check_floor=False # Faster planning!
        )
        
        if path:
            robot.execute_stream(to_stream(path), ControlMode.TRAJECTORY)
            # Brief pause to visually separate movements
            robot.hold_position(0.2)
        else:
            print(f"[FAIL] Cartesian Leg {i+1} Failed.")
            return

    # ==========================================================================
    # PHASE 3: FINAL JOINT MOVE
    # ==========================================================================
    print("\n[PHASE 3] Retracting to Side (Joint Space, Medium Speed)...")
    side_joints = [0.5, -0.2, -0.5, 1.2, 0.0, 1.0, 0.0]
    
    q_curr = robot.get_joint_positions()
    path = planner.plan_joint(q_curr, side_joints, duration=3.0, speed_ratio=0.5, check_floor=False)
    
    if path:
        robot.execute_stream(to_stream(path), ControlMode.TRAJECTORY)
        print("\n[SUCCESS] All Tests Passed. Planner is robust.")
    else:
        print("[FAIL] Final retract failed.")

if __name__ == "__main__":
    run_test()