#!/usr/bin/env python3.8
import sys
import rospy
import numpy as np
import rospkg
import time

# Ensure imports work
sys.path.insert(0, '/geometry2_ws/devel/lib/python3/dist-packages')
sys.path.insert(0, '/cv_bridge_ws/devel/lib/python3/dist-packages')

from tossingbot.hardware.sawyer import SawyerInterface, RobotCommand, ControlMode
from tossingbot.planning.kinematics import CasadiKinematics
from tossingbot.planning.casadi_planner import CasadiPlanner
from tossingbot.planning.orientation_helper import RotationPrimitive

def to_stream(plan_data):
    stream = []
    for p in plan_data:
        stream.append(RobotCommand(
            position=p['position'],
            velocity=p['velocity'],
            acceleration=p['acceleration']
        ))
    return stream

def test_orientations():
    rospy.init_node("test_orientations")
    print("--- TESTING 4-WAY GRIPPER ORIENTATION ---")
    
    # 1. Init System
    robot = SawyerInterface()
    
    rp = rospkg.RosPack()
    urdf = rp.get_path('grasping') + "/sawyer_model.urdf"
    model = CasadiKinematics(urdf, "base", "right_gripper_tip")
    planner = CasadiPlanner(model)
    
    # 4 Rotations: [0, 45, 90, 135]
    rot_helper = RotationPrimitive(num_rotations=4) 
    
    # 2. Define a Safe Center Point (High up to avoid table collision)
    hover_pos = [0.65, 0.0, 0.30] 
    
    print(f"Target Position: {hover_pos} (High Hover)")
    
    # 3. Setup: Move to Angle 0 first to start clean
    print("\n[SETUP] Moving to Angle 0...")
    q_curr = robot.get_joint_positions()
    quat0 = rot_helper.get_quaternion(0)
    target_quat = [quat0.x, quat0.y, quat0.z, quat0.w]
    
    path = planner.plan_cartesian(q_curr, hover_pos, target_quat, duration=3.0, check_floor=False)
    if path:
        robot.execute_stream(to_stream(path), ControlMode.TRAJECTORY)
    else:
        print("[FAIL] Could not reach start position.")
        return

    # 4. Cycle through all 4 angles
    for i in range(4):
        angle = rot_helper.get_angle(i)
        quat_msg = rot_helper.get_quaternion(i)
        
        # Convert msg to list [x, y, z, w]
        target_quat = [quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w]
        
        print(f"\n--> Index {i}: {angle:.1f} degrees")
        print(f"    Quat: [{target_quat[0]:.3f}, {target_quat[1]:.3f}, {target_quat[2]:.3f}, {target_quat[3]:.3f}]")
        
        q_curr = robot.get_joint_positions()
        
        # Plan Motion
        path = planner.plan_cartesian(
            q_start=q_curr, 
            target_pos=hover_pos, 
            target_quat=target_quat, 
            duration=2.0,       # 2 seconds per rotation
            speed_ratio=0.5,    # Medium speed
            check_floor=False   # Optimization: Don't check floor if we are at Z=0.3
        )
        
        if path:
            robot.execute_stream(to_stream(path), ControlMode.TRAJECTORY)
            rospy.sleep(1.0) # Pause to let you see the alignment
        else:
            print(f"[FAIL] Planning failed for angle {angle}")

    # 5. Return to Neutral
    print("\nTest Complete. Returning to Neutral...")
    # Matches your gazebo_env neutral
    neutral_joints = [0.0, -1.27, 0.0, 2.06, 0.0, 0.0, 0.0] 
    
    q_curr = robot.get_joint_positions()
    path = planner.plan_joint(q_curr, neutral_joints, duration=2.0, check_floor=False)
    if path:
        robot.execute_stream(to_stream(path), ControlMode.TRAJECTORY)

if __name__ == "__main__":
    test_orientations()