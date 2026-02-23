#!/usr/bin/env python3.8
import sys
import rospy
import numpy as np
import rospkg
import time
from scipy.spatial.transform import Rotation as R

# Ensure imports work
sys.path.insert(0, '/geometry2_ws/devel/lib/python3/dist-packages')
sys.path.insert(0, '/cv_bridge_ws/devel/lib/python3/dist-packages')

from tossingbot.hardware.sawyer import SawyerInterface, RobotCommand, ControlMode
from tossingbot.planning.kinematics import CasadiKinematics
from tossingbot.planning.casadi_planner import CasadiPlanner
from tossingbot.planning.orientation_helper import RotationPrimitive

def quaternion_error(q_target, q_actual):
    """
    Calculate orientation error between two quaternions in degrees.
    Args:
        q_target: [x, y, z, w]
        q_actual: [x, y, z, w]
    Returns:
        Angular error in degrees
    """
    r_target = R.from_quat(q_target)
    r_actual = R.from_quat(q_actual)
    r_error = r_target.inv() * r_actual
    angle_rad = r_error.magnitude()
    return np.degrees(angle_rad)

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
    print("="*70)
    print("TESTING GRIPPER ORIENTATION ACCURACY")
    print("="*70)
    
    # 1. Init System
    robot = SawyerInterface()
    
    rp = rospkg.RosPack()
    urdf = rp.get_path('grasping') + "/sawyer_model.urdf"
    model = CasadiKinematics(urdf, "base", "right_gripper_tip")
    planner = CasadiPlanner(model)
    
    # 4 Rotations: [0, 45, 90, 135]
    rot_helper = RotationPrimitive(num_rotations=4) 
    
    # 2. Test multiple positions to find pattern
    test_positions = [
        [0.65, 0.0, 0.30],   # Center high
        [0.70, 0.2, 0.30],   # Right
        [0.70, -0.2, 0.30],  # Left
        [0.60, 0.0, 0.25],   # Center low
    ]
    
    results = []
    
    for pos_idx, hover_pos in enumerate(test_positions):
        print(f"\n{'='*70}")
        print(f"TEST POSITION {pos_idx+1}/4: {hover_pos}")
        print(f"{'='*70}")
        
        for rot_idx in range(4):
            angle = rot_helper.get_angle(rot_idx)
            quat_msg = rot_helper.get_quaternion(rot_idx)
            
            # Convert msg to list [x, y, z, w]
            target_quat = [quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w]
            
            print(f"\n--> Rotation {rot_idx}: {angle:.1f}°")
            print(f"    Target Quat: [{target_quat[0]:.4f}, {target_quat[1]:.4f}, {target_quat[2]:.4f}, {target_quat[3]:.4f}]")
            
            q_curr = robot.get_joint_positions()
            
            # Plan Motion
            path = planner.plan_cartesian(
                q_start=q_curr, 
                target_pos=hover_pos, 
                target_quat=target_quat, 
                duration=2.0,
                speed_ratio=0.5,
                check_floor=False
            )
            
            if not path:
                print(f"    [FAIL] Planning failed")
                continue
            
            # Execute
            robot.execute_stream(to_stream(path), ControlMode.TRAJECTORY)
            rospy.sleep(0.5)  # Let it settle
            
            # Read actual endpoint pose
            endpoint = robot.get_endpoint_pose()
            if endpoint is None:
                print(f"    [FAIL] Could not read endpoint pose")
                continue
            
            actual_pos = endpoint['position']
            actual_quat = endpoint['orientation']
            
            # Calculate errors
            pos_error = np.linalg.norm(np.array(actual_pos) - np.array(hover_pos))
            ori_error = quaternion_error(target_quat, actual_quat)
            
            print(f"    Actual Quat:  [{actual_quat[0]:.4f}, {actual_quat[1]:.4f}, {actual_quat[2]:.4f}, {actual_quat[3]:.4f}]")
            print(f"    Position Error: {pos_error*1000:.2f} mm")
            print(f"    Orientation Error: {ori_error:.2f}°")
            
            if ori_error > 5.0:
                print(f"    ⚠️  WARNING: Large orientation error!")
            
            results.append({
                'position': hover_pos,
                'rot_idx': rot_idx,
                'angle': angle,
                'pos_error': pos_error,
                'ori_error': ori_error
            })
            
            rospy.sleep(0.5)
    
    # Summary
    print(f"\n{'='*70}")
    print("SUMMARY")
    print(f"{'='*70}")
    print(f"{'Pos':<15} {'Rot':>5} {'Angle':>7} {'Pos Err(mm)':>12} {'Ori Err(°)':>11}")
    print("-"*70)
    
    for r in results:
        pos_str = f"[{r['position'][0]:.2f},{r['position'][1]:.2f},{r['position'][2]:.2f}]"
        print(f"{pos_str:<15} {r['rot_idx']:>5} {r['angle']:>7.1f} {r['pos_error']*1000:>12.2f} {r['ori_error']:>11.2f}")
    
    # Statistics
    ori_errors = [r['ori_error'] for r in results]
    print(f"\nOrientation Error Statistics:")
    print(f"  Mean: {np.mean(ori_errors):.2f}°")
    print(f"  Max:  {np.max(ori_errors):.2f}°")
    print(f"  Min:  {np.min(ori_errors):.2f}°")
    print(f"  Std:  {np.std(ori_errors):.2f}°")
    
    # Check if specific rotations are problematic
    for rot_idx in range(4):
        rot_errors = [r['ori_error'] for r in results if r['rot_idx'] == rot_idx]
        angle = rot_helper.get_angle(rot_idx)
        print(f"  Rot {rot_idx} ({angle:.0f}°): avg={np.mean(rot_errors):.2f}°, max={np.max(rot_errors):.2f}°")
    
    # 5. Return to Neutral
    print(f"\n{'='*70}")
    print("Test Complete. Returning to Neutral...")
    neutral_joints = [0.0, -1.27, 0.0, 2.06, 0.0, 0.0, 0.0] 
    
    q_curr = robot.get_joint_positions()
    path = planner.plan_joint(q_curr, neutral_joints, duration=2.0, check_floor=False)
    if path:
        robot.execute_stream(to_stream(path), ControlMode.TRAJECTORY)

if __name__ == "__main__":
    test_orientations()