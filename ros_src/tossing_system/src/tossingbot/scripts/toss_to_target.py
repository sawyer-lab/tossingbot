#!/usr/bin/env python3.8
import rospy
import numpy as np
import rospkg
import json
import os
import time
from geometry_msgs.msg import Pose, Point, Quaternion
from tossingbot.hardware.sawyer import SawyerInterface, ControlMode, RobotCommand
from tossingbot.hardware.gripper import GripperInterface
from tossingbot.planning.casadi_planner import CasadiPlanner
from tossingbot.planning.kinematics import CasadiKinematics
from tossingbot.tossing.motion_planner import TossingPlanner
from tossingbot.environment.gazebo_object_manager import GazeboObjectManager
from tossingbot.environment.landing_sensor import LandingSensor
from tossingbot import config as cfg
from tossingbot.alignment import TargetAlignment

# --- CALIBRATION CONSTANTS (Refined from 8cm undershoot) ---
G = 9.806
TABLE_Z = 0.75
RELEASE_X_EST = 0.68  # Adjusted from 0.71 to 0.68
RELEASE_Z_EST = 1.11  
TOSS_ANGLE_RAD = np.deg2rad(45)

# Mapping: Act_Speed = Cmd_Speed + BIAS
# To fix 8cm undershoot, we set bias to -0.10 (making robot throw harder)
SPEED_BIAS = -0.10  

# --- CONFIGURATION ---
NUM_TARGETS = 5
X_RANGE = [1.0, 1.3]
Y_OFFSET_RANGE = [-0.15, 0.15]
PICK_POS_WORLD = [0.60, cfg.CENTER_Y, 0.760]

def calculate_required_speed(target_x):
    """
    Inverts the ballistic equation to find required release speed.
    """
    dx = target_x - RELEASE_X_EST
    dz = RELEASE_Z_EST - TABLE_Z
    tan = np.tan(TOSS_ANGLE_RAD)
    cos = np.cos(TOSS_ANGLE_RAD)
    
    # v^2 = (g * dx^2) / (2 * cos^2 * (dx * tan + z0 - zt))
    v_sq = (G * dx**2) / (2 * cos**2 * (dx * tan + dz))
    v_actual = np.sqrt(v_sq)
    
    # Map back to commanded speed
    # S_act = S_cmd + 0.81 => S_cmd = S_act - 0.81
    v_cmd = v_actual - SPEED_BIAS
    return np.clip(v_cmd, 0.5, 2.5), v_actual

def to_robot_frame(world_pos):
    p = list(world_pos)
    p[2] -= 1.0
    return p

def execute_trajectory(robot, plan_data):
    if not plan_data: return False
    cmd_stream = [RobotCommand(p['position'], p['velocity'], p['acceleration']) for p in plan_data]
    robot.execute_stream(cmd_stream, ControlMode.TRAJECTORY)
    return True

def run_inverse_tossing():
    rospy.init_node('toss_to_target_inverse')
    rospy.loginfo("--- STARTING INVERSE TOSSING EXPERIMENT ---")
    
    robot = SawyerInterface()
    gripper = GripperInterface()
    manager = GazeboObjectManager(wait_for_services=True)
    sensor = LandingSensor()
    
    rp = rospkg.RosPack()
    urdf_path = rp.get_path('grasping') + "/sawyer_model.urdf"
    pick_planner = CasadiPlanner(CasadiKinematics(urdf_path, "base", "right_gripper_tip"))

    results = []

    for i in range(NUM_TARGETS):
        # 1. Select Random Target
        tx = np.random.uniform(X_RANGE[0], X_RANGE[1])
        ty = cfg.CENTER_Y + np.random.uniform(Y_OFFSET_RANGE[0], Y_OFFSET_RANGE[1])
        
        # 2. Infer Parameters
        cmd_speed, exp_act_speed = calculate_required_speed(tx)
        j0_angle = TargetAlignment.get_base_rotation(tx, ty, cfg.CENTER_Y)
        
        rospy.loginfo(f"\n>>> TARGET {i+1}: ({tx:.3f}, {ty:.3f})")
        rospy.loginfo(f"Inferred: Speed={cmd_speed:.3f} (Exp Act={exp_act_speed:.3f}), J0={np.rad2deg(j0_angle):.2f}deg")

        # 3. Reset & Pick
        manager.despawn("toss_cube")
        manager.spawn("cube", "toss_cube", Pose(Point(*PICK_POS_WORLD), Quaternion(0,0,0,1)))
        rospy.sleep(0.5)
        
        # Move to Neutral
        execute_trajectory(robot, pick_planner.plan_joint(robot.get_joint_positions(), cfg.NEUTRAL_JOINT_POS, joint_speed=1.5))
        
        # Pick Sequence (Standard)
        gripper.open()
        pick_r = to_robot_frame(PICK_POS_WORLD)
        hover_r = [pick_r[0], pick_r[1], pick_r[2] + 0.20]
        
        q_hover = pick_planner.compute_inverse_kinematics(robot.get_joint_positions(), hover_r, [0,1,0,0])
        if not q_hover:
            # Fallback
            traj = pick_planner.plan_cartesian(robot.get_joint_positions(), hover_r, [0,1,0,0], linear_speed=0.2)
        else:
            traj = pick_planner.plan_joint(robot.get_joint_positions(), q_hover)
        execute_trajectory(robot, traj)
        
        execute_trajectory(robot, pick_planner.plan_cartesian(robot.get_joint_positions(), pick_r, [0,1,0,0], linear_speed=0.1))
        gripper.close()
        rospy.sleep(0.5)
        execute_trajectory(robot, pick_planner.plan_cartesian(robot.get_joint_positions(), hover_r, [0,1,0,0], linear_speed=0.2))

        # 4. Align & Toss
        q_ready = list(cfg.TOSS_READY_POS)
        q_ready[0] = j0_angle
        execute_trajectory(robot, pick_planner.plan_joint(robot.get_joint_positions(), q_ready, joint_speed=1.5))
        
        q_curr = np.array(robot.get_joint_positions())
        tp = TossingPlanner(profile="express", angle_deg=45, q0=q_curr[1:6:2]) # J1, J3, J5
        sol_3d = tp.get_trajectory(cmd_speed)
        sol_7d = tp.map_to_7dof(sol_3d["Q"], sol_3d["Qd"], sol_3d["Qdd"], q_curr[0], q_curr[2], q_curr[4], q_curr[6])
        
        sensor.start_listening()
        
        # Execute Toss (Manual Stream for release logic)
        Q, release_idx = sol_7d["Q"], sol_3d["index"] - 4
        robot._command_msg.mode = ControlMode.TRAJECTORY
        for k in range(Q.shape[0]):
            if k == release_idx: gripper.open()
            
            # Explicitly cast to list to avoid ROS serialization issues with numpy types
            robot._command_msg.position = list(sol_7d["Q"][k])
            robot._command_msg.velocity = list(sol_7d["Qd"][k])
            robot._command_msg.acceleration = list(sol_7d["Qdd"][k])
            
            robot._command_msg.header.stamp = rospy.Time.now()
            robot._pub_joint_cmd.publish(robot._command_msg)
            robot._rate.sleep()

        # 5. Check Result
        land_pos, _ = sensor.get_landing_result(timeout=5.0)
        if land_pos:
            err_x = land_pos.x - tx
            err_y = land_pos.y - ty
            dist_err = np.sqrt(err_x**2 + err_y**2)
            rospy.loginfo(f"LANDED at ({land_pos.x:.3f}, {land_pos.y:.3f}). Error: {dist_err*1000:.1f}mm")
            results.append(dist_err)
        else:
            rospy.logwarn("MISSED TABLE!")

    if results:
        rospy.loginfo(f"AVERAGE TARGETING ERROR: {np.mean(results)*1000:.1f}mm")

if __name__ == "__main__":
    try:
        run_inverse_tossing()
    except rospy.ROSInterruptException:
        pass
