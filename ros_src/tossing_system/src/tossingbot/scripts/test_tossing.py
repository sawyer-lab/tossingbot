#!/usr/bin/env python3.8
import rospy
import numpy as np
import rospkg
import csv
import os
from geometry_msgs.msg import Pose, Point, Quaternion

from tossingbot.hardware.sawyer import SawyerInterface, ControlMode, RobotCommand
from tossingbot.hardware.gripper import GripperInterface
from tossingbot.planning.casadi_planner import CasadiPlanner
from tossingbot.planning.kinematics import CasadiKinematics
from tossingbot.tossing.motion_planner import TossingPlanner
from tossingbot.environment.gazebo_object_manager import GazeboObjectManager
from tossingbot.environment.landing_sensor import LandingSensor
from tossingbot import config as cfg
from scipy.spatial.transform import Rotation as R

# --- CONFIGURATION ---
ROBOT_Z_OFFSET = 1.0
# Pick position (boxes already shifted in world file to align with gripper offset)
PICK_POS_WORLD = [0.60, 0.0, 0.760]  # Centered on work table
TARGET_QUAT = [0, 1, 0, 0]  # Vertical grasp
LOG_FILE = "toss_results.csv"

def to_robot_frame(world_pos):
    p = list(world_pos)
    p[2] -= ROBOT_Z_OFFSET
    return p

def execute_trajectory(robot, plan_data):
    """Standard execution for CasadiPlanner results"""
    if plan_data is None: 
        rospy.logerr("Plan is None!")
        return False
    
    cmd_stream = [
        RobotCommand(
            position=p['position'],
            velocity=p['velocity'],
            acceleration=p['acceleration']
        ) for p in plan_data
    ]
    
    robot.execute_stream(cmd_stream, ControlMode.TRAJECTORY)
    return True

def execute_toss_trajectory(robot, gripper, sol, release_index, split_index):
    """
    Executes the tossing trajectory.
    """
    rospy.loginfo(f"Executing Toss... Release at index {release_index}. Splitting at {split_index}")
    
    Q = sol["Q"]
    Qd = sol["Qd"]
    Qdd = sol["Qdd"]
    N = Q.shape[0] # Shape is (TimeSteps, Joints)
    
    # Create full stream
    stream = []
    for k in range(N):
        cmd = RobotCommand(
            position=Q[k, :].tolist(),
            velocity=Qd[k, :].tolist(),
            acceleration=Qdd[k, :].tolist()
        )
        stream.append(cmd)
        
    robot._command_msg.mode = ControlMode.TRAJECTORY

    # --- EXECUTION: TOSS ONLY ---
    rospy.loginfo("--- EXECUTING TOSS PHASE ---")
    for i in range(split_index):
        if rospy.is_shutdown(): break
        cmd = stream[i]
        
        # Trigger Release
        if i == release_index:
            gripper.open()
            rospy.loginfo("RELEASE!")
            
        robot._command_msg.position = cmd.position
        robot._command_msg.velocity = cmd.velocity
        robot._command_msg.acceleration = cmd.acceleration
        robot._command_msg.header.stamp = rospy.Time.now()
        
        robot._pub_joint_cmd.publish(robot._command_msg)
        robot._rate.sleep()

    rospy.loginfo("Toss Phase Complete.")
    return True

def run_single_toss(target_speed, robot, gripper, pick_planner, manager, sensor):
    rospy.loginfo(f"=== RUNNING TOSS AT SPEED: {target_speed} m/s ===")
    
    # 1. Reset Scene
    rospy.loginfo("Resetting Scene...")
    manager.despawn("toss_cube")
    rospy.sleep(0.5)
    manager.spawn("cube", "toss_cube", Pose(
        position=Point(*PICK_POS_WORLD), 
        orientation=Quaternion(0,0,0,1)
    ))
    rospy.sleep(1.0)
    
    # Coordinates
    pick_target = to_robot_frame(PICK_POS_WORLD)
    
    # 2. PICK SEQUENCE
    rospy.loginfo("Starting Pick Sequence...")
    gripper.open()
    
    # Define Targets
    hover_target = list(pick_target)
    hover_target[2] += 0.05 # 5cm Hover
    
    # A. Move to Hover (Joint Space)
    rospy.loginfo("Moving to Hover (Joint)...")
    q_curr = robot.get_joint_positions()
    q_hover = pick_planner.compute_inverse_kinematics(q_curr, hover_target, TARGET_QUAT)
    
    if q_hover:
        traj = pick_planner.plan_joint(q_curr, q_hover, joint_speed=1.5)
        execute_trajectory(robot, traj)
    else:
        rospy.logerr("Hover IK Failed!")
        return None, None, False

    # B. Approach (Cartesian)
    rospy.loginfo("Approaching (Cartesian)...")
    q_curr = robot.get_joint_positions()
    traj = pick_planner.plan_cartesian(q_curr, pick_target, TARGET_QUAT, linear_speed=0.1)
    execute_trajectory(robot, traj)

    # C. Grasp
    rospy.loginfo("Grasping...")
    rospy.sleep(0.2)
    gripper.close()
    rospy.sleep(0.5)
    
    # Verify Grasp
    if not gripper.is_grasping():
        rospy.logwarn("Grasp Failed! Object not detected in gripper. Aborting trial.")
        gripper.open()
        return None, None, False
    
    # D. Lift (Cartesian)
    rospy.loginfo("Lifting (Cartesian)...")
    q_curr = robot.get_joint_positions()
    traj = pick_planner.plan_cartesian(q_curr, hover_target, TARGET_QUAT, linear_speed=0.1)
    execute_trajectory(robot, traj)
    
    # 3. Move to Fixed Toss-Ready Position (Maintains 3R Planar Constraint)
    rospy.loginfo("Moving to Toss-Ready Position...")

    # Move to fixed configuration with J2=0, J4=0, J6=1.766
    # This ensures the robot is in the planar configuration
    q_curr = robot.get_joint_positions()
    traj = pick_planner.plan_joint(q_curr, cfg.TOSS_READY_POS, joint_speed=1.5)
    execute_trajectory(robot, traj)

    # Note: J0=0 points at workspace center (Y=0.1363)
    # Tossing motion (J1, J3, J5) is independent of target location
    # Only J0 would need to rotate for different target Y positions
    
    # 4. Plan Toss
    rospy.loginfo("Planning Toss...")
    current_q = np.array(robot.get_joint_positions())
    
    # Extract 3-DOF active joints for Planar Planner (J1, J3, J5)
    q0_3dof = np.array([current_q[1], current_q[3], current_q[5]])
    
    # Initialize Tossing Planner with 3-DOF start config
    toss_planner = TossingPlanner(profile="express", angle_deg=45, q0=q0_3dof)
    
    try:
        # Get 3-DOF planar trajectory for the current speed
        sol_3dof = toss_planner.get_trajectory(target_speed)
        
        # Map to 7-DOF using current J0 base angle
        base_angle = current_q[0]
        sol_7dof = toss_planner.map_to_7dof(
            sol_3dof["Q"], sol_3dof["Qd"], sol_3dof["Qdd"], base_angle
        )
        
        # Release exactly 4 steps before peak velocity to account for hardware response time
        release_index = sol_3dof["index"] - 4
        split_index = sol_3dof["index"]
        
        # 5. Execute Toss
        sensor.start_listening()
        execute_toss_trajectory(robot, gripper, sol_7dof, release_index, split_index)
        
        # 6. Check Result
        rospy.loginfo("Waiting for landing...")
        pos, obj = sensor.get_landing_result(timeout=5.0)
        
        if pos:
            rospy.loginfo(f"SUCCESS! Object '{obj}' landed at {pos}")
            return pos.x, pos.y, True
        else:
            rospy.loginfo("Missed? No contact detected on landing table.")
            return None, None, False
            
    except Exception as e:
        rospy.logerr(f"Tossing Failed: {e}")
        return None, None, False

    rospy.sleep(1.0)


def run_tossing_experiment():
    rospy.init_node('test_tossing_script')
    
    # 1. Setup
    robot = SawyerInterface()
    gripper = GripperInterface()
    manager = GazeboObjectManager(wait_for_services=True)
    sensor = LandingSensor()
    
    # 2. Planners
    rp = rospkg.RosPack()
    urdf_path = rp.get_path('grasping') + "/sawyer_model.urdf"
    kinematics = CasadiKinematics(urdf_path, "base", "right_gripper_tip")
    pick_planner = CasadiPlanner(kinematics)
    
    # 3. Speed Loop setup
    test_speeds = [1.0, 1.2, 1.4, 1.6, 1.8, 2.0]
    TRIALS_PER_SPEED = 3
    
    # Init CSV
    with open(LOG_FILE, 'w') as f:
        writer = csv.writer(f)
        writer.writerow(['Speed', 'Trial', 'Landing_X', 'Landing_Y', 'Success'])
        
    rospy.loginfo(f"Starting Experiment. Logging to {LOG_FILE}")
    
    # 0. Ensure Safe Start (Move to Neutral ONCE at start)
    rospy.loginfo("Moving to Neutral/Home...")
    q_curr = robot.get_joint_positions()
    traj = pick_planner.plan_joint(q_curr, cfg.NEUTRAL_JOINT_POS, joint_speed=1.5)
    execute_trajectory(robot, traj)
    
    for speed in test_speeds:
        for i in range(TRIALS_PER_SPEED):
            rospy.loginfo(f"--- TRIAL {i+1}/{TRIALS_PER_SPEED} for SPEED {speed} ---")
            
            lx, ly, success = run_single_toss(speed, robot, gripper, pick_planner, manager, sensor)
            
            # Log Data
            with open(LOG_FILE, 'a') as f:
                writer = csv.writer(f)
                writer.writerow([speed, i+1, lx, ly, success])
            
            # Move Home between trials to reset state
            q_curr = robot.get_joint_positions()
            traj = pick_planner.plan_joint(q_curr, cfg.NEUTRAL_JOINT_POS, joint_speed=1.5)
            execute_trajectory(robot, traj)
            
            rospy.sleep(1.0)

    rospy.loginfo("All Experiments Complete.")
    
if __name__ == "__main__":
    try:
        run_tossing_experiment()
    except rospy.ROSInterruptException:
        pass
