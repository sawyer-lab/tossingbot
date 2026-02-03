#!/usr/bin/env python3.8
"""
Test script for aligned tossing with base rotation.
Tests tosses to targets at different Y positions using gripper alignment.
"""
import rospy
import numpy as np
import rospkg
import csv
import os
from geometry_msgs.msg import Pose, Point, Quaternion
from scipy.spatial.transform import Rotation as R

from tossingbot.hardware.sawyer import SawyerInterface, ControlMode, RobotCommand
from tossingbot.hardware.gripper import GripperInterface
from tossingbot.planning.casadi_planner import CasadiPlanner
from tossingbot.planning.kinematics import CasadiKinematics
from tossingbot.tossing.motion_planner import TossingPlanner
from tossingbot.environment.gazebo_object_manager import GazeboObjectManager
from tossingbot.environment.landing_sensor import LandingSensor
from tossingbot import config as cfg

class TargetAlignment:
    """Helper class for calculating robot base alignment."""
    
    @staticmethod
    def get_base_rotation(target_x, target_y, offset_y):
        """
        Calculates the required base rotation (J0) to aim a laterally offset arm at a target.
        Exact solution: theta = atan2(y, x) - arcsin(d / R)
        
        Args:
            target_x (float): Target X in robot frame.
            target_y (float): Target Y in robot frame.
            offset_y (float): Lateral offset of the tossing arm (positive = left shift).
            
        Returns:
            float: Required J0 angle in radians.
        """
        dist_to_target = np.sqrt(target_x**2 + target_y**2)

        # Safety check: Target must be outside the offset circle
        if dist_to_target < abs(offset_y):
            rospy.logwarn(f"Target distance ({dist_to_target:.3f}) < Offset ({offset_y:.3f}). Using approx.")
            return np.arctan2(target_y - offset_y, target_x)

        return np.arctan2(target_y, target_x) - np.arcsin(offset_y / dist_to_target)

# --- CONFIGURATION ---
ROBOT_Z_OFFSET = 1.0
# Pick position on work table (boxes already shifted in world file)
PICK_POS_WORLD = [0.60, cfg.CENTER_Y, 0.760]  # Centered on work table
LOG_FILE = "toss_results_aligned.csv"

# Test configuration: diverse Y positions
TEST_TARGETS = [
    {"name": "Center",      "world_pos": [1.10, cfg.CENTER_Y, 0.760],        "speed": 1.5},
    {"name": "Left",        "world_pos": [1.10, cfg.CENTER_Y + 0.15, 0.760], "speed": 1.5},
    {"name": "Right",       "world_pos": [1.10, cfg.CENTER_Y - 0.15, 0.760], "speed": 1.5},
    {"name": "Far_Left",    "world_pos": [1.10, cfg.CENTER_Y + 0.25, 0.760], "speed": 1.6},
    {"name": "Far_Right",   "world_pos": [1.10, cfg.CENTER_Y - 0.25, 0.760], "speed": 1.6},
]

def to_robot_frame(world_pos):
    """Convert world position to robot frame."""
    p = list(world_pos)
    p[2] -= ROBOT_Z_OFFSET
    return p

def execute_trajectory(robot, plan_data):
    """Standard execution for CasadiPlanner results."""
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
    Executes the tossing trajectory with gripper release.
    """
    rospy.loginfo(f"Executing Toss... Release at index {release_index}. Splitting at {split_index}")

    Q = sol["Q"]
    Qd = sol["Qd"]
    Qdd = sol["Qdd"]
    N = Q.shape[0]  # Shape is (TimeSteps, Joints)

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

    # Execute toss phase
    rospy.loginfo("--- EXECUTING TOSS PHASE ---")
    for i in range(split_index):
        if rospy.is_shutdown():
            break
        cmd = stream[i]

        # Trigger release
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

def run_single_toss(target_config, robot, gripper, pick_planner, manager, sensor):
    """
    Run a single toss to a specified target using base alignment.

    Args:
        target_config: Dict with 'name', 'world_pos', 'speed'
    """
    target_name = target_config["name"]
    target_world = target_config["world_pos"]
    target_speed = target_config["speed"]

    rospy.loginfo("\n" + "=" * 60)
    rospy.loginfo(f"TOSSING TO TARGET: {target_name}")
    rospy.loginfo(f"World Position: {target_world}")
    rospy.loginfo(f"Speed: {target_speed} m/s")
    rospy.loginfo("=" * 60)

    # Convert to robot frame
    target_robot = to_robot_frame(target_world)

    # 1. Reset Scene
    rospy.loginfo("Resetting Scene...")
    manager.despawn("toss_cube")
    manager.despawn("target_marker")
    rospy.sleep(0.5)

    # Spawn object to toss
    manager.spawn("cube", "toss_cube", Pose(
        position=Point(*PICK_POS_WORLD),
        orientation=Quaternion(0, 0, 0, 1)
    ))
    rospy.sleep(0.3)

    # Spawn target marker
    manager.spawn("sphere", "target_marker", Pose(
        position=Point(*target_world),
        orientation=Quaternion(0, 0, 0, 1)
    ))
    rospy.sleep(1.0)

    # 2. PICK SEQUENCE
    rospy.loginfo("Starting Pick Sequence...")
    gripper.open()

    pick_target = to_robot_frame(PICK_POS_WORLD)
    hover_target = list(pick_target)
    hover_target[2] += 0.05  # 5cm hover

    # A. Move to Hover (Joint Space)
    rospy.loginfo("Moving to Hover (Joint)...")
    q_curr = robot.get_joint_positions()
    TARGET_QUAT = [0, 1, 0, 0]  # Vertical grasp
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

    # D. Lift
    rospy.loginfo("Lifting (Cartesian)...")
    q_curr = robot.get_joint_positions()
    traj = pick_planner.plan_cartesian(q_curr, hover_target, TARGET_QUAT, linear_speed=0.1)
    execute_trajectory(robot, traj)

    # 3. Move to Fixed Toss-Ready Position
    rospy.loginfo("Moving to Toss-Ready Position...")

    # Move to fixed configuration with J2=0, J4=0, J6=1.766
    q_curr = robot.get_joint_positions()
    traj = pick_planner.plan_joint(q_curr, cfg.TOSS_READY_POS, joint_speed=1.5)
    execute_trajectory(robot, traj)

    # 4. Rotate J0 to Align with Target
    # Calculate J0 angle using exact kinematic solution
    j0_aligned = TargetAlignment.get_base_rotation(
        target_robot[0], 
        target_robot[1], 
        cfg.CENTER_Y
    )
        
    rospy.loginfo(f"Rotating J0 to {np.rad2deg(j0_aligned):.2f}° for target at Y={target_world[1]:.3f} (Exact Sol.)")

    # Create configuration with only J0 changed (keeps J1,J3,J5,J2,J4,J6 same)
    q_aligned = list(cfg.TOSS_READY_POS)
    q_aligned[0] = j0_aligned

    q_curr = robot.get_joint_positions()
    traj = pick_planner.plan_joint(q_curr, q_aligned, joint_speed=1.0)
    execute_trajectory(robot, traj)

    rospy.loginfo("Robot aligned. J1,J3,J5 tossing motion will be identical for all targets.")

    # 4. Plan Toss Trajectory
    rospy.loginfo("Planning Toss Trajectory...")
    current_q = np.array(robot.get_joint_positions())

    # Extract 3-DOF active joints (J1, J3, J5)
    q0_3dof = np.array([current_q[1], current_q[3], current_q[5]])

    # Initialize Tossing Planner
    # NOTE: We do NOT pass target position - the planar motion is always the same!
    # We only rotated J0 to point the plane at the target
    toss_planner = TossingPlanner(
        profile="express",
        angle_deg=45,
        q0=q0_3dof
        # xT uses default value - tossing motion is independent of target location
    )

    try:
        # Get 3-DOF planar trajectory
        sol_3dof = toss_planner.get_trajectory(target_speed)

        # Map to 7-DOF with calculated alignment
        # Preserve J0, J2, J4, J6 from current configuration
        sol_7dof = toss_planner.map_to_7dof(
            sol_3dof["Q"],
            sol_3dof["Qd"],
            sol_3dof["Qdd"],
            base_angle_j0=current_q[0],  # Use current J0 (which is j0_aligned)
            j2=current_q[2],
            j4=current_q[4],
            j6=current_q[6]
        )

        # Release 4 steps before peak velocity
        release_index = sol_3dof["index"] - 4
        split_index = sol_3dof["index"]

        # 5. Execute Toss
        rospy.loginfo("Executing toss with alignment...")
        sensor.start_listening()
        execute_toss_trajectory(robot, gripper, sol_7dof, release_index, split_index)

        # 6. Check Result
        rospy.loginfo("Waiting for landing...")
        pos, obj = sensor.get_landing_result(timeout=5.0)

        if pos:
            rospy.loginfo(f"SUCCESS! Object '{obj}' landed at ({pos.x:.3f}, {pos.y:.3f})")
            rospy.loginfo(f"Target was at ({target_world[0]:.3f}, {target_world[1]:.3f})")
            error_x = pos.x - target_world[0]
            error_y = pos.y - target_world[1]
            rospy.loginfo(f"Error: X={error_x:.3f}m, Y={error_y:.3f}m")
            return pos.x, pos.y, True
        else:
            rospy.loginfo("Missed - No contact detected on landing table.")
            return None, None, False

    except Exception as e:
        rospy.logerr(f"Tossing Failed: {e}")
        import traceback
        traceback.print_exc()
        return None, None, False

def run_aligned_tossing_experiment():
    """Main experiment function."""
    rospy.init_node('test_tossing_aligned_script')

    rospy.loginfo("\n" + "=" * 70)
    rospy.loginfo("ALIGNED TOSSING EXPERIMENT")
    rospy.loginfo("Testing base alignment for targets at different Y positions")
    rospy.loginfo("=" * 70)

    # Setup
    robot = SawyerInterface()
    gripper = GripperInterface()
    manager = GazeboObjectManager(wait_for_services=True)
    sensor = LandingSensor()

    # Planners
    rp = rospkg.RosPack()
    urdf_path = rp.get_path('grasping') + "/sawyer_model.urdf"
    kinematics = CasadiKinematics(urdf_path, "base", "right_gripper_tip")
    pick_planner = CasadiPlanner(kinematics)

    # Initialize CSV log
    with open(LOG_FILE, 'w') as f:
        writer = csv.writer(f)
        writer.writerow([
            'Target_Name', 'Target_X', 'Target_Y', 'Commanded_Speed',
            'Landing_X', 'Landing_Y', 'Error_X', 'Error_Y', 'Success'
        ])

    rospy.loginfo(f"Logging results to {LOG_FILE}")

    # Move to neutral position
    rospy.loginfo("\nMoving to Neutral/Home...")
    q_curr = robot.get_joint_positions()
    traj = pick_planner.plan_joint(q_curr, cfg.NEUTRAL_JOINT_POS, joint_speed=1.5)
    execute_trajectory(robot, traj)

    # Run tests for each target
    TRIALS_PER_TARGET = 2
    for target_config in TEST_TARGETS:
        for trial in range(TRIALS_PER_TARGET):
            rospy.loginfo(f"\n### TRIAL {trial+1}/{TRIALS_PER_TARGET} for {target_config['name']} ###")

            lx, ly, success = run_single_toss(
                target_config, robot, gripper, pick_planner, manager, sensor
            )

            # Calculate errors
            target_x = target_config["world_pos"][0]
            target_y = target_config["world_pos"][1]
            commanded_speed = target_config["speed"]
            error_x = (lx - target_x) if lx is not None else None
            error_y = (ly - target_y) if ly is not None else None

            # Log data
            with open(LOG_FILE, 'a') as f:
                writer = csv.writer(f)
                writer.writerow([
                    target_config["name"],
                    target_x,
                    target_y,
                    commanded_speed,
                    lx, ly,
                    error_x, error_y,
                    success
                ])

            # Return to neutral
            rospy.loginfo("Returning to neutral...")
            q_curr = robot.get_joint_positions()
            traj = pick_planner.plan_joint(q_curr, cfg.NEUTRAL_JOINT_POS, joint_speed=1.5)
            execute_trajectory(robot, traj)

            rospy.sleep(1.0)

    # Cleanup
    manager.despawn("toss_cube")
    manager.despawn("target_marker")

    rospy.loginfo("\n" + "=" * 70)
    rospy.loginfo("ALIGNED TOSSING EXPERIMENT COMPLETE")
    rospy.loginfo(f"Results saved to {LOG_FILE}")
    rospy.loginfo("=" * 70)

if __name__ == "__main__":
    try:
        run_aligned_tossing_experiment()
    except rospy.ROSInterruptException:
        pass
