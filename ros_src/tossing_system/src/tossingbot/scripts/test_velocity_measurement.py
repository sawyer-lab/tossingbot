#!/usr/bin/env python3.8
"""
Test script to measure actual velocities achieved during tosses.

This script executes tosses at different commanded speeds and uses
the VelocityMonitor to measure the actual velocities achieved by both
the end-effector and the object. This helps validate physics calculations
and understand discrepancies between commanded and actual velocities.
"""
import rospy
import numpy as np
import rospkg
import csv
from geometry_msgs.msg import Pose, Point, Quaternion

from tossingbot.hardware.sawyer import SawyerInterface, ControlMode, RobotCommand
from tossingbot.hardware.gripper import GripperInterface
from tossingbot.planning.casadi_planner import CasadiPlanner
from tossingbot.planning.kinematics import CasadiKinematics
from tossingbot.tossing.motion_planner import TossingPlanner
from tossingbot.environment.gazebo_object_manager import GazeboObjectManager
from tossingbot.utils.velocity_monitor import VelocityMonitor
from tossingbot import config as cfg

# Configuration
ROBOT_Z_OFFSET = 1.0
# Pick position (boxes already shifted in world file to align with gripper offset)
PICK_POS_WORLD = [0.60, 0.0, 0.760]  # Centered on work table
TARGET_QUAT = [0, 1, 0, 0]  # Vertical grasp
LOG_FILE = "velocity_measurements.csv"

# Test speeds
TEST_SPEEDS = [1.0, 1.2, 1.5, 1.8, 2.0]  # m/s


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


def execute_toss_with_monitoring(robot, gripper, sol, release_index, split_index, vel_monitor):
    """
    Execute toss trajectory while monitoring velocities.
    """
    rospy.loginfo(f"Executing Toss with velocity monitoring...")
    rospy.loginfo(f"Release at index {release_index}, Split at {split_index}")

    Q = sol["Q"]
    Qd = sol["Qd"]
    Qdd = sol["Qdd"]
    N = Q.shape[0]

    # Create command stream
    stream = []
    for k in range(N):
        cmd = RobotCommand(
            position=Q[k, :].tolist(),
            velocity=Qd[k, :].tolist(),
            acceleration=Qdd[k, :].tolist()
        )
        stream.append(cmd)

    robot._command_msg.mode = ControlMode.TRAJECTORY

    # Start velocity recording
    vel_monitor.start_recording()

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

        # Send command
        robot._command_msg.position = cmd.position
        robot._command_msg.velocity = cmd.velocity
        robot._command_msg.acceleration = cmd.acceleration
        robot._command_msg.header.stamp = rospy.Time.now()
        robot._pub_joint_cmd.publish(robot._command_msg)

        # Record EE velocity
        ee_vel = robot.get_endpoint_velocity()
        if ee_vel:
            vel_monitor.record_ee_velocity(ee_vel)

        robot._rate.sleep()

    rospy.loginfo("Toss Phase Complete.")

    # Keep recording for a bit after toss to capture object velocity
    rospy.sleep(0.5)

    # Stop recording
    vel_monitor.stop_recording()

    return True


def run_single_velocity_test(target_speed, robot, gripper, pick_planner, manager, vel_monitor, target_pos_world=None):
    """
    Run a single toss at a specified speed and measure actual velocities.

    Args:
        target_pos_world: Optional [x, y, z] target position for J0 alignment.
                         If None, uses J0=0 (workspace center)
    """
    rospy.loginfo("\n" + "=" * 70)
    rospy.loginfo(f"VELOCITY TEST - Commanded Speed: {target_speed} m/s")
    rospy.loginfo("=" * 70)

    # Reset scene
    rospy.loginfo("Resetting scene...")
    manager.despawn("toss_cube")
    rospy.sleep(0.5)
    manager.spawn("cube", "toss_cube", Pose(
        position=Point(*PICK_POS_WORLD),
        orientation=Quaternion(0, 0, 0, 1)
    ))
    rospy.sleep(1.0)

    # Pick sequence
    rospy.loginfo("Starting pick sequence...")
    gripper.open()

    pick_target = to_robot_frame(PICK_POS_WORLD)
    hover_target = list(pick_target)
    hover_target[2] += 0.05

    # Move to hover
    q_curr = robot.get_joint_positions()
    q_hover = pick_planner.compute_inverse_kinematics(q_curr, hover_target, TARGET_QUAT)

    if q_hover:
        traj = pick_planner.plan_joint(q_curr, q_hover, joint_speed=1.5)
        execute_trajectory(robot, traj)
    else:
        rospy.logerr("Hover IK Failed!")
        return None

    # Approach
    q_curr = robot.get_joint_positions()
    traj = pick_planner.plan_cartesian(q_curr, pick_target, TARGET_QUAT, linear_speed=0.1)
    execute_trajectory(robot, traj)

    # Grasp
    rospy.sleep(0.2)
    gripper.close()
    rospy.sleep(0.5)

    if not gripper.is_grasping():
        rospy.logwarn("Grasp failed!")
        gripper.open()
        return None

    # Lift
    q_curr = robot.get_joint_positions()
    traj = pick_planner.plan_cartesian(q_curr, hover_target, TARGET_QUAT, linear_speed=0.1)
    execute_trajectory(robot, traj)

    # Move to fixed toss-ready position (J2=0, J4=0, J6=1.766, J0=0)
    rospy.loginfo("Moving to toss-ready position...")
    q_curr = robot.get_joint_positions()
    traj = pick_planner.plan_joint(q_curr, cfg.TOSS_READY_POS, joint_speed=1.5)
    execute_trajectory(robot, traj)

    # Rotate J0 based on target location (if specified)
    if target_pos_world is not None:
        target_robot = to_robot_frame(target_pos_world)
        j0_aligned = np.arctan2(target_robot[1], target_robot[0])
        rospy.loginfo(f"Rotating J0 to {np.rad2deg(j0_aligned):.2f}° for target alignment")

        # Create configuration with only J0 changed
        q_aligned = list(cfg.TOSS_READY_POS)
        q_aligned[0] = j0_aligned

        q_curr = robot.get_joint_positions()
        traj = pick_planner.plan_joint(q_curr, q_aligned, joint_speed=1.0)
        execute_trajectory(robot, traj)
    else:
        rospy.loginfo("Using J0=0 (workspace center, no target specified)")

    # NOTE: Tossing motion (J1, J3, J5) is independent of target location
    # All tosses use the same planar motion for a given speed

    # Plan toss
    rospy.loginfo(f"Planning toss at {target_speed} m/s...")
    current_q = np.array(robot.get_joint_positions())
    q0_3dof = np.array([current_q[1], current_q[3], current_q[5]])

    toss_planner = TossingPlanner(profile="express", angle_deg=45, q0=q0_3dof)

    try:
        sol_3dof = toss_planner.get_trajectory(target_speed)

        sol_7dof = toss_planner.map_to_7dof(
            sol_3dof["Q"],
            sol_3dof["Qd"],
            sol_3dof["Qdd"],
            base_angle_j0=current_q[0],
            j2=current_q[2],
            j4=current_q[4],
            j6=current_q[6]
        )

        release_index = sol_3dof["index"] - 4
        split_index = sol_3dof["index"]

        # Execute with monitoring
        rospy.loginfo("Executing toss with velocity monitoring...")
        execute_toss_with_monitoring(robot, gripper, sol_7dof, release_index, split_index, vel_monitor)

        # Analyze results
        rospy.loginfo("Analyzing velocity data...")
        vel_monitor.print_analysis()
        peaks = vel_monitor.get_peak_velocities()

        return peaks

    except Exception as e:
        rospy.logerr(f"Toss failed: {e}")
        import traceback
        traceback.print_exc()
        return None


def run_velocity_measurement_experiment():
    """Main experiment function."""
    rospy.init_node('test_velocity_measurement_script')

    rospy.loginfo("\n" + "=" * 70)
    rospy.loginfo("VELOCITY MEASUREMENT EXPERIMENT")
    rospy.loginfo("Measuring actual vs commanded velocities")
    rospy.loginfo("=" * 70)

    # Setup
    robot = SawyerInterface()
    gripper = GripperInterface()
    manager = GazeboObjectManager(wait_for_services=True)
    vel_monitor = VelocityMonitor(model_name="toss_cube")

    # Planners
    rp = rospkg.RosPack()
    urdf_path = rp.get_path('grasping') + "/sawyer_model.urdf"
    kinematics = CasadiKinematics(urdf_path, "base", "right_gripper_tip")
    pick_planner = CasadiPlanner(kinematics)

    # Initialize CSV
    with open(LOG_FILE, 'w') as f:
        writer = csv.writer(f)
        writer.writerow([
            'Commanded_Speed', 'EE_Peak_Speed', 'Obj_Peak_Speed',
            'EE_vx', 'EE_vy', 'EE_vz',
            'Obj_vx', 'Obj_vy', 'Obj_vz',
            'Obj_Angle_Deg'
        ])

    rospy.loginfo(f"Logging to {LOG_FILE}")

    # Move to neutral
    rospy.loginfo("\nMoving to neutral...")
    q_curr = robot.get_joint_positions()
    traj = pick_planner.plan_joint(q_curr, cfg.NEUTRAL_JOINT_POS, joint_speed=1.5)
    execute_trajectory(robot, traj)

    # Test each speed
    for speed in TEST_SPEEDS:
        rospy.loginfo(f"\n### TESTING SPEED: {speed} m/s ###")

        peaks = run_single_velocity_test(speed, robot, gripper, pick_planner, manager, vel_monitor)

        # Log results
        if peaks:
            ee_peak = peaks.get('ee_peak', {})
            obj_peak = peaks.get('obj_peak', {})

            ee_speed = ee_peak.get('v_mag', None)
            obj_speed = obj_peak.get('v_mag', None)
            ee_vx = ee_peak.get('vx', None)
            ee_vy = ee_peak.get('vy', None)
            ee_vz = ee_peak.get('vz', None)
            obj_vx = obj_peak.get('vx', None)
            obj_vy = obj_peak.get('vy', None)
            obj_vz = obj_peak.get('vz', None)

            # Calculate angle
            obj_angle = None
            if obj_vx is not None and obj_vy is not None and obj_vz is not None:
                v_horizontal = np.sqrt(obj_vx**2 + obj_vy**2)
                obj_angle = np.rad2deg(np.arctan2(obj_vz, v_horizontal))

            with open(LOG_FILE, 'a') as f:
                writer = csv.writer(f)
                writer.writerow([
                    speed, ee_speed, obj_speed,
                    ee_vx, ee_vy, ee_vz,
                    obj_vx, obj_vy, obj_vz,
                    obj_angle
                ])

        # Return to neutral
        rospy.loginfo("Returning to neutral...")
        q_curr = robot.get_joint_positions()
        traj = pick_planner.plan_joint(q_curr, cfg.NEUTRAL_JOINT_POS, joint_speed=1.5)
        execute_trajectory(robot, traj)

        rospy.sleep(1.0)

    # Cleanup
    manager.despawn("toss_cube")

    rospy.loginfo("\n" + "=" * 70)
    rospy.loginfo("VELOCITY MEASUREMENT EXPERIMENT COMPLETE")
    rospy.loginfo(f"Results saved to {LOG_FILE}")
    rospy.loginfo("=" * 70)


if __name__ == "__main__":
    try:
        run_velocity_measurement_experiment()
    except rospy.ROSInterruptException:
        pass
