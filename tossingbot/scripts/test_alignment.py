#!/usr/bin/env python3.8
"""
Test script to verify robot base alignment with target positions.

This script demonstrates the simplified alignment approach:
1. Move to fixed TOSS_READY_POS (ensures J2=0, J4=0, J6=1.766)
2. Rotate ONLY J0 to point at different targets
3. Maintains 3R planar constraint throughout

No tosses are executed - this is for visual verification of alignment only.
"""
import rospy
import numpy as np
import rospkg
from geometry_msgs.msg import Pose, Point, Quaternion

from tossingbot.hardware.sawyer import SawyerInterface, ControlMode, RobotCommand
from tossingbot.hardware.gripper import GripperInterface
from tossingbot.planning.casadi_planner import CasadiPlanner
from tossingbot.planning.kinematics import CasadiKinematics
from tossingbot.environment.gazebo_object_manager import GazeboObjectManager
from tossingbot import config as cfg

# --- CONFIGURATION ---
ROBOT_Z_OFFSET = 1.0
GRIPPER_Y_OFFSET = 0.1363  # meters (136.3 mm)

# Test targets in WORLD frame
# Boxes in world file are already shifted to align with gripper offset
TEST_TARGETS_WORLD = [
    [0.60, 0.0, 0.760],      # Target at workspace center
    [0.60, 0.15, 0.760],     # Target left of center
    [0.60, -0.15, 0.760],    # Target right of center
    [1.10, 0.0, 0.760],      # Further target at center
    [1.10, 0.15, 0.760],     # Further target left
]

TARGET_QUAT = [0, 1, 0, 0]  # Vertical grasp orientation

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

def verify_alignment(robot, kinematics, target_pos_robot, j0_calculated):
    """
    Verify alignment by computing forward kinematics and checking
    if the gripper is pointing toward the target.
    Also verify that J2, J4, J6 are maintained at correct values.
    """
    # Get current joint positions
    q_current = robot.get_joint_positions()

    # Compute forward kinematics to get gripper position
    ee_pos = kinematics.forward_kinematics(q_current)

    rospy.loginfo("=" * 60)
    rospy.loginfo("ALIGNMENT VERIFICATION")
    rospy.loginfo("=" * 60)
    rospy.loginfo(f"Target Position (robot frame): [{target_pos_robot[0]:.3f}, {target_pos_robot[1]:.3f}, {target_pos_robot[2]:.3f}]")
    rospy.loginfo(f"Calculated J0: {np.rad2deg(j0_calculated):.2f} degrees ({j0_calculated:.4f} rad)")
    rospy.loginfo(f"Actual J0:     {np.rad2deg(q_current[0]):.2f} degrees ({q_current[0]:.4f} rad)")
    rospy.loginfo(f"Gripper Position: [{ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f}]")

    # Calculate angle from gripper to target in XY plane
    delta_x = target_pos_robot[0] - ee_pos[0]
    delta_y = target_pos_robot[1] - ee_pos[1]
    angle_to_target = np.arctan2(delta_y, delta_x)

    rospy.loginfo(f"Angle to target from gripper: {np.rad2deg(angle_to_target):.2f} degrees")
    rospy.loginfo(f"J0 Alignment error: {np.rad2deg(abs(angle_to_target - q_current[0])):.2f} degrees")

    # Verify 3R planar constraint
    rospy.loginfo("\n3R Planar Constraint Check:")
    rospy.loginfo(f"  J2 = {q_current[2]:.4f} rad (should be ~0.0)")
    rospy.loginfo(f"  J4 = {q_current[4]:.4f} rad (should be ~0.0)")
    rospy.loginfo(f"  J6 = {q_current[6]:.4f} rad (should be ~1.766)")

    constraint_ok = abs(q_current[2]) < 0.01 and abs(q_current[4]) < 0.01 and abs(q_current[6] - 1.766) < 0.01
    if constraint_ok:
        rospy.loginfo("  ✓ 3R Planar constraint MAINTAINED")
    else:
        rospy.logwarn("  ✗ 3R Planar constraint VIOLATED!")

    rospy.loginfo("=" * 60)

def test_single_alignment(robot, gripper, planner, kinematics, manager, target_world):
    """
    Test alignment for a single target position.

    Steps:
    1. Spawn a visual marker at the target
    2. Calculate required J0 for alignment
    3. Move robot to aligned position
    4. Verify and display alignment info
    5. Wait for user to observe
    """
    rospy.loginfo("\n" + "=" * 60)
    rospy.loginfo(f"TESTING ALIGNMENT FOR TARGET: {target_world}")
    rospy.loginfo("=" * 60)

    # Convert to robot frame
    target_robot = to_robot_frame(target_world)

    # 1. Spawn marker at target
    rospy.loginfo("Spawning target marker...")
    manager.despawn("target_marker")
    rospy.sleep(0.3)
    manager.spawn("cube", "target_marker", Pose(
        position=Point(*target_world),
        orientation=Quaternion(0, 0, 0, 1)
    ))
    rospy.sleep(0.5)

    # 2. Calculate required J0 for alignment (just the angle)
    j0_required = np.arctan2(target_robot[1], target_robot[0])
    rospy.loginfo(f"Calculated J0 for alignment: {np.rad2deg(j0_required):.2f} degrees")

    # 3. Move to neutral first
    rospy.loginfo("Moving to neutral position...")
    gripper.open()
    q_curr = robot.get_joint_positions()
    traj = planner.plan_joint(q_curr, cfg.NEUTRAL_JOINT_POS, joint_speed=1.5)
    execute_trajectory(robot, traj)

    # 4. Move to toss-ready position (ensures J2=0, J4=0, J6=1.766)
    rospy.loginfo("Moving to toss-ready position...")
    q_curr = robot.get_joint_positions()
    traj = planner.plan_joint(q_curr, cfg.TOSS_READY_POS, joint_speed=1.5)
    execute_trajectory(robot, traj)

    # 5. Rotate ONLY J0 to align with target
    rospy.loginfo(f"Rotating J0 to {np.rad2deg(j0_required):.2f}° to align with target...")
    q_aligned = list(cfg.TOSS_READY_POS)
    q_aligned[0] = j0_required  # Only change J0

    q_curr = robot.get_joint_positions()
    traj = planner.plan_joint(q_curr, q_aligned, joint_speed=1.0)
    execute_trajectory(robot, traj)

    # 6. Verify alignment
    rospy.loginfo("J2, J4, J6 remain at 0, 0, 1.766 (3R planar constraint maintained)")
    verify_alignment(robot, kinematics, target_robot, j0_required)

    # 8. Wait for user observation
    rospy.loginfo("\n*** ROBOT IS NOW ALIGNED WITH TARGET ***")
    rospy.loginfo("Observe the robot position and press Ctrl+C when ready to continue...")
    rospy.sleep(5.0)  # Wait 5 seconds for observation

    # 9. Cleanup
    manager.despawn("target_marker")

    return True

def run_alignment_tests():
    """Main test function."""
    rospy.init_node('test_alignment_script')

    rospy.loginfo("\n" + "=" * 60)
    rospy.loginfo("ROBOT BASE ALIGNMENT TEST")
    rospy.loginfo("=" * 60)
    rospy.loginfo(f"Gripper Y-Offset: {GRIPPER_Y_OFFSET * 1000:.1f} mm")
    rospy.loginfo(f"Testing {len(TEST_TARGETS_WORLD)} target positions")
    rospy.loginfo("=" * 60 + "\n")

    # Setup
    robot = SawyerInterface()
    gripper = GripperInterface()
    manager = GazeboObjectManager(wait_for_services=True)

    # Setup planner and kinematics
    rp = rospkg.RosPack()
    urdf_path = rp.get_path('grasping') + "/sawyer_model.urdf"
    kinematics = CasadiKinematics(urdf_path, "base", "right_gripper_tip")
    planner = CasadiPlanner(kinematics)

    # Move to neutral to start
    rospy.loginfo("Initial move to neutral position...")
    q_curr = robot.get_joint_positions()
    traj = planner.plan_joint(q_curr, cfg.NEUTRAL_JOINT_POS, joint_speed=1.5)
    execute_trajectory(robot, traj)
    rospy.sleep(1.0)

    # Test each target
    for i, target_world in enumerate(TEST_TARGETS_WORLD):
        rospy.loginfo(f"\n### TEST {i+1}/{len(TEST_TARGETS_WORLD)} ###")

        try:
            success = test_single_alignment(
                robot, gripper, planner, kinematics, manager, target_world
            )

            if not success:
                rospy.logwarn(f"Test {i+1} failed, continuing...")

        except KeyboardInterrupt:
            rospy.loginfo("\nTest interrupted by user. Moving to next target...")
            rospy.sleep(0.5)

        except Exception as e:
            rospy.logerr(f"Test {i+1} failed with error: {e}")
            import traceback
            traceback.print_exc()

    # Return to neutral
    rospy.loginfo("\nAll tests complete. Returning to neutral...")
    q_curr = robot.get_joint_positions()
    traj = planner.plan_joint(q_curr, cfg.NEUTRAL_JOINT_POS, joint_speed=1.5)
    execute_trajectory(robot, traj)

    rospy.loginfo("\n" + "=" * 60)
    rospy.loginfo("ALIGNMENT TESTS COMPLETE")
    rospy.loginfo("=" * 60)

if __name__ == "__main__":
    try:
        run_alignment_tests()
    except rospy.ROSInterruptException:
        pass
