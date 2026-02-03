#!/usr/bin/env python3.8
import rospy
import numpy as np
import rospkg
import json
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
from tossingbot.alignment import TargetAlignment

# --- CONFIGURATION ---
G = 9.806
TOSS_ANGLE_RAD = np.deg2rad(45)
MIN_CMD_SPEED = 0.5
MAX_CMD_SPEED = 2.0
DEFAULT_RELEASE_RADIUS = 0.68
DEFAULT_RELEASE_HEIGHT = 0.11
NUM_TARGETS = 5
X_RANGE = [1.0, 1.3]
Y_OFFSET_RANGE = [-0.15, 0.15]
PICK_POS_WORLD = [0.60, cfg.CENTER_Y, 0.760]
ROBOT_Z_OFFSET = 1.0

def to_robot_frame(world_pos):
    p = list(world_pos)
    p[2] -= ROBOT_Z_OFFSET
    return p

def load_latest_suite_log():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    log_dir = os.path.join(script_dir, "../../../logs/test_suite")
    if not os.path.isdir(log_dir):
        rospy.logwarn(f"Suite log dir not found: {log_dir}")
        return None, None
    files = [os.path.join(log_dir, f) for f in os.listdir(log_dir) if f.endswith(".json")]
    if not files:
        rospy.logwarn(f"No suite logs found in: {log_dir}")
        return None, None
    latest = max(files, key=os.path.getctime)
    with open(latest, "r") as f:
        data = json.load(f)
    return latest, data

def estimate_release_stats(trials):
    release_positions = []
    cmd_speeds = []
    act_speeds = []
    for trial in trials:
        traj = trial.get("trajectory")
        if not traj:
            continue
        vels = np.array([np.linalg.norm(s["vel"]) for s in traj])
        if vels.size == 0:
            continue
        idx = int(np.argmax(vels))
        release_positions.append(traj[idx]["pos"])
        cmd_speeds.append(trial["config"]["speed"])
        act_speeds.append(vels[idx])
    if not release_positions:
        return None
    pos = np.array(release_positions, dtype=float)
    pos[:, 2] -= ROBOT_Z_OFFSET
    radii = np.linalg.norm(pos[:, :2], axis=1)
    speed_map = None
    if len(cmd_speeds) >= 2:
        coeffs = np.polyfit(cmd_speeds, act_speeds, 1)
        if coeffs[0] > 0:
            speed_map = (float(coeffs[0]), float(coeffs[1]))
    return {
        "release_radius": float(np.mean(radii)),
        "release_height": float(np.mean(pos[:, 2])),
        "speed_map": speed_map,
        "count": len(release_positions)
    }

def calculate_required_speed(target_robot, release_radius, release_height, speed_map):
    target_xy = np.array(target_robot[:2], dtype=float)
    dist = np.linalg.norm(target_xy)
    if dist <= 1e-6:
        rospy.logwarn("Target too close to base for inverse solve.")
        return None, None
    release_xy = target_xy / dist * release_radius
    d_xy = np.linalg.norm(target_xy - release_xy)
    dz = release_height - target_robot[2]
    denom = 2.0 * (np.cos(TOSS_ANGLE_RAD) ** 2) * (d_xy * np.tan(TOSS_ANGLE_RAD) + dz)
    if denom <= 0:
        rospy.logwarn(f"Inverse solve invalid: denom={denom:.3f}")
        return None, None
    v_required = np.sqrt(G * d_xy**2 / denom)
    a, b = speed_map
    cmd_speed = (v_required - b) / a
    return np.clip(cmd_speed, MIN_CMD_SPEED, MAX_CMD_SPEED), v_required

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

    log_path, suite_data = load_latest_suite_log()
    if suite_data:
        stats = estimate_release_stats(suite_data)
    else:
        stats = None

    release_radius = DEFAULT_RELEASE_RADIUS
    release_height = DEFAULT_RELEASE_HEIGHT
    speed_map = (1.0, 0.0)
    if stats:
        release_radius = stats["release_radius"]
        release_height = stats["release_height"]
        if stats["speed_map"]:
            speed_map = stats["speed_map"]
        rospy.loginfo(
            f"Loaded suite log {log_path} with {stats['count']} releases "
            f"(radius={release_radius:.3f}, height={release_height:.3f}, "
            f"speed_map=({speed_map[0]:.3f},{speed_map[1]:.3f}))"
        )
    else:
        rospy.logwarn(
            f"No suite log found; using defaults "
            f"(radius={release_radius:.3f}, height={release_height:.3f}, "
            f"speed_map=({speed_map[0]:.3f},{speed_map[1]:.3f}))"
        )
    results = []

    for i in range(NUM_TARGETS):
        # 1. Select Random Target
        tx = np.random.uniform(X_RANGE[0], X_RANGE[1])
        ty = cfg.CENTER_Y + np.random.uniform(Y_OFFSET_RANGE[0], Y_OFFSET_RANGE[1])
        target_world = [tx, ty, cfg.TABLE_HEIGHT]
        target_robot = to_robot_frame(target_world)
        
        # 2. Infer Parameters
        cmd_speed, v_required = calculate_required_speed(
            target_robot,
            release_radius,
            release_height,
            speed_map
        )
        if cmd_speed is None:
            continue
        j0_angle = TargetAlignment.get_base_rotation(target_robot[0], target_robot[1], cfg.CENTER_Y)
        
        rospy.loginfo(f"\n>>> TARGET {i+1}: ({tx:.3f}, {ty:.3f})")
        rospy.loginfo(
            f"Inferred: CmdSpeed={cmd_speed:.3f}, "
            f"RelSpeed={v_required:.3f}, J0={np.rad2deg(j0_angle):.2f}deg"
        )

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
