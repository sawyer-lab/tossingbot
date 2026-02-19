#!/usr/bin/env python3
"""
run_speed_sweep.py — Run multiple toss experiments at varying release speeds.

Iterates over SPEEDS, executing the full pick->grasp->toss pipeline at each
speed and saving per-run JSON results and plots.  After all runs a summary
plot is generated comparing landing position, velocity loss, and release
timing across speeds.

Usage:
    python run_speed_sweep.py [--speeds 2.0 2.5 3.0] [--release-offset 0]
"""
import argparse
import os
import sys
import time
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from robot_client import RobotClient, GripperClient, GazeboClient, ContactSensorClient
from planning.casadi_planner import CasadiPlanner
from planning.kinematics import CasadiKinematics
from tossing.motion_planner import TossingPlanner
import config as cfg
from toss_recorder import execute_toss_with_comparison
from toss_analysis import plot_results, save_results, analyze_release_timing

# ---------------------------------------------------------------------------
# Defaults (override via CLI arguments)
# ---------------------------------------------------------------------------
DEFAULT_SPEEDS         = [1.0, 1.25, 1.5, 1.75, 2.0, 2.25, 2.5, 2.7]  # m/s
DEFAULT_RELEASE_OFFSET = 0                             # steps before peak

CONTAINER_HOST = os.environ.get("ROBOT_HOST", "localhost")
MODEL_PATHS    = [os.path.join(cfg.PROJECT_ROOT, "ros", "environments", "models")]

OBJECT_NAME    = "toss_cube"
PICK_POS_WORLD = [0.60, cfg.CENTER_Y, 0.760]
TARGET_QUAT    = [0, 1, 0, 0]
ROBOT_Z_OFFSET = 1.0
TABLE_HEIGHT   = 0.75
CUBE_SIZE      = 0.035
CUBE_HALF      = CUBE_SIZE / 2
CUBE_QUARTER   = CUBE_SIZE / 4

URDF_PATH = os.path.join(os.path.dirname(__file__), "..", "planning", "sawyer_electric.urdf")


def to_robot_frame(world_pos):
    p = list(world_pos)
    p[2] -= ROBOT_Z_OFFSET
    return p


# ---------------------------------------------------------------------------
# Pick-and-grasp helper (shared across all runs)
# ---------------------------------------------------------------------------
def pick_object(robot, gripper, pick_planner):
    """Execute the full pick sequence.  Returns True on success."""
    pick_target  = to_robot_frame([
        PICK_POS_WORLD[0] - CUBE_HALF,
        PICK_POS_WORLD[1],
        PICK_POS_WORLD[2],
    ])
    hover_target    = list(pick_target)
    hover_target[2] += 0.05

    print("[INFO] Opening gripper...")
    gripper.open()

    print("[INFO] Moving to hover (IK -> joint)...")
    q_curr  = robot.get_joint_angles()
    q_hover = pick_planner.compute_inverse_kinematics(q_curr, hover_target, TARGET_QUAT)
    if q_hover is None:
        print("[ERROR] Hover IK failed.")
        return False
    robot.move_to_joints(q_hover.tolist() if hasattr(q_hover, 'tolist') else list(q_hover))

    print("[INFO] Cartesian approach...")
    q_curr    = robot.get_joint_angles()
    cart_plan = pick_planner.plan_cartesian(q_curr, pick_target, TARGET_QUAT, linear_speed=0.1)
    if cart_plan:
        robot.move_to_joints(cart_plan[-1]['position'])

    print("[INFO] Grasping...")
    time.sleep(0.2)
    gripper.close()
    time.sleep(0.5)

    grip_state = gripper.get_state()
    if not grip_state.get('is_grasping', False):
        print("[ERROR] Grasp failed.")
        gripper.open()
        return False

    print("[INFO] Lifting...")
    q_curr    = robot.get_joint_angles()
    lift_plan = pick_planner.plan_cartesian(q_curr, hover_target, TARGET_QUAT, linear_speed=0.1)
    if lift_plan:
        robot.move_to_joints(lift_plan[-1]['position'])

    return True


# ---------------------------------------------------------------------------
# Single-speed experiment
# ---------------------------------------------------------------------------
def run_single(robot, gripper, gazebo, contact, pick_planner, speed, release_offset, sweep_dir):
    """Spawn, pick, toss at *speed* m/s.  Returns summary dict or None."""
    run_ts  = int(time.time())
    run_dir = os.path.join(sweep_dir, f"speed_{speed:.1f}_{run_ts}")
    os.makedirs(run_dir, exist_ok=True)

    # Reset scene
    print(f"\n[SWEEP] --- Speed {speed:.1f} m/s ---")
    print("[INFO] Resetting scene...")
    gazebo.despawn(OBJECT_NAME)
    time.sleep(0.5)
    gazebo.spawn("cube", OBJECT_NAME, list(PICK_POS_WORLD))
    time.sleep(1.0)

    # Move to neutral, then pick
    robot.move_to_joints(cfg.NEUTRAL_JOINT_POS)
    if not pick_object(robot, gripper, pick_planner):
        print(f"[SWEEP] Skipping speed {speed:.1f} m/s — pick failed.")
        return None

    # Move to toss-ready position
    robot.move_to_joints(cfg.TOSS_READY_POS)

    # Plan toss
    q_curr   = np.array(robot.get_joint_angles())
    q0_3dof  = np.array([q_curr[1], q_curr[3], q_curr[5]])
    planner  = TossingPlanner(profile="express", angle_deg=45, q0=q0_3dof)

    sol_3d = planner.get_trajectory(speed)
    sol_7d = planner.map_to_7dof(
        sol_3d["Q"], sol_3d["Qd"], sol_3d["Qdd"],
        q_curr[0], q_curr[2], q_curr[4], q_curr[6],
    )

    peak_idx      = sol_3d["index"]
    release_delay = max(0.0, (peak_idx - release_offset) * 0.01)
    print(f"[INFO] Peak index: {peak_idx} | Offset: {release_offset} | "
          f"Release delay: {release_delay:.3f}s")

    # Execute
    results = execute_toss_with_comparison(
        robot, gazebo, contact, sol_7d, release_delay,
        container_host=CONTAINER_HOST,
        object_name=OBJECT_NAME,
    )

    # Save per-run data
    log_file = os.path.join(run_dir, "results.json")
    save_results(results, log_file)
    plot_results(
        results, run_dir,
        object_name=OBJECT_NAME,
        toss_speed=speed,
        robot_z_offset=ROBOT_Z_OFFSET,
        table_height=TABLE_HEIGHT,
        cube_half=CUBE_HALF,
    )

    # Extract summary metrics
    obj_traj = results['obj_traj']
    final_pos = obj_traj[-1]['pos'] if obj_traj else [None, None, None]

    timing = analyze_release_timing(results, OBJECT_NAME)
    delay_ms   = timing['delay_ms']       if timing else None
    vel_loss   = timing['velocity_loss']  if timing else None

    landing = results.get('landing')
    land_pos = landing['position'] if landing else None

    summary = {
        'speed_target':  speed,
        'run_dir':       run_dir,
        'final_pos':     final_pos,
        'land_pos':      land_pos,
        'delay_ms':      delay_ms,
        'velocity_loss': vel_loss,
        'release_index': results['release_index'],
    }
    print(f"[SWEEP] Done: final_pos={final_pos}, delay={delay_ms}ms, "
          f"vel_loss={vel_loss}")
    return summary


# ---------------------------------------------------------------------------
# Summary plot
# ---------------------------------------------------------------------------
def plot_summary(summaries, sweep_dir):
    """Generate comparative plots across all speeds."""
    speeds  = [s['speed_target']  for s in summaries]
    land_x  = [s['final_pos'][0]  if s['final_pos'][0] is not None else float('nan')
                for s in summaries]
    land_y  = [s['final_pos'][1]  if s['final_pos'][1] is not None else float('nan')
                for s in summaries]
    delays  = [s['delay_ms']      if s['delay_ms']      is not None else float('nan')
                for s in summaries]
    vel_loss = [s['velocity_loss'] if s['velocity_loss'] is not None else float('nan')
                for s in summaries]

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle("Speed Sweep Summary", fontsize=14)

    ax = axes[0, 0]
    ax.plot(speeds, land_x, 'bo-', lw=1.5, ms=7)
    ax.set_xlabel("Target Speed (m/s)")
    ax.set_ylabel("Landing x (m)")
    ax.set_title("Landing X vs Speed")
    ax.grid(True, alpha=0.3)

    ax = axes[0, 1]
    ax.plot(speeds, land_y, 'go-', lw=1.5, ms=7)
    ax.set_xlabel("Target Speed (m/s)")
    ax.set_ylabel("Landing y (m)")
    ax.set_title("Landing Y vs Speed")
    ax.grid(True, alpha=0.3)

    ax = axes[1, 0]
    ax.plot(speeds, delays, 'ro-', lw=1.5, ms=7)
    ax.set_xlabel("Target Speed (m/s)")
    ax.set_ylabel("Gripper Delay (ms)")
    ax.set_title("Gripper Release Delay vs Speed")
    ax.grid(True, alpha=0.3)

    ax = axes[1, 1]
    ax.plot(speeds, vel_loss, 'mo-', lw=1.5, ms=7)
    ax.set_xlabel("Target Speed (m/s)")
    ax.set_ylabel("Velocity Loss (m/s)")
    ax.set_title("Velocity Loss vs Speed")
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out = os.path.join(sweep_dir, "sweep_summary.png")
    plt.savefig(out, dpi=150)
    plt.close()
    print(f"[SWEEP] Summary plot saved to {out}")


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------
def run_sweep(speeds, release_offset):
    timestamp = int(time.time())
    sweep_dir = os.path.join(cfg.PROJECT_ROOT, "logs", "speed_sweep", f"sweep_{timestamp}")
    os.makedirs(sweep_dir, exist_ok=True)

    print(f"=== SPEED SWEEP: {speeds} m/s | release_offset={release_offset} ===")
    print(f"[SWEEP] Results in: {sweep_dir}")

    robot   = RobotClient(protocol='zmq', host=CONTAINER_HOST)
    gripper = GripperClient(protocol='zmq', host=CONTAINER_HOST)
    gazebo  = GazeboClient(protocol='zmq', host=CONTAINER_HOST, model_paths=MODEL_PATHS)
    contact = ContactSensorClient(protocol='zmq', host=CONTAINER_HOST)

    urdf_path    = os.path.abspath(URDF_PATH)
    pick_planner = CasadiPlanner(CasadiKinematics(urdf_path, "base", "right_gripper_tip"))

    summaries = []
    for speed in speeds:
        summary = run_single(robot, gripper, gazebo, contact, pick_planner,
                             speed, release_offset, sweep_dir)
        if summary:
            summaries.append(summary)
        robot.move_to_joints(cfg.NEUTRAL_JOINT_POS)
        time.sleep(1.0)   # brief pause between runs

    # Save sweep-level summary JSON
    summary_file = os.path.join(sweep_dir, "sweep_summary.json")
    with open(summary_file, 'w') as f:
        json.dump(summaries, f, indent=2)
    print(f"[SWEEP] Summary JSON saved to {summary_file}")

    if summaries:
        plot_summary(summaries, sweep_dir)

    print(f"\n=== SWEEP COMPLETE: {len(summaries)}/{len(speeds)} runs succeeded ===")

    robot.close()
    gripper.close()
    gazebo.close()
    contact.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run toss experiments at multiple speeds.")
    parser.add_argument(
        "--speeds", nargs="+", type=float, default=DEFAULT_SPEEDS,
        metavar="SPEED",
        help=f"List of target release speeds in m/s (default: {DEFAULT_SPEEDS})",
    )
    parser.add_argument(
        "--release-offset", type=int, default=DEFAULT_RELEASE_OFFSET,
        metavar="STEPS",
        help=f"Steps before peak index to open gripper (default: {DEFAULT_RELEASE_OFFSET})",
    )
    args = parser.parse_args()

    try:
        run_sweep(args.speeds, args.release_offset)
    except KeyboardInterrupt:
        print("\nAborted.")
