#!/usr/bin/env python3
"""
test_toss_full.py — Full spawn->grasp->toss pipeline with trajectory comparison.

Uses the robot_client API (ZMQ) to communicate with the container.

Recording strategy:
  - During trajectory: SUB socket listens to the 100Hz PUB broadcast (port 5556)
    while the REQ socket is blocked inside execute_toss_trajectory.
  - Post-flight: Gazebo is polled at ~50Hz to track the cube arc.

Plots produced:
  1. joint_positions.png   — planned vs actual for J1, J3, J5
  2. joint_velocities.png  — planned vs actual for J1, J3, J5
  3. ee_position_speed.png — EE x/y/z + speed over time
  4. cube_vs_ee.png        — cube position/speed vs EE (post-release)
  5. ballistic.png         — predicted ballistic arc vs actual cube trajectory
"""
import threading
import numpy as np
import json
import os
import sys
import time
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# Ensure src/ is on the path for absolute imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from robot_client import RobotClient, GripperClient, GazeboClient, ContactSensorClient
from planning.casadi_planner import CasadiPlanner
from planning.kinematics import CasadiKinematics
from tossing.motion_planner import TossingPlanner
import config as cfg

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
CONTAINER_HOST = os.environ.get("ROBOT_HOST", "localhost")
MODEL_PATHS    = [os.path.join(cfg.PROJECT_ROOT, "ros", "environments", "models")]

OBJECT_NAME    = "toss_cube"
PICK_POS_WORLD = [0.60, cfg.CENTER_Y, 0.760]
TARGET_QUAT    = [0, 1, 0, 0]   # Vertical grasp
TOSS_SPEED     = 2.7            # m/s target release speed
RELEASE_OFFSET = 0              # Steps before peak index to open gripper
ROBOT_Z_OFFSET = 1.0            # Robot base height in world frame (m)
TABLE_HEIGHT   = 0.75           # Table surface height in world frame (m)
CUBE_SIZE      = 0.035          # Cube full width (m), model "cube"
CUBE_HALF      = CUBE_SIZE / 2  # Half-width for center calculations
CUBE_QUARTER   = CUBE_SIZE / 4  # Quarter-width for safety margin in pick

TIMESTAMP = int(time.time())
LOG_DIR   = os.path.join(cfg.PROJECT_ROOT, "logs", "toss_comparison")
LOG_FILE  = os.path.join(LOG_DIR, f"toss_comparison_{TIMESTAMP}.json")
PLOT_DIR  = os.path.join(LOG_DIR, f"plots_{TIMESTAMP}")

URDF_PATH = os.path.join(os.path.dirname(__file__), "..", "planning", "sawyer_electric.urdf")


# ---------------------------------------------------------------------------
# Utilities
# ---------------------------------------------------------------------------
def to_robot_frame(world_pos):
    p = list(world_pos)
    p[2] -= ROBOT_Z_OFFSET
    return p


def execute_trajectory(robot, plan_data):
    """Send a planned trajectory to the container for execution."""
    if plan_data is None:
        print("[ERROR] Plan is None!")
        return False
    waypoints = [
        {'position': p['position'], 'velocity': p['velocity'], 'acceleration': p['acceleration']}
        for p in plan_data
    ]
    return robot.execute_trajectory(waypoints, rate_hz=100.0)


def get_ballistic_landing(release_pos_robot, release_vel_world, 
                          robot_z_offset=ROBOT_Z_OFFSET, table_z=TABLE_HEIGHT, 
                          cube_half=CUBE_HALF, g=9.806):
    """
    Calculate landing position using projectile motion physics.
    
    Accounts for coordinate frames:
    - Release position given in ROBOT frame (planner coordinates)
    - Converted to WORLD frame using robot_z_offset
    - Landing plane is table surface + cube half-height (where cube center lands)
    
    Solves: z(t) = z0 + vz*t - 0.5*g*t^2 = landing_z
    Using quadratic formula to find time-of-flight.
    
    Args:
        release_pos_robot: [x, y, z] release position in ROBOT frame (meters)
        release_vel_world: [vx, vy, vz] release velocity in WORLD frame (m/s)
        robot_z_offset: Robot base height in world frame (default 1.0m)
        table_z: Table surface height in world frame (default 0.75m)
        cube_half: Cube half-height to add to landing plane (default 0.0175m)
        g: Gravitational acceleration (m/s^2)
    
    Returns:
        tuple: (landing_position, tof, trajectory) or (None, None, None)
            landing_position: [x, y] numpy array in WORLD frame
            tof: time of flight (seconds)
            trajectory: dict with 't', 'x', 'y', 'z' arrays for plotting (WORLD frame)
    """
    # Convert release position from robot frame to world frame
    x0 = release_pos_robot[0]
    y0 = release_pos_robot[1]
    z0 = release_pos_robot[2] + robot_z_offset  # e.g., 0 + 1.0 = 1.0m
    
    vx, vy, vz = release_vel_world
    
    # Landing plane: table top + cube half-height (where cube CENTER lands)
    landing_z = table_z + cube_half
    
    # Quadratic: z(t) = z0 + vz*t - 0.5*g*t^2 = landing_z
    # Rearrange: 0.5*g*t^2 - vz*t + (landing_z - z0) = 0
    a = 0.5 * g
    b = -vz
    c = landing_z - z0  # Fixed sign: landing_z - z0, not z0 - landing_z
    
    discriminant = b**2 - 4*a*c
    if discriminant < 0:
        return None, None, None
    
    # Time of flight (positive root)
    tof = (vz + np.sqrt(discriminant)) / g
    if tof <= 0:
        return None, None, None
    
    # Landing position in world frame
    land_x = x0 + vx * tof
    land_y = y0 + vy * tof
    
    # Generate trajectory for plotting (world frame)
    t_traj = np.linspace(0, tof, 300)
    x_traj = x0 + vx * t_traj
    y_traj = y0 + vy * t_traj
    z_traj = z0 + vz * t_traj - 0.5 * g * t_traj**2
    
    trajectory = {
        't': t_traj,
        'x': x_traj,
        'y': y_traj,
        'z': z_traj
    }
    
    return np.array([land_x, land_y]), tof, trajectory


def analyze_release_timing(results, object_name):
    """
    Analyze gripper release timing and velocity loss.
    
    Args:
        results: Results dict from execute_toss_with_comparison
        object_name: Name of thrown object
    
    Returns:
        dict: Timing analysis with keys:
            - release_commanded_idx: When gripper command sent
            - release_actual_idx: When object velocity changed
            - delay_ms: Time between command and actual release
            - velocity_at_command: Object velocity when command sent
            - velocity_at_release: Object velocity at actual release
            - velocity_loss: Energy lost during release
            - velocity_loss_pct: Percentage loss
    """
    states = results['actual_states']
    rel_idx = results['release_index']
    
    if rel_idx >= len(states):
        return None
    
    # Extract object velocities from gazebo state in PUB data
    obj_vels = []
    for i, s in enumerate(states):
        gz = s.get('gazebo', {})
        if object_name in gz and 'velocity' in gz[object_name]:
            vel = np.array(gz[object_name]['velocity'])
            obj_vels.append({
                'idx': i,
                'vel': vel,
                'speed': np.linalg.norm(vel)
            })
    
    if len(obj_vels) < rel_idx + 10:
        return None
    
    # Velocity at commanded release
    cmd_data = next((v for v in obj_vels if v['idx'] == rel_idx), None)
    if not cmd_data:
        return None
    
    vel_at_cmd = cmd_data['speed']
    
    # Find actual release by detecting velocity change
    # Look for when object velocity drops (gripper releases)
    actual_rel_idx = rel_idx
    for v in obj_vels:
        if v['idx'] > rel_idx and v['speed'] < vel_at_cmd * 0.95:
            actual_rel_idx = v['idx']
            break
    
    actual_data = next((v for v in obj_vels if v['idx'] == actual_rel_idx), cmd_data)
    vel_at_release = actual_data['speed']
    delay_s = (actual_rel_idx - rel_idx) * 0.01  # 100 Hz
    
    return {
        'release_commanded_idx': rel_idx,
        'release_actual_idx': actual_rel_idx,
        'delay_ms': delay_s * 1000,
        'velocity_at_command': vel_at_cmd,
        'velocity_at_release': vel_at_release,
        'velocity_loss': vel_at_cmd - vel_at_release,
        'velocity_loss_pct': ((vel_at_cmd - vel_at_release) / vel_at_cmd * 100) if vel_at_cmd > 0 else 0,
        'obj_velocities': obj_vels  # For plotting
    }


# ---------------------------------------------------------------------------
# PUB socket subscriber — records robot state during trajectory execution
# (runs in a daemon thread while REQ socket is blocked)
# ---------------------------------------------------------------------------
def _sub_record(host, port, out, stop_ev):
    """Drain the 100Hz PUB socket into out[] until stop_ev is set."""
    import zmq as _zmq
    ctx = _zmq.Context()
    sub = ctx.socket(_zmq.SUB)
    sub.connect(f"tcp://{host}:{port}")
    sub.setsockopt_string(_zmq.SUBSCRIBE, "")
    sub.setsockopt(_zmq.RCVTIMEO, 20)   # 20 ms poll timeout
    while not stop_ev.is_set():
        try:
            out.append(sub.recv_json())
        except _zmq.Again:
            pass
    sub.close()
    ctx.term()


# ---------------------------------------------------------------------------
# Toss execution with comparison recording
# ---------------------------------------------------------------------------
def execute_toss_with_comparison(robot, gazebo, contact, sol_7d, release_delay, kinematics=None):
    """
    Execute toss and collect all data needed for comparison plots.

    Args:
        robot: RobotClient instance
        gazebo: GazeboClient instance
        contact: ContactSensorClient instance
        sol_7d: 7-DOF trajectory solution
        release_delay: Time in seconds to delay gripper opening
        kinematics: CasadiKinematics instance for computing commanded EE velocities (optional)

    Returns a results dict:
      planned_q       (N, 7) numpy array
      planned_qd      (N, 7) numpy array
      commanded_ee_vel (N, 3) numpy array (if kinematics provided)
      release_index   int
      actual_states   list of dicts from PUB socket (aligned to trajectory)
      obj_traj        list of {t, pos, vel} from Gazebo post-flight polling
      landing         dict from contact sensor or None
    """
    Q, Qd, Qdd = sol_7d["Q"], sol_7d["Qd"], sol_7d["Qdd"]
    N = Q.shape[0]

    release_index = int(release_delay * 100)
    release_index = max(0, min(release_index, N - 1))
    print(f"[INFO] Release at step {release_index}/{N} "
          f"(t={release_index * 0.01:.3f}s)")
    
    # Compute commanded EE velocities if kinematics provided
    commanded_ee_vel = None
    if kinematics is not None:
        try:
            commanded_ee_vel = np.zeros((N, 3))
            for i in range(N):
                J = kinematics.compute_jacobian(Q[i, :])
                ee_twist = J @ Qd[i, :]
                commanded_ee_vel[i, :] = ee_twist[:3]  # Linear velocity
        except Exception as e:
            print(f"[WARN] Could not compute commanded EE velocity: {e}")
            commanded_ee_vel = None

    # Start contact sensor detection
    contact.clear()
    contact.start_detection()

    # Start SUB thread before sending trajectory so we don't miss early frames
    actual_states = []
    stop_ev = threading.Event()
    sub_thread = threading.Thread(
        target=_sub_record,
        args=(CONTAINER_HOST, 5556, actual_states, stop_ev),
        daemon=True,
    )
    sub_thread.start()

    # REQ socket blocks for N * 0.01 s; SUB thread records in parallel
    robot.execute_toss_trajectory(Q, Qd, Qdd, release_index)

    stop_ev.set()
    sub_thread.join(timeout=0.5)

    # Keep at most N states; if more arrived, drop early ones (pre-trajectory)
    if len(actual_states) > N:
        actual_states = actual_states[-N:]
    print(f"[INFO] Captured {len(actual_states)}/{N} states via PUB socket")

    # Extract object trajectory from PUB states (100Hz, covers whole trajectory
    # including the flight arc which typically lands DURING the trajectory window).
    # t is relative to trajectory start so release is at release_index * 0.01 s.
    obj_traj = []
    for i, s in enumerate(actual_states):
        gz = s.get('gazebo', {})
        if OBJECT_NAME in gz:
            traj_point = {
                't':   i * 0.01,
                'pos': gz[OBJECT_NAME]['position'],
            }
            # Add velocity if available
            if 'velocity' in gz[OBJECT_NAME]:
                traj_point['vel'] = gz[OBJECT_NAME]['velocity']
            obj_traj.append(traj_point)

    # Wait for landing via contact sensor
    print("[INFO] Waiting for landing detection...")
    landing = contact.wait_for_landing(timeout=3.0)
    if landing:
        print(f"[INFO] Landing detected: {landing['object_name']} at "
              f"[{landing['position'][0]:.3f}, {landing['position'][1]:.3f}, {landing['position'][2]:.3f}]")
    else:
        print("[INFO] No landing detected (timeout)")

    # Short post-trajectory poll via REQ (REQ is free now) to capture settling
    # if the cube is still moving when the trajectory loop ended.
    t0_post = time.time()
    while time.time() - t0_post < 0.5:
        pose = gazebo.get_pose(OBJECT_NAME)
        if pose:
            traj_point = {
                't':   N * 0.01 + (time.time() - t0_post),
                'pos': pose['position'],
            }
            obj_traj.append(traj_point)
        time.sleep(0.02)

    print(f"[INFO] Object trajectory: {len(obj_traj)} samples "
          f"({sum(1 for o in obj_traj if o['t'] < N*0.01)} during traj, "
          f"{sum(1 for o in obj_traj if o['t'] >= N*0.01)} post-traj)")

    return {
        'planned_q':     Q,
        'planned_qd':    Qd,
        'release_index': release_index,
        'actual_states': actual_states,
        'obj_traj':      obj_traj,
        'landing':       landing,
    }


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------
def _extract_actual(states):
    """Pull numpy arrays from PUB socket state list."""
    if not states:
        return None, None, None, None
    try:
        q   = np.array([s['robot']['joint_angles']                   for s in states])
        qd  = np.array([s['robot']['joint_velocities']               for s in states])
        eep = np.array([s['robot']['endpoint_pose']['position']      for s in states])
        eev = np.array([s['robot']['endpoint_velocity']['linear']    for s in states])
        return q, qd, eep, eev
    except (KeyError, TypeError):
        return None, None, None, None


def plot_results(results, save_dir):
    os.makedirs(save_dir, exist_ok=True)

    Q          = np.array(results['planned_q'])
    Qd         = np.array(results['planned_qd'])
    rel_idx    = results['release_index']
    states     = results['actual_states']
    obj_traj   = results['obj_traj']

    N_plan = len(Q)
    t_plan = np.arange(N_plan) * 0.01
    t_rel  = rel_idx * 0.01

    act_q, act_qd, act_eep, act_eev = _extract_actual(states)
    N_act  = len(act_q) if act_q is not None else 0
    t_act  = np.arange(N_act) * 0.01

    JOINTS = [1, 3, 5]

    # ---- 1. Joint Positions: planned vs actual ----
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle("Joint Positions: Planned vs Actual")
    for ax, ji in zip(axes, JOINTS):
        ax.plot(t_plan, Q[:, ji],     'b-',  lw=1.5,         label='Planned')
        if N_act:
            ax.plot(t_act,  act_q[:, ji], color='orange', ls='--', lw=1.5, label='Actual')
        ax.axvline(t_rel, color='r', ls=':', lw=1.2, label='Release')
        ax.set_ylabel(f"J{ji} (rad)")
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel("Time (s)")
    plt.tight_layout()
    plt.savefig(os.path.join(save_dir, "joint_positions.png"), dpi=150)
    plt.close()

    # ---- 2. Joint Velocities: planned vs actual ----
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle("Joint Velocities: Planned vs Actual")
    for ax, ji in zip(axes, JOINTS):
        ax.plot(t_plan, Qd[:, ji],     'b-',  lw=1.5,         label='Planned')
        if N_act:
            ax.plot(t_act,  act_qd[:, ji], color='orange', ls='--', lw=1.5, label='Actual')
        ax.axvline(t_rel, color='r', ls=':', lw=1.2, label='Release')
        ax.set_ylabel(f"J{ji} (rad/s)")
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel("Time (s)")
    plt.tight_layout()
    plt.savefig(os.path.join(save_dir, "joint_velocities.png"), dpi=150)
    plt.close()

    # ---- 3. EE X/Z position + Vx/Vz + speed ----
    if N_act:
        ee_spd = np.linalg.norm(act_eev, axis=1)
        fig, axes = plt.subplots(5, 1, figsize=(10, 12), sharex=True)
        fig.suptitle("End-Effector — X/Z plane")
        for ax, ki, lbl in zip(axes[:2], [0, 2], ['x', 'z']):
            ax.plot(t_act, act_eep[:, ki], 'g-', lw=1.5)
            ax.axvline(t_rel, color='r', ls=':', lw=1.2, label='Release')
            ax.set_ylabel(f"{lbl} (m)")
            ax.legend(loc='upper right', fontsize=8)
            ax.grid(True, alpha=0.3)
        for ax, ki, lbl in zip(axes[2:4], [0, 2], ['Vx', 'Vz']):
            ax.plot(t_act, act_eev[:, ki], 'g-', lw=1.5)
            ax.axvline(t_rel, color='r', ls=':', lw=1.2, label='Release')
            ax.set_ylabel(f"{lbl} (m/s)")
            ax.legend(loc='upper right', fontsize=8)
            ax.grid(True, alpha=0.3)
        axes[4].plot(t_act, ee_spd, 'g-', lw=1.5, label='||v|| (m/s)')
        axes[4].axvline(t_rel, color='r', ls=':', lw=1.2, label='Release')
        axes[4].set_ylabel("Speed (m/s)")
        axes[4].set_xlabel("Time (s)")
        axes[4].legend(loc='upper right', fontsize=8)
        axes[4].grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(os.path.join(save_dir, "ee_position_speed.png"), dpi=150)
        plt.close()

    # ---- 4. Cube vs EE: X/Z position + Vx/Vz velocity components ----
    if obj_traj:
        t_obj = np.array([r['t'] for r in obj_traj])
        p_obj = np.array([r['pos'] for r in obj_traj])

        # Cube velocity components (Vx, Vz) from finite differences
        if len(p_obj) > 1:
            dp     = np.diff(p_obj, axis=0)
            dt_obj = np.diff(t_obj)[:, None]
            v_obj  = dp / dt_obj          # (M-1, 3)
            t_vobj = (t_obj[:-1] + t_obj[1:]) / 2
        else:
            v_obj, t_vobj = np.zeros((0, 3)), np.array([])

        # Both t_act and t_obj share the same i*0.01 clock from trajectory start.
        fig, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=True)
        fig.suptitle("Cube vs End-Effector — X/Z plane (World Frame)\n(red dashed = release)")

        # Row 0: X position
        ax = axes[0]
        if N_act:
            ax.plot(t_act, act_eep[:, 0], 'b-',  lw=1.2, label='EE x',   alpha=0.8)
        ax.plot(t_obj, p_obj[:, 0],       'm-',  lw=1.5, label='Cube x')
        ax.axvline(t_rel, color='r', ls=':', lw=1.2, label='Release')
        ax.set_ylabel("x (m)")
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)

        # Row 1: Z position (add 1.0m to EE z for world frame)
        ax = axes[1]
        if N_act:
            ax.plot(t_act, act_eep[:, 2] + ROBOT_Z_OFFSET, 'b-',  lw=1.2, label='EE z',   alpha=0.8)
        ax.plot(t_obj, p_obj[:, 2],       'm-',  lw=1.5, label='Cube z')
        ax.axvline(t_rel, color='r', ls=':', lw=1.2, label='Release')
        ax.axhline(TABLE_HEIGHT, color='gray', ls=':', lw=1, alpha=0.5, label=f'Table ({TABLE_HEIGHT}m)')
        ax.set_ylabel("z (m)")
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)

        # Row 2: Vx
        ax = axes[2]
        if N_act:
            ax.plot(t_act, act_eev[:, 0], 'b-',  lw=1.2, label='EE Vx',  alpha=0.8)
        if len(v_obj):
            ax.plot(t_vobj, v_obj[:, 0],  'm-',  lw=1.5, label='Cube Vx')
        ax.axvline(t_rel, color='r', ls=':', lw=1.2, label='Release')
        ax.set_ylabel("Vx (m/s)")
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)

        # Row 3: Vz
        ax = axes[3]
        if N_act:
            ax.plot(t_act, act_eev[:, 2], 'b-',  lw=1.2, label='EE Vz',  alpha=0.8)
        if len(v_obj):
            ax.plot(t_vobj, v_obj[:, 2],  'm-',  lw=1.5, label='Cube Vz')
        ax.axvline(t_rel, color='r', ls=':', lw=1.2, label='Release')
        ax.set_ylabel("Vz (m/s)")
        ax.set_xlabel("Time from trajectory start (s)")
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig(os.path.join(save_dir, "cube_vs_ee.png"), dpi=150)
        plt.close()
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig(os.path.join(save_dir, "cube_vs_ee.png"), dpi=150)
        plt.close()

    # ---- 5. Ballistic prediction vs actual trajectory ----
    if obj_traj and N_act and rel_idx < N_act:
        print("[DEBUG] Creating ballistic plot...")
        try:
            # Release position and velocity
            p0_robot = act_eep[rel_idx]
            v0_actual = act_eev[rel_idx]
            
            # Calculate actual ballistic (using measured velocity)
            pred_actual, tof_actual, traj_actual = get_ballistic_landing(p0_robot, v0_actual)
            
            # For ideal: assume 45 degree angle at 2.0 m/s (commanded)
            speed_ideal = TOSS_SPEED
            angle_rad = np.radians(45)
            v0_ideal = np.array([
                speed_ideal * np.cos(angle_rad),  # vx
                0.0,                               # vy
                speed_ideal * np.sin(angle_rad)    # vz
            ])
            pred_ideal, tof_ideal, traj_ideal = get_ballistic_landing(p0_robot, v0_ideal)
            
            # Get landing position
            actual_land = np.array([p_obj[-1, 0], p_obj[-1, 1]])
            land_z = p_obj[-1, 2]
            
            # Convert release to world frame
            p0_world = p0_robot.copy()
            p0_world[2] += ROBOT_Z_OFFSET
            
            fig, axes = plt.subplots(1, 2, figsize=(15, 7))
            
            title_lines = [
                "Ballistic Predictions vs Actual Trajectory",
                f"Release: [{p0_world[0]:.2f}, {p0_world[1]:.2f}, {p0_world[2]:.2f}] m"
            ]
            if pred_ideal is not None:
                err_ideal = np.linalg.norm(pred_ideal - actual_land) * 100
                title_lines.append(f"Ideal (2.0 m/s @ 45°): TOF={tof_ideal:.3f}s, error={err_ideal:.1f}cm")
            if pred_actual is not None:
                err_actual = np.linalg.norm(pred_actual - actual_land) * 100
                title_lines.append(f"Actual ({np.linalg.norm(v0_actual):.2f} m/s): TOF={tof_actual:.3f}s, error={err_actual:.1f}cm")
            
            fig.suptitle('\n'.join(title_lines), fontsize=10)

            # XZ side view
            ax = axes[0]
            # Ballistic trajectories
            if traj_ideal:
                ax.plot(traj_ideal['x'], traj_ideal['z'], 'b--', lw=2, alpha=0.7, label='Ideal (2.0 m/s @ 45°)')
            if traj_actual:
                ax.plot(traj_actual['x'], traj_actual['z'], 'g-.', lw=2, alpha=0.7, label=f'Actual ({np.linalg.norm(v0_actual):.2f} m/s)')
            # Gazebo trajectory
            ax.plot(p_obj[:, 0], p_obj[:, 2], 'm-', lw=2.5, label='Gazebo trajectory')
            # Markers
            ax.scatter(p0_world[0], p0_world[2], c='red', zorder=5, s=100, marker='o',
                      edgecolors='black', linewidths=1.5, label='Release')
            if pred_ideal is not None:
                ax.scatter(pred_ideal[0], TABLE_HEIGHT + CUBE_HALF, c='blue', zorder=5, s=90, 
                          marker='x', linewidths=2, label=f'Ideal landing')
            if pred_actual is not None:
                ax.scatter(pred_actual[0], TABLE_HEIGHT + CUBE_HALF, c='green', zorder=5, s=90, 
                          marker='+', linewidths=2, label=f'Actual landing')
            ax.scatter(actual_land[0], land_z, c='cyan', zorder=5, s=120, 
                      marker='*', edgecolors='black', linewidths=1.5, label='Measured')
            ax.axhline(TABLE_HEIGHT, color='gray', ls=':', lw=1.5, alpha=0.6, label=f'Table')
            ax.set_xlabel("x (m)", fontsize=10)
            ax.set_ylabel("z (m)", fontsize=10)
            ax.set_title("Side view (X-Z)")
            ax.legend(fontsize=8, loc='best')
            ax.grid(True, alpha=0.3)

            # XY top view  
            ax = axes[1]
            if traj_ideal:
                ax.plot(traj_ideal['x'], traj_ideal['y'], 'b--', lw=2, alpha=0.7, label='Ideal')
            if traj_actual:
                ax.plot(traj_actual['x'], traj_actual['y'], 'g-.', lw=2, alpha=0.7, label='Actual')
            ax.plot(p_obj[:, 0], p_obj[:, 1], 'm-', lw=2.5, label='Gazebo')
            ax.scatter(p0_world[0], p0_world[1], c='red', zorder=5, s=100, marker='o',
                      edgecolors='black', linewidths=1.5, label='Release')
            if pred_ideal is not None:
                ax.scatter(pred_ideal[0], pred_ideal[1], c='blue', zorder=5, s=90, 
                          marker='x', linewidths=2, label='Ideal')
            if pred_actual is not None:
                ax.scatter(pred_actual[0], pred_actual[1], c='green', zorder=5, s=90, 
                          marker='+', linewidths=2, label='Actual')
            ax.scatter(actual_land[0], actual_land[1], c='cyan', zorder=5, s=120, 
                      marker='*', edgecolors='black', linewidths=1.5, label='Measured')
            ax.set_xlabel("x (m)", fontsize=10)
            ax.set_ylabel("y (m)", fontsize=10)
            ax.set_title("Top view (X-Y)")
            ax.legend(fontsize=8, loc='best')
            ax.grid(True, alpha=0.3)
            ax.axis('equal')

            plt.tight_layout()
            plt.savefig(os.path.join(save_dir, "ballistic.png"), dpi=150)
            plt.close()
            
            if pred_ideal is not None:
                print(f"[INFO] Ideal ballistic: error={err_ideal:.1f}cm, TOF={tof_ideal:.3f}s")
            if pred_actual is not None:
                print(f"[INFO] Actual ballistic: error={err_actual:.1f}cm, TOF={tof_actual:.3f}s")
                
        except Exception as e:
            import traceback
            print(f"[ERROR] Exception in ballistic plot: {e}")
            traceback.print_exc()
    else:
        print(f"[DEBUG] Skipping ballistic plot")

    # ---- 6. Release Timing and Velocity Loss (NEW) ----
    timing = analyze_release_timing(results, OBJECT_NAME)
    if timing and N_act:
        fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
        fig.suptitle(f"Release Timing Analysis\n"
                    f"Delay: {timing['delay_ms']:.1f} ms  |  "
                    f"Velocity loss: {timing['velocity_loss']:.3f} m/s ({timing['velocity_loss_pct']:.1f}%)")
        
        # Plot 1: Object velocity over time
        ax = axes[0]
        obj_vels = timing['obj_velocities']
        t_vel = np.array([v['idx'] * 0.01 for v in obj_vels])
        speeds = np.array([v['speed'] for v in obj_vels])
        
        ax.plot(t_vel, speeds, 'g-', lw=1.5, label='Object speed')
        ax.axvline(rel_idx * 0.01, color='orange', ls='--', lw=1.2, 
                  label=f'Command sent ({rel_idx})')
        ax.axvline(timing['release_actual_idx'] * 0.01, color='red', ls='--', lw=1.2,
                  label=f'Actual release ({timing["release_actual_idx"]})')
        ax.axhspan(timing['velocity_at_release'], timing['velocity_at_command'],
                  alpha=0.2, color='red', label='Velocity loss')
        ax.set_ylabel("Speed (m/s)")
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.set_title("Object Velocity During Release")
        
        # Plot 2: Delay region zoom
        ax = axes[1]
        zoom_start = max(0, rel_idx - 10)
        zoom_end = min(len(t_vel), timing['release_actual_idx'] + 20)
        zoom_mask = (t_vel >= zoom_start * 0.01) & (t_vel <= zoom_end * 0.01)
        
        ax.plot(t_vel[zoom_mask], speeds[zoom_mask], 'g-', lw=2, label='Object speed (zoomed)')
        ax.axvline(rel_idx * 0.01, color='orange', ls='--', lw=1.5, 
                  label='Command')
        ax.axvline(timing['release_actual_idx'] * 0.01, color='red', ls='--', lw=1.5,
                  label='Release')
        ax.fill_betweenx([speeds[zoom_mask].min(), speeds[zoom_mask].max()],
                        rel_idx * 0.01, timing['release_actual_idx'] * 0.01,
                        alpha=0.2, color='yellow', label=f'Delay: {timing["delay_ms"]:.1f} ms')
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Speed (m/s)")
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.set_title("Release Region (Zoomed)")
        
        plt.tight_layout()
        plt.savefig(os.path.join(save_dir, "release_timing.png"), dpi=150)
        plt.close()
        print(f"[INFO] Release delay: {timing['delay_ms']:.1f} ms")
        print(f"[INFO] Velocity loss: {timing['velocity_loss']:.3f} m/s ({timing['velocity_loss_pct']:.1f}%)")

    print(f"[INFO] Plots saved to {save_dir}")


def save_results(results, path):
    """Serialize results dict to JSON (converts numpy arrays)."""
    os.makedirs(os.path.dirname(path), exist_ok=True)

    def _conv(o):
        if isinstance(o, np.ndarray):
            return o.tolist()
        raise TypeError(f"Not serializable: {type(o)}")

    with open(path, 'w') as f:
        json.dump(results, f, default=_conv, indent=2)
    print(f"[INFO] Results saved to {path}")


# ---------------------------------------------------------------------------
# Main workflow
# ---------------------------------------------------------------------------
def run_toss_comparison():
    print("=== TOSS FULL PIPELINE: spawn -> grasp -> toss + trajectory comparison ===")

    os.makedirs(LOG_DIR,  exist_ok=True)
    os.makedirs(PLOT_DIR, exist_ok=True)

    # --- Setup clients (all talk to container via ZMQ) ---
    robot   = RobotClient(protocol='zmq', host=CONTAINER_HOST)
    gripper = GripperClient(protocol='zmq', host=CONTAINER_HOST)
    gazebo  = GazeboClient(protocol='zmq', host=CONTAINER_HOST, model_paths=MODEL_PATHS)
    contact = ContactSensorClient(protocol='zmq', host=CONTAINER_HOST)

    urdf_path = os.path.abspath(URDF_PATH)
    pick_planner = CasadiPlanner(CasadiKinematics(urdf_path, "base", "right_gripper_tip"))

    # Move to neutral home
    print("[INFO] Moving to neutral home...")
    robot.move_to_joints(cfg.NEUTRAL_JOINT_POS)

    # ------------------------------------------------------------------
    # 1. Scene Reset
    # ------------------------------------------------------------------
    print("[INFO] Resetting scene...")
    gazebo.despawn(OBJECT_NAME)
    time.sleep(0.5)
    gazebo.spawn("cube", OBJECT_NAME, list(PICK_POS_WORLD))
    time.sleep(1.0)

    # ------------------------------------------------------------------
    # 2. Pick Sequence
    # ------------------------------------------------------------------
    pick_target  = to_robot_frame([PICK_POS_WORLD[0] - CUBE_HALF, PICK_POS_WORLD[1], PICK_POS_WORLD[2]])
    hover_target = list(pick_target)
    hover_target[2] += 0.05  # 5 cm above pick point

    print("[INFO] Opening gripper...")
    gripper.open()

    # A. Hover via IK + joint-space move
    print("[INFO] Moving to hover (IK -> joint)...")
    q_curr  = robot.get_joint_angles()
    q_hover = pick_planner.compute_inverse_kinematics(q_curr, hover_target, TARGET_QUAT)
    if q_hover is None:
        print("[ERROR] Hover IK failed - aborting.")
        return
    robot.move_to_joints(q_hover.tolist() if hasattr(q_hover, 'tolist') else list(q_hover))

    # B. Cartesian approach to pick height
    print("[INFO] Cartesian approach...")
    q_curr = robot.get_joint_angles()
    cart_plan = pick_planner.plan_cartesian(q_curr, pick_target, TARGET_QUAT, linear_speed=0.1)
    if cart_plan:
        # Use final waypoint as target
        robot.move_to_joints(cart_plan[-1]['position'])

    # C. Grasp
    print("[INFO] Grasping...")
    time.sleep(0.2)
    gripper.close()
    time.sleep(0.5)

    grip_state = gripper.get_state()
    if not grip_state.get('is_grasping', False):
        print("[ERROR] Grasp failed - aborting.")
        gripper.open()
        return

    # D. Lift
    print("[INFO] Lifting...")
    q_curr = robot.get_joint_angles()
    lift_plan = pick_planner.plan_cartesian(q_curr, hover_target, TARGET_QUAT, linear_speed=0.1)
    if lift_plan:
        robot.move_to_joints(lift_plan[-1]['position'])

    # ------------------------------------------------------------------
    # 3. Move to Toss-Ready Position
    # ------------------------------------------------------------------
    print("[INFO] Moving to toss-ready position...")
    robot.move_to_joints(cfg.TOSS_READY_POS)

    # ------------------------------------------------------------------
    # 4. Plan Toss
    # ------------------------------------------------------------------
    print(f"[INFO] Planning toss at {TOSS_SPEED} m/s...")
    q_curr   = np.array(robot.get_joint_angles())
    q0_3dof  = np.array([q_curr[1], q_curr[3], q_curr[5]])
    toss_planner = TossingPlanner(profile="express", angle_deg=45, q0=q0_3dof)

    sol_3d = toss_planner.get_trajectory(TOSS_SPEED)
    sol_7d = toss_planner.map_to_7dof(
        sol_3d["Q"], sol_3d["Qd"], sol_3d["Qdd"],
        q_curr[0], q_curr[2], q_curr[4], q_curr[6]
    )

    peak_idx      = sol_3d["index"]
    release_delay = max(0.0, (peak_idx - RELEASE_OFFSET  - 0) * 0.01)
    print(
        f"[INFO] Peak index: {peak_idx} | Offset: {RELEASE_OFFSET} | "
        f"Release delay: {release_delay:.3f}s"
    )

    # ------------------------------------------------------------------
    # 5. Execute Toss — records via PUB socket + contact sensor
    # ------------------------------------------------------------------
    print("[INFO] Executing toss with recording...")
    results = execute_toss_with_comparison(robot, gazebo, contact, sol_7d, release_delay)

    # ------------------------------------------------------------------
    # 6. Save + Plot
    # ------------------------------------------------------------------
    save_results(results, LOG_FILE)
    plot_results(results, PLOT_DIR)

    # Return to neutral
    print("[INFO] Returning to neutral...")
    robot.move_to_joints(cfg.NEUTRAL_JOINT_POS)

    # Summary
    obj_traj = results['obj_traj']
    print("=== TEST COMPLETE ===")
    if obj_traj:
        final = obj_traj[-1]['pos']
        print(f"  Object final pos: ({final[0]:.3f}, {final[1]:.3f}, {final[2]:.3f})")
    print(f"  PUB states:  {len(results['actual_states'])}")
    print(f"  Obj samples: {len(obj_traj)}")
    print(f"  Log:   {LOG_FILE}")
    print(f"  Plots: {PLOT_DIR}")

    # Cleanup
    robot.close()
    gripper.close()
    gazebo.close()
    contact.close()


if __name__ == "__main__":
    try:
        run_toss_comparison()
    except KeyboardInterrupt:
        print("\nAborted.")
