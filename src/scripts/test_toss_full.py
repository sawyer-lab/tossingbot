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

from robot_client import RobotClient, GripperClient, GazeboClient
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
TOSS_SPEED     = 2.0            # m/s target release speed
RELEASE_OFFSET = 0              # Steps before peak index to open gripper
ROBOT_Z_OFFSET = 1.0

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
def execute_toss_with_comparison(robot, gazebo, sol_7d, release_delay):
    """
    Execute toss and collect all data needed for comparison plots.

    Returns a results dict:
      planned_q       (N, 7) numpy array
      planned_qd      (N, 7) numpy array
      release_index   int
      actual_states   list of dicts from PUB socket (aligned to trajectory)
      obj_traj        list of {t, pos} from Gazebo post-flight polling
    """
    Q, Qd, Qdd = sol_7d["Q"], sol_7d["Qd"], sol_7d["Qdd"]
    N = Q.shape[0]

    release_index = int(release_delay * 100)
    release_index = max(0, min(release_index, N - 1))
    print(f"[INFO] Release at step {release_index}/{N} "
          f"(t={release_index * 0.01:.3f}s)")

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
            obj_traj.append({
                't':   i * 0.01,
                'pos': gz[OBJECT_NAME]['position'],
            })

    # Short post-trajectory poll via REQ (REQ is free now) to capture settling
    # if the cube is still moving when the trajectory loop ended.
    t0_post = time.time()
    while time.time() - t0_post < 1.5:
        pose = gazebo.get_pose(OBJECT_NAME)
        if pose:
            obj_traj.append({
                't':   N * 0.01 + (time.time() - t0_post),
                'pos': pose['position'],
            })
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
        fig.suptitle("Cube vs End-Effector — X/Z plane\n(red dashed = release)")

        # Row 0: X position
        ax = axes[0]
        if N_act:
            ax.plot(t_act, act_eep[:, 0], 'b-',  lw=1.2, label='EE x',   alpha=0.8)
        ax.plot(t_obj, p_obj[:, 0],       'm-',  lw=1.5, label='Cube x')
        ax.axvline(t_rel, color='r', ls=':', lw=1.2, label='Release')
        ax.set_ylabel("x (m)")
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)

        # Row 1: Z position
        ax = axes[1]
        if N_act:
            ax.plot(t_act, act_eep[:, 2], 'b-',  lw=1.2, label='EE z',   alpha=0.8)
        ax.plot(t_obj, p_obj[:, 2],       'm-',  lw=1.5, label='Cube z')
        ax.axvline(t_rel, color='r', ls=':', lw=1.2, label='Release')
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

    # ---- 5. Ballistic prediction vs actual ----
    if N_act and rel_idx < N_act and obj_traj:
        p0 = act_eep[rel_idx]      # release position (EE ≈ object at release)
        v0 = act_eev[rel_idx]      # release velocity
        g  = 9.81

        t_b = np.linspace(0, 3.0, 600)
        xb  = p0[0] + v0[0] * t_b
        yb  = p0[1] + v0[1] * t_b
        zb  = p0[2] + v0[2] * t_b - 0.5 * g * t_b**2

        # Predicted landing (first z ≤ 0 after release)
        above = zb > 0
        if above.any() and not above.all():
            li = np.argmax(~above)
        else:
            li = -1
        pred_land = np.array([xb[li], yb[li]])
        actual_land = np.array([p_obj[-1, 0], p_obj[-1, 1]])
        err_m = np.linalg.norm(pred_land - actual_land)

        fig, axes = plt.subplots(1, 2, figsize=(14, 6))
        fig.suptitle(
            f"Ballistic Prediction vs Actual  |  "
            f"Landing error: {err_m*100:.1f} cm  |  "
            f"Release speed: {np.linalg.norm(v0):.2f} m/s"
        )

        # XZ side view
        ax = axes[0]
        ax.plot(xb, zb, 'b--', lw=1.5, label='Ballistic (predicted)')
        ax.plot(p_obj[:, 0], p_obj[:, 2], 'm-', lw=1.5, label='Actual')
        ax.scatter(p0[0], p0[2], c='r', zorder=5, s=60, label='Release')
        ax.scatter(pred_land[0],   0, c='b', zorder=5, s=60, marker='x',
                   label=f'Pred land ({pred_land[0]:.2f}, {pred_land[1]:.2f})')
        ax.scatter(actual_land[0], p_obj[-1, 2], c='m', zorder=5, s=60, marker='x',
                   label=f'Actual last ({actual_land[0]:.2f}, {actual_land[1]:.2f})')
        ax.set_xlabel("x (m)")
        ax.set_ylabel("z (m)")
        ax.set_title("Side view (X-Z)")
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

        # XY top view
        ax = axes[1]
        ax.plot(xb, yb, 'b--', lw=1.5, label='Ballistic (predicted)')
        ax.plot(p_obj[:, 0], p_obj[:, 1], 'm-', lw=1.5, label='Actual')
        ax.scatter(p0[0], p0[1], c='r', zorder=5, s=60, label='Release')
        ax.scatter(pred_land[0], pred_land[1], c='b', zorder=5, s=60, marker='x',
                   label='Pred landing')
        ax.scatter(actual_land[0], actual_land[1], c='m', zorder=5, s=60, marker='x',
                   label='Actual last pos')
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_title("Top view (X-Y)")
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig(os.path.join(save_dir, "ballistic.png"), dpi=150)
        plt.close()
        print(f"[INFO] Landing error: {err_m*100:.1f} cm")

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
    pick_target  = to_robot_frame(PICK_POS_WORLD)
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
    release_delay = max(0.0, (peak_idx - RELEASE_OFFSET  + 9) * 0.01)
    print(
        f"[INFO] Peak index: {peak_idx} | Offset: {RELEASE_OFFSET} | "
        f"Release delay: {release_delay:.3f}s"
    )

    # ------------------------------------------------------------------
    # 5. Execute Toss — records via PUB socket + post-flight Gazebo poll
    # ------------------------------------------------------------------
    print("[INFO] Executing toss with recording...")
    results = execute_toss_with_comparison(robot, gazebo, sol_7d, release_delay)

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


if __name__ == "__main__":
    try:
        run_toss_comparison()
    except KeyboardInterrupt:
        print("\nAborted.")
