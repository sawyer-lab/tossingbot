#!/usr/bin/env python3
"""
toss_analysis.py — Analysis and plotting layer for toss experiments.

Provides:
  - get_ballistic_landing: projectile-motion landing prediction
  - analyze_release_timing: gripper delay and velocity-loss analysis
  - plot_results: generates all comparison plots from a results dict
  - save_results: serialises results dict to JSON
"""
import os
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


# ---------------------------------------------------------------------------
# Physics helpers
# ---------------------------------------------------------------------------

def get_ballistic_landing(
    release_pos_robot, release_vel_world,
    robot_z_offset=1.0, table_z=0.75,
    cube_half=0.0175, g=9.806,
):
    """
    Calculate landing position using projectile motion physics.

    Accounts for coordinate frames:
    - Release position given in ROBOT frame (planner coordinates)
    - Converted to WORLD frame using robot_z_offset
    - Landing plane is table surface + cube half-height (where cube center lands)

    Solves: z(t) = z0 + vz*t - 0.5*g*t^2 = landing_z
    Using quadratic formula to find time-of-flight.

    Args:
        release_pos_robot: [x, y, z] release position in ROBOT frame (m)
        release_vel_world: [vx, vy, vz] release velocity in WORLD frame (m/s)
        robot_z_offset: Robot base height in world frame (default 1.0 m)
        table_z: Table surface height in world frame (default 0.75 m)
        cube_half: Cube half-height to add to landing plane (default 0.0175 m)
        g: Gravitational acceleration (m/s²)

    Returns:
        tuple: (landing_position, tof, trajectory) or (None, None, None)
            landing_position: [x, y] numpy array in WORLD frame
            tof: time of flight (seconds)
            trajectory: dict with 't', 'x', 'y', 'z' arrays (WORLD frame)
    """
    x0 = release_pos_robot[0]
    y0 = release_pos_robot[1]
    z0 = release_pos_robot[2] + robot_z_offset

    vx, vy, vz = release_vel_world

    landing_z = table_z + cube_half

    # 0.5*g*t^2 - vz*t + (landing_z - z0) = 0
    a = 0.5 * g
    b = -vz
    c = landing_z - z0

    discriminant = b**2 - 4 * a * c
    if discriminant < 0:
        return None, None, None

    tof = (vz + np.sqrt(discriminant)) / g
    if tof <= 0:
        return None, None, None

    land_x = x0 + vx * tof
    land_y = y0 + vy * tof

    t_traj = np.linspace(0, tof, 300)
    trajectory = {
        't': t_traj,
        'x': x0 + vx * t_traj,
        'y': y0 + vy * t_traj,
        'z': z0 + vz * t_traj - 0.5 * g * t_traj**2,
    }

    return np.array([land_x, land_y]), tof, trajectory


def analyze_release_timing(results, object_name):
    """
    Analyze gripper release timing and velocity loss.

    Args:
        results: Results dict from execute_toss_with_comparison
        object_name: Name of thrown object in Gazebo

    Returns:
        dict with keys: release_commanded_idx, release_actual_idx,
        delay_ms, velocity_at_command, velocity_at_release,
        velocity_loss, velocity_loss_pct, obj_velocities
        or None if insufficient data.
    """
    states  = results['actual_states']
    rel_idx = results['release_index']

    if rel_idx >= len(states):
        return None

    obj_vels = []
    for i, s in enumerate(states):
        gz = s.get('gazebo', {})
        if object_name in gz and 'velocity' in gz[object_name]:
            vel = np.array(gz[object_name]['velocity'])
            obj_vels.append({'idx': i, 'vel': vel, 'speed': np.linalg.norm(vel)})

    if len(obj_vels) < rel_idx + 10:
        return None

    cmd_data = next((v for v in obj_vels if v['idx'] == rel_idx), None)
    if not cmd_data:
        return None

    vel_at_cmd = cmd_data['speed']

    actual_rel_idx = rel_idx
    for v in obj_vels:
        if v['idx'] > rel_idx and v['speed'] < vel_at_cmd * 0.95:
            actual_rel_idx = v['idx']
            break

    actual_data   = next((v for v in obj_vels if v['idx'] == actual_rel_idx), cmd_data)
    vel_at_release = actual_data['speed']
    delay_s        = (actual_rel_idx - rel_idx) * 0.01  # 100 Hz

    return {
        'release_commanded_idx': rel_idx,
        'release_actual_idx':    actual_rel_idx,
        'delay_ms':              delay_s * 1000,
        'velocity_at_command':   vel_at_cmd,
        'velocity_at_release':   vel_at_release,
        'velocity_loss':         vel_at_cmd - vel_at_release,
        'velocity_loss_pct':     ((vel_at_cmd - vel_at_release) / vel_at_cmd * 100)
                                 if vel_at_cmd > 0 else 0,
        'obj_velocities':        obj_vels,
    }


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _extract_actual(states):
    """Pull numpy arrays from PUB socket state list."""
    if not states:
        return None, None, None, None
    try:
        q   = np.array([s['robot']['joint_angles']                for s in states])
        qd  = np.array([s['robot']['joint_velocities']            for s in states])
        eep = np.array([s['robot']['endpoint_pose']['position']   for s in states])
        eev = np.array([s['robot']['endpoint_velocity']['linear'] for s in states])
        return q, qd, eep, eev
    except (KeyError, TypeError):
        return None, None, None, None


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def plot_results(
    results, save_dir,
    object_name="toss_cube",
    toss_speed=2.7,
    robot_z_offset=1.0,
    table_height=0.75,
    cube_half=0.0175,
):
    """
    Generate all comparison plots from a results dict and save to save_dir.

    Plots produced:
      1. joint_positions.png   — planned vs actual for J1, J3, J5
      2. joint_velocities.png  — planned vs actual for J1, J3, J5
      3. ee_position_speed.png — EE x/z + Vx/Vz + speed over time
      4. cube_vs_ee.png        — cube position/speed vs EE (post-release)
      5. ballistic.png         — predicted ballistic arc vs actual cube trajectory
      6. release_timing.png    — gripper delay and velocity loss
    """
    os.makedirs(save_dir, exist_ok=True)

    Q        = np.array(results['planned_q'])
    Qd       = np.array(results['planned_qd'])
    rel_idx  = results['release_index']
    states   = results['actual_states']
    obj_traj = results['obj_traj']

    N_plan = len(Q)
    t_plan = np.arange(N_plan) * 0.01
    t_rel  = rel_idx * 0.01

    act_q, act_qd, act_eep, act_eev = _extract_actual(states)
    N_act = len(act_q) if act_q is not None else 0
    t_act = np.arange(N_act) * 0.01

    JOINTS = [1, 3, 5]

    # ---- 1. Joint Positions: planned vs actual ----
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle("Joint Positions: Planned vs Actual")
    for ax, ji in zip(axes, JOINTS):
        ax.plot(t_plan, Q[:, ji], 'b-', lw=1.5, label='Planned')
        if N_act:
            ax.plot(t_act, act_q[:, ji], color='orange', ls='--', lw=1.5, label='Actual')
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
        ax.plot(t_plan, Qd[:, ji], 'b-', lw=1.5, label='Planned')
        if N_act:
            ax.plot(t_act, act_qd[:, ji], color='orange', ls='--', lw=1.5, label='Actual')
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

        if len(p_obj) > 1:
            dp     = np.diff(p_obj, axis=0)
            dt_obj = np.diff(t_obj)[:, None]
            v_obj  = dp / dt_obj
            t_vobj = (t_obj[:-1] + t_obj[1:]) / 2
        else:
            v_obj, t_vobj = np.zeros((0, 3)), np.array([])

        fig, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=True)
        fig.suptitle("Cube vs End-Effector — X/Z plane (World Frame)\n(red dashed = release)")

        ax = axes[0]
        if N_act:
            ax.plot(t_act, act_eep[:, 0], 'b-', lw=1.2, label='EE x', alpha=0.8)
        ax.plot(t_obj, p_obj[:, 0], 'm-', lw=1.5, label='Cube x')
        ax.axvline(t_rel, color='r', ls=':', lw=1.2, label='Release')
        ax.set_ylabel("x (m)")
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)

        ax = axes[1]
        if N_act:
            ax.plot(t_act, act_eep[:, 2] + robot_z_offset, 'b-', lw=1.2,
                    label='EE z', alpha=0.8)
        ax.plot(t_obj, p_obj[:, 2], 'm-', lw=1.5, label='Cube z')
        ax.axvline(t_rel, color='r', ls=':', lw=1.2, label='Release')
        ax.axhline(table_height, color='gray', ls=':', lw=1, alpha=0.5,
                   label=f'Table ({table_height}m)')
        ax.set_ylabel("z (m)")
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)

        ax = axes[2]
        if N_act:
            ax.plot(t_act, act_eev[:, 0], 'b-', lw=1.2, label='EE Vx', alpha=0.8)
        if len(v_obj):
            ax.plot(t_vobj, v_obj[:, 0], 'm-', lw=1.5, label='Cube Vx')
        ax.axvline(t_rel, color='r', ls=':', lw=1.2, label='Release')
        ax.set_ylabel("Vx (m/s)")
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)

        ax = axes[3]
        if N_act:
            ax.plot(t_act, act_eev[:, 2], 'b-', lw=1.2, label='EE Vz', alpha=0.8)
        if len(v_obj):
            ax.plot(t_vobj, v_obj[:, 2], 'm-', lw=1.5, label='Cube Vz')
        ax.axvline(t_rel, color='r', ls=':', lw=1.2, label='Release')
        ax.set_ylabel("Vz (m/s)")
        ax.set_xlabel("Time from trajectory start (s)")
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig(os.path.join(save_dir, "cube_vs_ee.png"), dpi=150)
        plt.close()

    # ---- 5. Ballistic prediction vs actual trajectory ----
    if obj_traj and N_act and rel_idx < N_act:
        print("[DEBUG] Creating ballistic plot...")
        try:
            p0_robot  = act_eep[rel_idx]
            v0_actual = act_eev[rel_idx]

            pred_actual, tof_actual, traj_actual = get_ballistic_landing(
                p0_robot, v0_actual,
                robot_z_offset=robot_z_offset,
                table_z=table_height,
                cube_half=cube_half,
            )

            speed_ideal = toss_speed
            angle_rad   = np.radians(45)
            v0_ideal    = np.array([
                speed_ideal * np.cos(angle_rad),
                0.0,
                speed_ideal * np.sin(angle_rad),
            ])
            pred_ideal, tof_ideal, traj_ideal = get_ballistic_landing(
                p0_robot, v0_ideal,
                robot_z_offset=robot_z_offset,
                table_z=table_height,
                cube_half=cube_half,
            )

            actual_land = np.array([p_obj[-1, 0], p_obj[-1, 1]])
            land_z      = p_obj[-1, 2]

            p0_world    = p0_robot.copy()
            p0_world[2] += robot_z_offset

            fig, axes = plt.subplots(1, 2, figsize=(15, 7))

            title_lines = [
                "Ballistic Predictions vs Actual Trajectory",
                f"Release: [{p0_world[0]:.2f}, {p0_world[1]:.2f}, {p0_world[2]:.2f}] m",
            ]
            if pred_ideal is not None:
                err_ideal = np.linalg.norm(pred_ideal - actual_land) * 100
                title_lines.append(
                    f"Ideal ({toss_speed} m/s @ 45°): TOF={tof_ideal:.3f}s, "
                    f"error={err_ideal:.1f}cm"
                )
            if pred_actual is not None:
                err_actual = np.linalg.norm(pred_actual - actual_land) * 100
                title_lines.append(
                    f"Actual ({np.linalg.norm(v0_actual):.2f} m/s): "
                    f"TOF={tof_actual:.3f}s, error={err_actual:.1f}cm"
                )

            fig.suptitle('\n'.join(title_lines), fontsize=10)

            # XZ side view
            ax = axes[0]
            if traj_ideal:
                ax.plot(traj_ideal['x'], traj_ideal['z'], 'b--', lw=2, alpha=0.7,
                        label=f'Ideal ({toss_speed} m/s @ 45°)')
            if traj_actual:
                ax.plot(traj_actual['x'], traj_actual['z'], 'g-.', lw=2, alpha=0.7,
                        label=f'Actual ({np.linalg.norm(v0_actual):.2f} m/s)')
            ax.plot(p_obj[:, 0], p_obj[:, 2], 'm-', lw=2.5, label='Gazebo trajectory')
            ax.scatter(p0_world[0], p0_world[2], c='red', zorder=5, s=100,
                       marker='o', edgecolors='black', linewidths=1.5, label='Release')
            if pred_ideal is not None:
                ax.scatter(pred_ideal[0], table_height + cube_half, c='blue',
                           zorder=5, s=90, marker='x', linewidths=2, label='Ideal landing')
            if pred_actual is not None:
                ax.scatter(pred_actual[0], table_height + cube_half, c='green',
                           zorder=5, s=90, marker='+', linewidths=2, label='Actual landing')
            ax.scatter(actual_land[0], land_z, c='cyan', zorder=5, s=120,
                       marker='*', edgecolors='black', linewidths=1.5, label='Measured')
            ax.axhline(table_height, color='gray', ls=':', lw=1.5, alpha=0.6, label='Table')
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
            ax.scatter(p0_world[0], p0_world[1], c='red', zorder=5, s=100,
                       marker='o', edgecolors='black', linewidths=1.5, label='Release')
            if pred_ideal is not None:
                ax.scatter(pred_ideal[0], pred_ideal[1], c='blue', zorder=5,
                           s=90, marker='x', linewidths=2, label='Ideal')
            if pred_actual is not None:
                ax.scatter(pred_actual[0], pred_actual[1], c='green', zorder=5,
                           s=90, marker='+', linewidths=2, label='Actual')
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
        print("[DEBUG] Skipping ballistic plot")

    # ---- 6. Release Timing and Velocity Loss ----
    timing = analyze_release_timing(results, object_name)
    if timing and N_act:
        fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
        fig.suptitle(
            f"Release Timing Analysis\n"
            f"Delay: {timing['delay_ms']:.1f} ms  |  "
            f"Velocity loss: {timing['velocity_loss']:.3f} m/s "
            f"({timing['velocity_loss_pct']:.1f}%)"
        )

        ax = axes[0]
        obj_vels = timing['obj_velocities']
        t_vel    = np.array([v['idx'] * 0.01 for v in obj_vels])
        speeds   = np.array([v['speed'] for v in obj_vels])

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

        ax = axes[1]
        zoom_start = max(0, rel_idx - 10)
        zoom_end   = min(len(t_vel), timing['release_actual_idx'] + 20)
        zoom_mask  = (t_vel >= zoom_start * 0.01) & (t_vel <= zoom_end * 0.01)

        ax.plot(t_vel[zoom_mask], speeds[zoom_mask], 'g-', lw=2,
                label='Object speed (zoomed)')
        ax.axvline(rel_idx * 0.01, color='orange', ls='--', lw=1.5, label='Command')
        ax.axvline(timing['release_actual_idx'] * 0.01, color='red', ls='--', lw=1.5,
                   label='Release')
        ax.fill_betweenx(
            [speeds[zoom_mask].min(), speeds[zoom_mask].max()],
            rel_idx * 0.01, timing['release_actual_idx'] * 0.01,
            alpha=0.2, color='yellow', label=f'Delay: {timing["delay_ms"]:.1f} ms',
        )
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Speed (m/s)")
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.set_title("Release Region (Zoomed)")

        plt.tight_layout()
        plt.savefig(os.path.join(save_dir, "release_timing.png"), dpi=150)
        plt.close()
        print(f"[INFO] Release delay: {timing['delay_ms']:.1f} ms")
        print(f"[INFO] Velocity loss: {timing['velocity_loss']:.3f} m/s "
              f"({timing['velocity_loss_pct']:.1f}%)")

    print(f"[INFO] Plots saved to {save_dir}")


# ---------------------------------------------------------------------------
# Serialisation
# ---------------------------------------------------------------------------

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
