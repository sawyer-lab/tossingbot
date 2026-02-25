#!/usr/bin/env python3.8
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import os
import sys

# Constants
G = 9.806
TABLE_Z = 0.75

def get_ballistic_prediction(release_pos, release_vel, steps=100):
    """Calculates full trajectory based on projectile motion until it hits TABLE_Z."""
    x0, y0, z0 = release_pos
    vx, vy, vz = release_vel
    
    # Time to hit table: Solve z0 + vz*t - 0.5*g*t^2 = TABLE_Z
    a_q = 0.5 * G
    b_q = -vz
    c_q = TABLE_Z - z0
    
    discriminant = b_q**2 - 4*a_q*c_q
    if discriminant < 0: return None
    
    tof = (vz + np.sqrt(vz**2 + 2*G*(z0 - TABLE_Z))) / G
    
    t_steps = np.linspace(0, tof, steps)
    traj_x = x0 + vx * t_steps
    traj_y = y0 + vy * t_steps
    traj_z = z0 + vz * t_steps - 0.5 * G * t_steps**2
    
    # Also return constant velocity magnitude for comparison
    v_mag_arr = np.full_like(t_steps, np.linalg.norm([vx, vy, vz]))
    
    return t_steps, traj_x, traj_y, traj_z, v_mag_arr

def visualize_trajectory(log_path):
    print(f"Visualizing Trajectory from: {log_path}")
    with open(log_path, 'r') as f:
        data = json.load(f)
        
    if not data:
        print("Empty log file.")
        return

    # Create directory for trajectory plots
    plot_dir = log_path.replace(".json", "_plots")
    os.makedirs(plot_dir, exist_ok=True)

    for i, trial in enumerate(data):
        speed = trial['speed']
        offset = trial['offset']
        traj = trial.get('trajectory', [])
        
        if not traj:
            continue
            
        # Extract data
        x = [p['pos'][0] for p in traj if 'pos' in p]
        y = [p['pos'][1] for p in traj if 'pos' in p]
        z = [p['pos'][2] for p in traj if 'pos' in p]
        vx_vals = [p['vel'][0] for p in traj if 'vel' in p]
        vz_vals = [p['vel'][2] for p in traj if 'vel' in p]
        v_mag_vals = [np.linalg.norm(p['vel']) for p in traj if 'vel' in p]
        t_vals = [p['t'] for p in traj if 'pos' in p]
        cmd_active = [p['cmd_active'] for p in traj if 'pos' in p]
        
        if not x: continue
        
        # Relative time
        t_arr = np.array(t_vals) - t_vals[0]
        
        fig, (ax1, ax2, ax3, ax4, ax5) = plt.subplots(5, 1, figsize=(10, 20))
        
        # Find release and stop indices
        cmd_idx = next((j for j, active in enumerate(cmd_active) if active), None)
        stop_idx = trial.get('stop_idx')
        motion_idx = None
        if cmd_idx is not None:
            w_start = traj[cmd_idx]['grip_width']
            motion_idx = next((j for j in range(cmd_idx, len(traj)) 
                               if abs(traj[j]['grip_width'] - w_start) > 0.001), None)

        # Commanded Ideal Path (Fixed Planner Reference)
        c_speed = speed 
        c_angle = np.deg2rad(45.0)
        c_vx_ideal = c_speed * np.cos(c_angle)
        c_vz_ideal = c_speed * np.sin(c_angle)
        
        c_pos_world = [0.7, 0.0, 1.0] 
        res_cmd = get_ballistic_prediction(c_pos_world, [c_vx_ideal, 0, c_vz_ideal])
        
        # --- Plot 1: XZ Trajectory ---
        ax1.plot(x, z, 'b-', linewidth=2, label='Actual Path')
        res_actual = None
        if motion_idx is not None:
            r_pos = traj[motion_idx]['pos']
            r_vel = traj[motion_idx]['vel']
            res_actual = get_ballistic_prediction(r_pos, r_vel)
            if res_actual:
                at_t, ax_x, ay_y, az_z, av_v = res_actual
                ax1.plot(ax_x, az_z, 'k--', alpha=0.6, label='Ideal Ballistic (Actual)')
        if res_cmd:
             cit, cix, ciy, ciz, civ = res_cmd
             ax1.plot(cix, ciz, 'b:', linewidth=2, label='Ideal Command (Fixed)')

        if cmd_idx is not None:
            ax1.plot(x[cmd_idx], z[cmd_idx], 'ro', markersize=10, label='Release Cmd', zorder=5)
            if motion_idx:
                ax1.plot(x[motion_idx], z[motion_idx], 'go', markersize=8, label='Actual Release', zorder=6)
        if stop_idx is not None and stop_idx < len(x):
            ax1.plot(x[stop_idx], z[stop_idx], 'orange', marker='x', markersize=10, label='Decel Start', zorder=7)

        ax1.set_title(f"Trial {i} ({trial.get('mode', 'N/A')}): XZ Trajectory")
        ax1.set_xlabel("X (m)"); ax1.set_ylabel("Z (m)"); ax1.legend(prop={'size': 8}); ax1.grid(True, alpha=0.3); ax1.set_aspect('equal')
        
        # --- Plot 2: Z vs Time ---
        ax2.plot(t_arr, z, 'g-', linewidth=2, label='Actual Z')
        if motion_idx is not None and res_actual:
            ax2.plot(at_t + t_arr[motion_idx], az_z, 'k--', alpha=0.6, label='Ideal Z')
        if cmd_idx is not None:
            ax2.axvline(t_arr[cmd_idx], color='r', linestyle='--', label='Release Cmd')
            if motion_idx: ax2.axvline(t_arr[motion_idx], color='g', linestyle='--')
        if stop_idx is not None and stop_idx < len(t_arr):
            ax2.axvline(t_arr[stop_idx], color='orange', linestyle='--', label='Decel Start')
        ax2.set_title("Vertical Position vs Time"); ax2.set_ylabel("Height Z (m)"); ax2.legend(prop={'size': 8}); ax2.grid(True, alpha=0.3)

        # --- Plot 3: Velocity Magnitude vs Time ---
        ax3.plot(t_arr[:len(v_mag_vals)], v_mag_vals, 'm-', linewidth=2, label='Actual Speed')
        ax3.axhline(speed, color='b', linestyle=':', linewidth=2, label='Target Speed')
        if cmd_idx is not None:
            ax3.axvline(t_arr[cmd_idx], color='r', linestyle='--')
            if motion_idx: ax3.axvline(t_arr[motion_idx], color='g', linestyle='--')
        if stop_idx is not None and stop_idx < len(t_arr):
            ax3.axvline(t_arr[stop_idx], color='orange', linestyle='--')
        ax3.set_title("Velocity Magnitude vs Time"); ax3.set_ylabel("Speed (m/s)"); ax3.legend(prop={'size': 8}); ax3.grid(True, alpha=0.3)

        # --- Plot 4: Vx vs Time ---
        ax4.plot(t_arr[:len(vx_vals)], vx_vals, 'c-', linewidth=2, label='Actual Vx')
        ax4.axhline(c_vx_ideal, color='b', linestyle=':', linewidth=2, label='Target Vx')
        if cmd_idx is not None:
            ax4.axvline(t_arr[cmd_idx], color='r', linestyle='--')
            if motion_idx: ax4.axvline(t_arr[motion_idx], color='g', linestyle='--')
        if stop_idx is not None and stop_idx < len(t_arr):
            ax4.axvline(t_arr[stop_idx], color='orange', linestyle='--')
        ax4.set_title("Horizontal Velocity (Vx) vs Time"); ax4.set_ylabel("Vx (m/s)"); ax4.legend(prop={'size': 8}); ax4.grid(True, alpha=0.3)

        # --- Plot 5: Vz vs Time ---
        ax5.plot(t_arr[:len(vz_vals)], vz_vals, 'y-', linewidth=2, label='Actual Vz')
        ax5.axhline(c_vz_ideal, color='b', linestyle=':', linewidth=2, label='Target Vz')
        if cmd_idx is not None:
            ax5.axvline(t_arr[cmd_idx], color='r', linestyle='--')
            if motion_idx: ax5.axvline(t_arr[motion_idx], color='g', linestyle='--')
        if stop_idx is not None and stop_idx < len(t_arr):
            ax5.axvline(t_arr[stop_idx], color='orange', linestyle='--')
        ax5.set_title("Vertical Velocity (Vz) vs Time"); ax5.set_xlabel("Time (s)"); ax5.set_ylabel("Vz (m/s)"); ax5.legend(prop={'size': 8}); ax5.grid(True, alpha=0.3)

        plt.tight_layout()
        plot_file = os.path.join(plot_dir, f"trial_{i}_s{speed}_o{offset}.png")
        plt.savefig(plot_file)
        plt.close(fig)

    print(f"Trajectory plots saved to: {plot_dir}")

if __name__ == "__main__":
    log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../../logs/timing_test")
    if len(sys.argv) > 1:
        log_path = sys.argv[1]
    else:
        files = [os.path.join(log_dir, f) for f in os.listdir(log_dir) if f.endswith(".json")]
        if files: log_path = max(files, key=os.path.getctime)
        else: sys.exit(1)
            
    visualize_trajectory(log_path)