#!/usr/bin/env python3.8
import json
import matplotlib
matplotlib.use('Agg') # Support headless environments
import matplotlib.pyplot as plt
import numpy as np
import sys
import os
from mpl_toolkits.mplot3d import Axes3D

# Constants
G = 9.806
TABLE_Z = 0.75  # Table surface height

def get_ballistic_prediction(release_pos, release_vel, duration=2.0, steps=100):
    """Calculates landing position and full trajectory based on projectile motion."""
    x0, y0, z0 = release_pos
    vx, vy, vz = release_vel
    
    # Time to hit table: Solve z0 + vz*t - 0.5*g*t^2 = TABLE_Z
    # 0.5*G*t^2 - vz*t + (TABLE_Z - z0) = 0
    a_q = 0.5 * G
    b_q = -vz
    c_q = TABLE_Z - z0
    
    discriminant = b_q**2 - 4*a_q*c_q
    if discriminant < 0: return None, 0, None
    
    tof = (vz + np.sqrt(vz**2 + 2*G*(z0 - TABLE_Z))) / G
    
    # Calculate full trajectory line for plotting
    t_steps = np.linspace(0, tof, steps)
    traj_x = x0 + vx * t_steps
    traj_y = y0 + vy * t_steps
    traj_z = z0 + vz * t_steps - 0.5 * G * t_steps**2
    
    pred_pos = np.array([traj_x[-1], traj_y[-1]])
    full_traj = np.stack([traj_x, traj_y, traj_z], axis=1)
    
    return pred_pos, tof, full_traj

def analyze_log(log_path):
    with open(log_path, 'r') as f:
        data = json.load(f)
        
    print(f"Analyzing {len(data)} trials...")
    
    processed = []
    configs_seen = set()
    sample_trajectories = [] # Store a few for the comparison plot

    for trial in data:
        if not trial['landing']['success'] or not trial['trajectory']: continue
        
        traj = trial['trajectory']
        # Use Peak Velocity as the release indicator (proved accurate in v1)
        vels = np.array([np.linalg.norm(s['vel']) for s in traj])
        idx = np.argmax(vels)
        
        r_pos = traj[idx]['pos']
        r_vel = traj[idx]['vel']
        r_speed = vels[idx]
        
    processed = []
    velocity_profiles = []

    for trial in data:
        if not trial['landing']['success'] or not trial['trajectory']: continue
        
        traj = trial['trajectory']
        
        # Determine Release Index
        # Newer logs have 'release_idx', older ones don't. Fallback to argmax or middle.
        if 'release_idx' in trial:
            rel_idx = trial['release_idx']
        else:
            # Fallback: Peak velocity is usually impact, but we want release.
            # Assuming release is roughly 0.5s before end?
            # Or just use argmax of velocity as a proxy for "high energy event"
            vels = np.array([np.linalg.norm(s['vel']) for s in traj])
            # This is flawed as discussed, but best effort for legacy logs
            rel_idx = np.argmax(vels) 
            
        # Extract Velocity Profile ( Window: -0.1s to +0.2s around release)
        # 1 step = 0.01s (approx, usually 100Hz recording)
        start_i = max(0, rel_idx - 10)
        end_i = min(len(traj), rel_idx + 20)
        
        window_vels = []
        window_times = []
        t0 = traj[rel_idx]['t']
        
        for i in range(start_i, end_i):
            v_mag = np.linalg.norm(traj[i]['vel'])
            t_rel = traj[i]['t'] - t0
            window_vels.append(v_mag)
            window_times.append(t_rel)
            
        velocity_profiles.append({
            'times': window_times,
            'vels': window_vels,
            'cmd_speed': trial['config']['speed']
        })

        # Use FLIGHT speed (rel_idx + 5) for analysis if available
        flight_idx = min(rel_idx + 5, len(traj) - 1)
        
        r_pos = traj[flight_idx]['pos']
        r_vel = traj[flight_idx]['vel']
        r_speed = np.linalg.norm(r_vel)
        
        real_pos = np.array([trial['landing']['x'], trial['landing']['y']])
        
        # Physics Prediction (Flight State)
        pred_pos, _, _ = get_ballistic_prediction(r_pos, r_vel)
        
        # Commanded Prediction (recalc)
        cmd_speed = trial['config']['speed']
        j0 = np.deg2rad(trial['config']['angle_deg'])
        vx_p = cmd_speed * np.cos(np.deg2rad(45))
        vz_p = cmd_speed * np.sin(np.deg2rad(45))
        px0 = 0.825 * np.cos(j0) - 0.1363 * np.sin(j0)
        py0 = 0.825 * np.sin(j0) + 0.1363 * np.cos(j0)
        vx0 = vx_p * np.cos(j0)
        vy0 = vx_p * np.sin(j0)
        
        pred_cmd_pos, _, _ = get_ballistic_prediction([px0, py0, 1.0], [vx0, vy0, vz_p])

        # Track data
        item = {
            'cmd_speed': cmd_speed,
            'act_speed': r_speed, # Flight speed
            'real_pos': real_pos,
            'pred_pos': pred_pos,
            'pred_cmd_pos': pred_cmd_pos,
            'error_ballistic': np.linalg.norm(real_pos - pred_pos) if pred_pos is not None else 0,
            'error_commanded': np.linalg.norm(real_pos - pred_cmd_pos) if pred_cmd_pos is not None else 0
        }
        processed.append(item)

    if not processed:
        print("No valid trials found.")
        return

    # --- PLOTTING ---
    fig = plt.figure(figsize=(18, 12))
    
    # 1. Top-Down Landing Dispersion
    ax1 = fig.add_subplot(2, 2, 1)
    real_xy = np.array([p['real_pos'] for p in processed])
    ax1.scatter(real_xy[:,0], real_xy[:,1], c='blue', alpha=0.4, label='Actual Landing')
    
    pred_xy = np.array([p['pred_pos'] for p in processed if p['pred_pos'] is not None])
    if len(pred_xy) > 0:
        ax1.scatter(pred_xy[:,0], pred_xy[:,1], c='red', marker='x', alpha=0.4, label='Physics Pred')
        
    cmd_xy = np.array([p['pred_cmd_pos'] for p in processed if p['pred_cmd_pos'] is not None])
    if len(cmd_xy) > 0:
        ax1.scatter(cmd_xy[:,0], cmd_xy[:,1], c='green', marker='o', facecolors='none', alpha=0.6, label='Commanded Pred')
        
    ax1.set_title("Top-Down Landing Dispersion")
    ax1.set_xlabel("X (Forward)")
    ax1.set_ylabel("Y (Lateral)")
    ax1.grid(True)
    ax1.axis('equal')
    ax1.legend()

    # 2. Release Velocity Profile
    ax2 = fig.add_subplot(2, 2, 2)
    colors = plt.cm.viridis(np.linspace(0, 1, 5))
    speed_map = {s: colors[i] for i, s in enumerate(sorted(list(set([p['cmd_speed'] for p in processed]))))}
    
    for vp in velocity_profiles:
        c = speed_map.get(vp['cmd_speed'], 'blue')
        ax2.plot(vp['times'], vp['vels'], color=c, alpha=0.3)
        
    ax2.axvline(0, color='k', linestyle='--', label='Release Cmd')
    ax2.set_title("Release Velocity Profile (-0.1s to +0.2s)")
    ax2.set_xlabel("Time from Release (s)")
    ax2.set_ylabel("Object Speed (m/s)")
    ax2.grid(True)

    # 3. Commanded Error Histogram (Control)
    ax3 = fig.add_subplot(2, 2, 3)
    err_cmd = [p['error_commanded'] for p in processed]
    ax3.hist(err_cmd, bins=20, color='orange', alpha=0.7)
    ax3.set_title("Commanded Prediction Error (Control)")
    ax3.set_xlabel("Error (m)")
    ax3.set_ylabel("Frequency")
    
    # 4. Ballistic Error Histogram (Physics)
    ax4 = fig.add_subplot(2, 2, 4)
    err_ballistic = [p['error_ballistic'] for p in processed]
    ax4.hist(err_ballistic, bins=20, color='green', alpha=0.7)
    ax4.set_title("Ballistic Prediction Error (Physics)")
    ax4.set_xlabel("Error (m)")
    ax4.set_ylabel("Frequency")

    plt.tight_layout()
    plot_path = log_path.replace(".json", "_v3_trajectory_plot.png")
    plt.savefig(plot_path)
    print(f"Comparison Analysis saved to: {plot_path}")

    # Summary Stats
    err_vals = np.array(err_ballistic)
    cmd_err_vals = np.array(err_cmd)
    
    cmds = [p['cmd_speed'] for p in processed]
    acts = [p['act_speed'] for p in processed]

    print("\n--- PERFORMANCE SUMMARY ---")
    print(f"Mean Flight Speed Bias: {np.mean(np.array(acts) - np.array(cmds)):.3f} m/s")
    print(f"Mean Ballistic Error (Physics): {np.mean(err_vals)*1000:.2f} mm")
    print(f"Mean Commanded Error (Control): {np.mean(cmd_err_vals)*1000:.2f} mm")
    print(f"Std Dev Error: {np.std(err_vals)*1000:.2f} mm")

if __name__ == "__main__":
    base_dir = os.path.dirname(os.path.abspath(__file__))
    log_dir = os.path.join(base_dir, "../../../logs/test_suite")
    files = [os.path.join(log_dir, f) for f in os.listdir(log_dir) if f.endswith(".json")]
    if files:
        latest = max(files, key=os.path.getctime)
        print(f"Analyzing latest: {latest}")
        analyze_log(latest)