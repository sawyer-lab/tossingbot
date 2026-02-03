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
        
        pred_pos, tof, pred_traj_pts = get_ballistic_prediction(r_pos, r_vel)
        real_pos = np.array([trial['landing']['x'], trial['landing']['y']])
        
        # Track data
        item = {
            'cmd_speed': trial['config']['speed'],
            'cmd_angle': trial['config']['angle_deg'],
            'act_speed': r_speed,
            'actual_vel_vec': r_vel, # Added for side view prediction
            'real_pos': real_pos,
            'pred_pos': pred_pos,
            'error_ballistic': np.linalg.norm(real_pos - pred_pos) if pred_pos is not None else 0,
            'real_traj_pts': np.array([s['pos'] for s in traj[idx:]])
        }
        processed.append(item)
        
        # Save one sample per unique config for the comparison plot
        cfg_id = (trial['config']['speed'], trial['config']['angle_deg'])
        if cfg_id not in configs_seen and len(sample_trajectories) < 10:
            sample_trajectories.append({
                'real': item['real_traj_pts'],
                'pred': pred_traj_pts,
                'label': f"S:{item['cmd_speed']} A:{item['cmd_angle']}"
            })
            configs_seen.add(cfg_id)

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
        ax1.scatter(pred_xy[:,0], pred_xy[:,1], c='red', marker='x', alpha=0.4, label='Predicted Landing')
    ax1.set_title("Top-Down Landing Dispersion")
    ax1.set_xlabel("X (Forward)")
    ax1.set_ylabel("Y (Lateral)")
    ax1.grid(True)
    ax1.axis('equal')
    ax1.legend()

    # 2. Velocity Tracking
    ax2 = fig.add_subplot(2, 2, 2)
    cmds = [p['cmd_speed'] for p in processed]
    acts = [p['act_speed'] for p in processed]
    ax2.scatter(cmds, acts, alpha=0.5)
    lims = [min(cmds)-0.2, max(cmds)+0.2]
    ax2.plot(lims, lims, 'r--', label='Ideal')
    ax2.set_title("Commanded vs. Measured Release Speed")
    ax2.set_xlabel("Commanded (m/s)")
    ax2.set_ylabel("Actual (m/s)")
    ax2.legend()

    # 3. Side View (X-Z Plane) for J0=0
    ax3 = fig.add_subplot(2, 2, 3)
    colors = plt.cm.viridis(np.linspace(0, 1, 5))
    speed_color_map = {s: colors[i] for i, s in enumerate(sorted(list(set(cmds))))}
    
    j0_zero_count = 0
    for st in sample_trajectories:
        # We can also iterate through all processed data for this plot to see more density
        pass
        
    for p in processed:
        if abs(p['cmd_angle']) < 0.1: # Near zero J0
            c = speed_color_map.get(p['cmd_speed'], 'blue')
            # Actual (Dots)
            ax3.scatter(p['real_traj_pts'][:,0], p['real_traj_pts'][:,2], s=2, color=c, alpha=0.4)
            # Predicted (Line) - recalculate or use stored if added
            pred_pos, tof, pred_pts = get_ballistic_prediction(p['real_traj_pts'][0], p['actual_vel_vec'])
            if pred_pts is not None:
                ax3.plot(pred_pts[:,0], pred_pts[:,2], color=c, linewidth=2, alpha=0.8)
            j0_zero_count += 1
            
    ax3.set_title(f"Side View (X-Z Plane) for J0=0 ({j0_zero_count} trials)")
    ax3.set_xlabel("X (Forward) [m]")
    ax3.set_ylabel("Z (Height) [m]")
    ax3.grid(True)
    ax3.set_xlim(0.4, 1.4)
    ax3.set_ylim(0.6, 1.2) 
    
    # 4. Error Histogram
    ax4 = fig.add_subplot(2, 2, 4)
    err_ballistic = [p['error_ballistic'] for p in processed]
    ax4.hist(err_ballistic, bins=20, color='green', alpha=0.7)
    ax4.set_title("Ballistic Prediction Error Distribution")
    ax4.set_xlabel("Error (m)")
    ax4.set_ylabel("Frequency")

    plt.tight_layout()
    plot_path = log_path.replace(".json", "_v3_trajectory_plot.png")
    plt.savefig(plot_path)
    print(f"Comparison Analysis saved to: {plot_path}")

    # Summary Stats
    err_vals = np.array(err_ballistic)
    print("\n--- PERFORMANCE SUMMARY ---")
    print(f"Mean Speed Bias: {np.mean(np.array(acts) - np.array(cmds)):.3f} m/s")
    print(f"Mean Ballistic Error: {np.mean(err_vals)*1000:.2f} mm")
    print(f"Std Dev Error: {np.std(err_vals)*1000:.2f} mm")

if __name__ == "__main__":
    base_dir = os.path.dirname(os.path.abspath(__file__))
    log_dir = os.path.join(base_dir, "../../../logs/test_suite")
    files = [os.path.join(log_dir, f) for f in os.listdir(log_dir) if f.endswith(".json")]
    if files:
        latest = max(files, key=os.path.getctime)
        print(f"Analyzing latest: {latest}")
        analyze_log(latest)