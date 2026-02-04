#!/usr/bin/env python3.8
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import numpy as np
import os
import sys

def animate_trial(trial, trial_idx, output_dir):
    speed = trial['speed']
    offset = trial['offset']
    traj = trial.get('trajectory', [])
    if not traj: return

    x = [p['pos'][0] for p in traj if 'pos' in p]
    z = [p['pos'][2] for p in traj if 'pos' in p]
    cmd_active = [p['cmd_active'] for p in traj if 'pos' in p]
    
    if not x: return

    fig, ax = plt.subplots(figsize=(10, 6))
    line, = ax.plot([], [], 'b-', lw=2)
    point, = ax.plot([], [], 'bo')
    release_marker, = ax.plot([], [], 'ro', markersize=10, label='Release Cmd')
    
    ax.set_xlim(min(x) - 0.1, max(x) + 0.1)
    ax.set_ylim(min(z) - 0.1, max(z) + 0.1)
    ax.set_aspect('equal')
    ax.grid(True, linestyle='--', alpha=0.7)
    ax.set_title(f"Trial {trial_idx}: Speed={speed}m/s, Offset={offset}")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Z (m)")

    cmd_idx = next((j for j, active in enumerate(cmd_active) if active), None)

    def init():
        line.set_data([], [])
        point.set_data([], [])
        release_marker.set_data([], [])
        return line, point, release_marker

    def update(frame):
        line.set_data(x[:frame], z[:frame])
        point.set_data(x[frame], z[frame])
        if cmd_idx is not None and frame >= cmd_idx:
            release_marker.set_data(x[cmd_idx], z[cmd_idx])
        return line, point, release_marker

    ani = FuncAnimation(fig, update, frames=len(x), init_func=init, blit=True)
    
    writer = PillowWriter(fps=20)
    save_path = os.path.join(output_dir, f"trial_{trial_idx}_anim.gif")
    ani.save(save_path, writer=writer)
    plt.close()
    print(f"  Saved animation: {save_path}")

def main(log_path):
    print(f"Generating animations from: {log_path}")
    with open(log_path, 'r') as f:
        data = json.load(f)
    
    output_dir = log_path.replace(".json", "_animations")
    os.makedirs(output_dir, exist_ok=True)

    for i, trial in enumerate(data):
        animate_trial(trial, i, output_dir)

if __name__ == "__main__":
    base_dir = os.path.dirname(os.path.abspath(__file__))
    log_dir = os.path.join(base_dir, "../../../logs/timing_test")
    
    if len(sys.argv) > 1:
        log_path = sys.argv[1]
    else:
        files = [os.path.join(log_dir, f) for f in os.listdir(log_dir) if f.endswith(".json")]
        if files:
            log_path = max(files, key=os.path.getctime)
        else:
            print("No logs found.")
            sys.exit(1)
            
    main(log_path)
