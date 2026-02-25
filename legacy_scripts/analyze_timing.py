#!/usr/bin/env python3.8
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import os
import sys

def analyze_timing_log(log_path):
    print(f"Analyzing: {log_path}")
    with open(log_path, 'r') as f:
        data = json.load(f)
        
    if not data:
        print("Empty log file.")
        return

    # Organize data by Speed
    # { 1.0: { offset: [loss1, loss2], ... }, 1.5: ... }
    results = {}
    
    for trial in data:
        s = trial['speed']
        o = trial['offset']
        
        if s not in results: results[s] = {}
        if o not in results[s]: 
            results[s][o] = {'loss': [], 'delay': [], 'flight': [], 'error': []}
            
        results[s][o]['loss'].append(trial['loss'])
        results[s][o]['delay'].append(trial['delay_ms'])
        results[s][o]['flight'].append(trial['flight_v'])
        if trial.get('error_mm') and trial['error_mm'] > 0:
            results[s][o]['error'].append(trial['error_mm'])

    # --- PLOTTING ---
    fig = plt.figure(figsize=(15, 18))
    
    # 1. Velocity Loss vs Offset
    ax1 = fig.add_subplot(4, 1, 1)
    colors = ['r', 'g', 'b', 'm']
    
    speeds = sorted(results.keys())
    
    for i, s in enumerate(speeds):
        offsets = sorted(results[s].keys())
        mean_loss = [np.mean(results[s][o]['loss']) for o in offsets]
        std_loss = [np.std(results[s][o]['loss']) for o in offsets]
        
        ax1.errorbar(offsets, mean_loss, yerr=std_loss, label=f"Speed {s} m/s", marker='o', capsize=5)
        
    ax1.set_title("Velocity Loss vs Release Offset")
    ax1.set_xlabel("Release Offset (steps before end)")
    ax1.set_ylabel("Velocity Loss (Grip - Flight) [m/s]")
    ax1.axhline(0, color='k', linestyle='--', alpha=0.5)
    ax1.grid(True)
    ax1.legend()
    ax1.invert_xaxis()
    
    # 2. Flight Speed Consistency
    ax2 = fig.add_subplot(4, 1, 2)
    
    for i, s in enumerate(speeds):
        offsets = sorted(results[s].keys())
        mean_flight = [np.mean(results[s][o]['flight']) for o in offsets]
        target_flight = [s for _ in offsets] # Ideal
        
        ax2.plot(offsets, mean_flight, marker='o', label=f"Actual {s}")
        ax2.plot(offsets, target_flight, linestyle='--', alpha=0.5, label=f"Target {s}")

    ax2.set_title("Actual Flight Speed vs Offset")
    ax2.set_xlabel("Release Offset (steps)")
    ax2.set_ylabel("Flight Speed (m/s)")
    ax2.grid(True)
    ax2.legend()
    ax2.invert_xaxis()

    # 3. Ballistic Error vs Offset
    ax3 = fig.add_subplot(4, 1, 3)
    for i, s in enumerate(speeds):
        offsets = sorted(results[s].keys())
        valid_offsets = [o for o in offsets if len(results[s][o]['error']) > 0]
        mean_err = [np.mean(results[s][o]['error']) for o in valid_offsets]
        std_err = [np.std(results[s][o]['error']) for o in valid_offsets]
        
        ax3.errorbar(valid_offsets, mean_err, yerr=std_err, label=f"Speed {s} m/s", marker='s', capsize=5)

    ax3.set_title("Ballistic Prediction Error vs Offset")
    ax3.set_xlabel("Release Offset (steps)")
    ax3.set_ylabel("Error (mm)")
    ax3.grid(True)
    ax3.legend()
    ax3.invert_xaxis()

    # 4. Top-Down Landing Dispersion
    ax4 = fig.add_subplot(4, 1, 4)
    
    actual_x = [t['landing']['x'] for t in data if t.get('landing') and t['landing']['success']]
    actual_y = [t['landing']['y'] for t in data if t.get('landing') and t['landing']['success']]
    
    if actual_x:
        ax4.scatter(actual_x, actual_y, c='blue', alpha=0.5, label='Actual Landing')
        
        # Plot theoretical target (assuming straight toss at center_y)
        # Note: In test_release_timing.py, ANGLE_DEG=0, so Y should be cfg.CENTER_Y
        # We'll mark the mean commanded target if we had it, but for timing test it's mostly fixed.
        ax4.set_title("Top-Down Landing Dispersion")
        ax4.set_xlabel("X (m)")
        ax4.set_ylabel("Y (m)")
        ax4.grid(True)
        ax4.axis('equal')
        ax4.legend()
    else:
        ax4.text(0.5, 0.5, "No Landing Data Available", ha='center')

    plot_path = log_path.replace(".json", "_analysis.png")
    plt.tight_layout()
    plt.savefig(plot_path)
    print(f"Analysis Plot saved to: {plot_path}")
    
    # Text Summary
    print("\n--- SUMMARY (Best Offsets) ---")
    for s in speeds:
        best_off = None
        min_loss = float('inf')
        
        for o in results[s]:
            avg_loss = abs(np.mean(results[s][o]['loss']))
            if avg_loss < min_loss:
                min_loss = avg_loss
                best_off = o
        
        print(f"Speed {s} m/s -> Best Offset: {best_off} (Loss: {min_loss:.3f} m/s)")

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
            print("No timing logs found.")
            sys.exit(1)
            
    analyze_timing_log(log_path)
