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
            results[s][o] = {'loss': [], 'delay': [], 'flight': []}
            
        results[s][o]['loss'].append(trial['loss'])
        results[s][o]['delay'].append(trial['delay_ms'])
        results[s][o]['flight'].append(trial['flight_v'])

    # --- PLOTTING ---
    fig = plt.figure(figsize=(12, 10))
    
    # 1. Velocity Loss vs Offset
    ax1 = fig.add_subplot(2, 1, 1)
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
    ax1.invert_xaxis() # Show larger offsets (earlier) on left? Or keep standard? 
    # Standard: 4 (Late) -> 20 (Early). 
    # Let's keep 4 on left.
    
    # 2. Flight Speed Consistency
    ax2 = fig.add_subplot(2, 1, 2)
    
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
    
    files = [os.path.join(log_dir, f) for f in os.listdir(log_dir) if f.endswith(".json")]
    if files:
        latest = max(files, key=os.path.getctime)
        analyze_timing_log(latest)
    else:
        print("No timing logs found.")
