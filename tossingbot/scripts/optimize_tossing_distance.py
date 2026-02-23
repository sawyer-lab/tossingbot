
import os
import time
import json
import numpy as np
from tossingbot.tossing.motion_planner import TossingPlanner
from tossingbot import config as cfg

def calculate_distance(v, x, z, angle_deg=45):
    """Estimate projectile landing distance from robot base."""
    g = 9.81
    theta = np.deg2rad(angle_deg)
    h = z + 0.25 # Height above table
    
    vx = v * np.cos(theta)
    vz = v * np.sin(theta)
    
    # Time of flight: t = (vz + sqrt(vz^2 + 2gh)) / g
    t_flight = (vz + np.sqrt(vz**2 + 2 * g * h)) / g
    d_flight = vx * t_flight
    
    return x + d_flight

def find_max_speed(q_start, target_pos):
    planner = TossingPlanner(profile="express", angle_deg=45, q0=q_start)
    best_v = 0
    low, high = 2.0, 5.0 # Increased ceiling
    tol = 0.05
    
    while (high - low) > tol:
        mid = (low + high) / 2
        try:
            planner.solve(q_start, target_pos, mid, duration=planner.min_duration)
            best_v = mid
            low = mid
        except:
            high = mid
    return best_v

def run_refined_search():
    print("--- ULTIMATE TOSSING DISTANCE OPTIMIZATION ---")
    
    # Extreme ranges for absolute limits
    j1_range = np.linspace(-0.4, 0.0, 3) 
    j3_range = np.linspace(0.8, 1.4, 3)
    j5_range = np.linspace(0.4, 1.0, 3)
    
    # Outer task space
    x_range = np.linspace(0.9, 1.15, 4)
    z_range = np.linspace(0.2, 0.5, 3)
    
    tasks = []
    for j1 in j1_range:
        for j3 in j3_range:
            for j5 in j5_range:
                for x in x_range:
                    for z in z_range:
                        tasks.append((np.array([j1, j3, j5]), np.array([x, 0.0, z])))
    
    total = len(tasks)
    print(f"Total configurations to test: {total}")
    
    results = []
    start_time = time.time()
    
    for i, (q_start, target_pos) in enumerate(tasks):
        max_v = find_max_speed(q_start, target_pos)
        
        if max_v > 2.0:
            dist = calculate_distance(max_v, target_pos[0], target_pos[2])
            results.append({
                'q_start': q_start.tolist(),
                'target_pos': target_pos.tolist(),
                'max_speed': max_v,
                'est_distance': dist
            })
            
        if (i + 1) % 20 == 0 or (i + 1) == total:
            elapsed = time.time() - start_time
            best_dist = max([r['est_distance'] for r in results] + [0])
            print(f"Progress: {i+1}/{total} | Best Dist: {best_dist:.2f}m | Elapsed: {elapsed:.1f}s")

    # Sort by Distance
    results.sort(key=lambda x: x['est_distance'], reverse=True)
    
    print("" + "="*70)
    print(f"{'RANK':<5} | {'DIST':<8} | {'SPEED':<8} | {'START Q':<18} | {'REL (X,Z)':<12}")
    print("-" * 70)
    
    for i in range(min(10, len(results))):
        r = results[i]
        q_str = f"[{r['q_start'][0]:.1f},{r['q_start'][1]:.1f},{r['q_start'][2]:.1f}]"
        p_str = f"[{r['target_pos'][0]:.2f},{r['target_pos'][2]:.2f}]"
        print(f"{i+1:<5} | {r['est_distance']:<8.3f} | {r['max_speed']:<8.2f} | {q_str:<18} | {p_str:<12}")

    out_file = os.path.join(cfg.PROJECT_ROOT, "logs", "refined_distance_opt.json")
    os.makedirs(os.path.dirname(out_file), exist_ok=True)
    with open(out_file, 'w') as f:
        json.dump(results, f, indent=2)
    print(f"Saved {len(results)} feasible configs to: {out_file}")

if __name__ == "__main__":
    run_refined_search()
