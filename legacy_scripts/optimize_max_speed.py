import os
import time
import json
import numpy as np
from tossingbot.tossing.motion_planner import TossingPlanner
from tossingbot import config as cfg

def find_max_speed_fast(q_start, target_pos):
    """Coarse binary search for the maximum feasible speed."""
    planner = TossingPlanner(profile="express", angle_deg=45, q0=q_start)
    
    best_v = 0
    low = 3.0
    high = 4.5 # Push the ceiling slightly
    tol = 0.1  # Coarser tolerance for speed
    
    while (high - low) > tol:
        mid = (low + high) / 2
        try:
            planner.solve(q_start, target_pos, mid, duration=planner.min_duration)
            best_v = mid
            low = mid
        except:
            high = mid
            
    if best_v >= 3.0:
        return {
            'q_start': q_start.tolist(),
            'target_pos': target_pos.tolist(),
            'max_speed': best_v
        }
    return None

def run_coarse_search():
    print("--- COARSE TASK-SPACE OPTIMIZATION (Speed: 3.0 - 4.5 m/s) ---")
    
    # 1. Define Coarse Search Ranges (Square around current areas)
    # Start configurations (J1, J3, J5) - Big steps
    j1_range = np.array([-1.2, -0.8, -0.4])
    j3_range = np.array([1.2, 1.6, 2.0])
    j5_range = np.array([0.4, 0.8, 1.2])
    
    # Task Space (X, Z) - Square around 0.75m reach
    x_range = np.array([0.6, 0.75, 0.9])
    z_range = np.array([-0.1, 0.05, 0.2])
    
    tasks = []
    for j1 in j1_range:
        for j3 in j3_range:
            for j5 in j5_range:
                for x in x_range:
                    for z in z_range:
                        tasks.append((np.array([j1, j3, j5]), np.array([x, 0.0, z])))
    
    total = len(tasks)
    print(f"Total coarse configurations to test: {total}")
    
    results = []
    start_time = time.time()
    
    for i, (q_start, target_pos) in enumerate(tasks):
        res = find_max_speed_fast(q_start, target_pos)
        if res:
            results.append(res)
            
        if (i + 1) % 20 == 0 or (i + 1) == total:
            elapsed = time.time() - start_time
            best_so_far = max([r['max_speed'] for r in results] + [0])
            print(f"Progress: {i+1}/{total} | Best Speed: {best_so_far:.2f} m/s | Elapsed: {elapsed:.1f}s")

    # Sort and Report
    results.sort(key=lambda x: x['max_speed'], reverse=True)
    
    print("\n" + "="*60)
    print(f"{'RANK':<5} | {'SPEED':<10} | {'START Q':<20} | {'RELEASE (X,Z)':<15}")
    print("-" * 60)
    
    for i in range(min(10, len(results))):
        r = results[i]
        q_str = f"[{r['q_start'][0]:.1f}, {r['q_start'][1]:.1f}, {r['q_start'][2]:.1f}]"
        p_str = f"[{r['target_pos'][0]:.2f}, {r['target_pos'][2]:.2f}]"
        print(f"{i+1:<5} | {r['max_speed']:<10.2f} | {q_str:<20} | {p_str:<15}")

    out_file = os.path.join(cfg.PROJECT_ROOT, "logs", "coarse_speed_opt.json")
    os.makedirs(os.path.dirname(out_file), exist_ok=True)
    with open(out_file, 'w') as f:
        json.dump(results, f, indent=2)
    print(f"\nSaved {len(results)} feasible configs to: {out_file}")

if __name__ == "__main__":
    run_coarse_search()
