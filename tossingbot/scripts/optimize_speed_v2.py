
import os
import time
import json
import numpy as np
from tossingbot.tossing.motion_planner import TossingPlanner
from tossingbot.tossing.kinematics import RobotKinematics
from tossingbot import config as cfg

def calculate_distance(v, x, z, angle_deg=45):
    g = 9.81
    theta = np.deg2rad(angle_deg)
    h = z + 0.25
    vx, vz = v * np.cos(theta), v * np.sin(theta)
    t_flight = (vz + np.sqrt(vz**2 + 2 * g * h)) / g
    return x + vx * t_flight

def find_max_speed(q_start, target_pos):
    planner = TossingPlanner(profile="express", angle_deg=45, q0=q_start)
    best_v, low, high = 0, 1.5, 5.0 # Lowered floor
    tol = 0.1
    while (high - low) > tol:
        mid = (low + high) / 2
        try:
            planner.solve(q_start, target_pos, mid, duration=planner.min_duration)
            best_v, low = mid, mid
        except:
            high = mid
    return best_v

def run_v2_search():
    print("--- TASK-SPACE TOSSING OPTIMIZATION V2 (Relaxed) ---")
    rk = RobotKinematics()
    
    # Start: More generous range
    start_x_range = np.linspace(0.4, 0.7, 3)
    start_z_range = np.linspace(0.2, 0.6, 3)
    
    # End: Far and High
    end_x_range = np.linspace(0.8, 1.1, 3)
    end_z_range = np.linspace(0.3, 0.7, 3)
    
    tasks = []
    for sx in start_x_range:
        for sz in start_z_range:
            # We assume a vertical-ish orientation for the wind-up
            try:
                q_start = rk.inverse_kinematics_analytical([sx, sz, 0.0])
                for ex in end_x_range:
                    for ez in end_z_range:
                        if ex > sx: # Must move forward
                            tasks.append((q_start, np.array([ex, 0.0, ez]), [sx, sz]))
            except:
                continue # Skip unreachable wind-ups

    print(f"Total task-space combinations: {len(tasks)}")
    
    results = []
    start_time = time.time()
    
    for i, (q_start, target_pos, start_coords) in enumerate(tasks):
        max_v = find_max_speed(q_start, target_pos)
        
        if max_v > 2.5:
            dist = calculate_distance(max_v, target_pos[0], target_pos[2])
            results.append({
                'q_start': q_start.tolist(),
                'start_pos': start_coords,
                'target_pos': target_pos.tolist(),
                'max_speed': max_v,
                'est_distance': dist
            })
            
        if (i + 1) % 10 == 0 or (i + 1) == len(tasks):
            elapsed = time.time() - start_time
            best_dist = max([r['est_distance'] for r in results] + [0])
            print(f"Progress: {i+1}/{len(tasks)} | Best Dist: {best_dist:.2f}m | Elapsed: {elapsed:.1f}s")

    # Sort by Distance
    results.sort(key=lambda x: x['est_distance'], reverse=True)
    
    print("" + "="*80)
    print(f"{'RANK':<5} | {'DIST':<8} | {'SPEED':<8} | {'WIND-UP (X,Z)':<15} | {'RELEASE (X,Z)':<15}")
    print("-" * 80)
    
    for i in range(min(10, len(results))):
        r = results[i]
        w_str = f"[{r['start_pos'][0]:.2f}, {r['start_pos'][1]:.2f}]"
        p_str = f"[{r['target_pos'][0]:.2f}, {r['target_pos'][2]:.2f}]"
        print(f"{i+1:<5} | {r['est_distance']:<8.3f} | {r['max_speed']:<8.2f} | {w_str:<15} | {p_str:<15}")

    out_file = os.path.join(cfg.PROJECT_ROOT, "logs", "task_space_opt_v2.json")
    os.makedirs(os.path.dirname(out_file), exist_ok=True)
    with open(out_file, 'w') as f:
        json.dump(results, f, indent=2)
    print(f"\nSaved {len(results)} configurations to: {out_file}")

if __name__ == "__main__":
    run_v2_search()
