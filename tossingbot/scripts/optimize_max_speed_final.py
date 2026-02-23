
import os
import time
import json
import numpy as np
from tossingbot.tossing.motion_planner import TossingPlanner
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
    best_v, low, high = 0, 3.0, 5.0
    tol = 0.05
    while (high - low) > tol:
        mid = (low + high) / 2
        try:
            planner.solve(q_start, target_pos, mid, duration=planner.min_duration)
            best_v, low = mid, mid
        except:
            high = mid
    return best_v

def run_final_search():
    print("--- FINAL TOSSING LIMIT SEARCH ---")
    # Tighter grid around the winners
    j1_range = np.array([-0.2, -0.1, 0.0])
    j3_range = np.array([0.7, 0.8, 0.9])
    j5_range = np.array([0.3, 0.4, 0.5])
    
    # Task Space - Pushing reach and height
    x_range = np.array([0.95, 1.0, 1.05])
    z_range = np.array([0.45, 0.55, 0.65])
    
    tasks = []
    for j1 in j1_range:
        for j3 in j3_range:
            for j5 in j5_range:
                for x in x_range:
                    for z in z_range:
                        tasks.append((np.array([j1, j3, j5]), np.array([x, 0.0, z])))
    
    results = []
    for i, (q_start, target_pos) in enumerate(tasks):
        max_v = find_max_speed(q_start, target_pos)
        if max_v > 3.0:
            dist = calculate_distance(max_v, target_pos[0], target_pos[2])
            results.append({'q_start': q_start.tolist(), 'target_pos': target_pos.tolist(), 'max_speed': max_v, 'est_distance': dist})
        if (i+1) % 20 == 0:
            print(f"Progress: {i+1}/{len(tasks)} | Best Dist: {max([r['est_distance'] for r in results] + [0]):.3f}m")

    results.sort(key=lambda x: x['est_distance'], reverse=True)
    print("" + "="*70 + "WINNING CONFIGURATIONS" + "="*70)
    for i in range(min(5, len(results))):
        r = results[i]
        print(f"{i+1}. DIST: {r['est_distance']:.3f}m | SPEED: {r['max_speed']:.2f} m/s | REL: {r['target_pos']}")

if __name__ == "__main__":
    run_final_search()
