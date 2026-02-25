import sys
import os
import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), '../src/main'))
from tossingbot.legacy.tossing.motion_planner import TossingPlanner

def find_max_speed(q_start, target_pos):
    planner = TossingPlanner(profile="express", angle_deg=45, q0=q_start)
    best_v, low, high = 0, 0.1, 4.0
    tol = 0.1
    while (high - low) > tol:
        mid = (low + high) / 2
        try:
            # solve raises or returns sol
            sol = planner.solve(q_start, target_pos, mid, duration=planner.min_duration)
            if sol is not None:
                best_v, low = mid, mid
            else:
                high = mid
        except Exception as e:
            high = mid
    return best_v

def run_search():
    init_x_vals = [0.25, 0.3, 0.35]
    init_z_vals = [0.1, 0.2, 0.3]
    
    final_x_vals = [0.75, 0.8, 0.85]
    final_z_vals = [0.2, 0.3, 0.4]

    dummy_planner = TossingPlanner(profile="express")
    
    results = []
    print("Starting Legacy Grid Search...")
    print(f"{'Init Pos (X,Z)':<15} | {'Final Pos (X,Z)':<15} | {'Max Speed (m/s)':<15} | {'Q Start [J1,J3,J5]'}")
    print("-" * 80)

    for ix in init_x_vals:
        for iz in init_z_vals:
            init_pos = np.array([ix, 0.0, iz])
            
            try:
                q_start = dummy_planner.rk.inverse_kinematics_analytical(init_pos)
            except:
                continue
                
            if q_start is None or np.any(np.isnan(q_start)):
                continue

            for fx in final_x_vals:
                for fz in final_z_vals:
                    target_pos = np.array([fx, 0.0, fz])
                    max_speed = find_max_speed(q_start, target_pos)
                    
                    if max_speed > 0:
                        results.append({
                            'init_pos': [ix, iz],
                            'final_pos': [fx, fz],
                            'max_speed': max_speed,
                            'q_start': q_start.tolist()
                        })
                        print(f"[{ix}, {iz}]{'':<7} | [{fx}, {fz}]{'':<7} | {max_speed:.2f}{'':<11} | [{q_start[0]:.2f}, {q_start[1]:.2f}, {q_start[2]:.2f}]")

    results.sort(key=lambda x: x['max_speed'], reverse=True)
    print("\n" + "="*80)
    print("TOP 5 CONFIGURATIONS:")
    print("="*80)
    for i in range(min(5, len(results))):
        res = results[i]
        print(f"Rank {i+1}:")
        print(f"  Max Speed: {res['max_speed']:.2f} m/s")
        print(f"  Init Pos : {res['init_pos']}")
        print(f"  Final Pos: {res['final_pos']}")
        print(f"  Q Start  : {[round(q, 3) for q in res['q_start']]}")
        print("-" * 40)

if __name__ == '__main__':
    run_search()
