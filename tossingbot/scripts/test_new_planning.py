
import os
import numpy as np
import time
from sawyer_robot import SawyerRobot
from tossingbot import config as cfg
from tossingbot.planning.kinematics import CasadiKinematics
from tossingbot.planning.casadi_planner import CasadiPlanner

def run_test():
    print("--- NEW PLANNING INTERFACE TEST ---")
    
    # 1. Initialize Robot Client (ZMQ)
    # Ensure the ZMQ server is running in the container!
    robot = SawyerRobot(host='localhost', port=5555)
    
    # 2. Initialize Planning with NEW URDF
    print(f"Loading URDF: {cfg.SAWYER_PNEUMATIC_URDF}")
    # Note: BASE_LINK is now "right_arm_base_link" (tabletop)
    model = CasadiKinematics(cfg.SAWYER_PNEUMATIC_URDF, cfg.BASE_LINK, cfg.END_LINK)
    planner = CasadiPlanner(model)
    
    print(f"Model DOF: {model.n_dof} (should be 7)")
    print(f"Active Joints: {model.active_joint_names}")

    try:
        # 3. Get Current State
        # robot.arm.get_joints() returns a JointPositions object
        current_joints = robot.arm.get_joints()
        q_curr = [
            current_joints.j0, current_joints.j1, current_joints.j2,
            current_joints.j3, current_joints.j4, current_joints.j5, current_joints.j6
        ]
        
        # 4. Plan a simple Cartesian move (10cm up)
        start_pos = np.array(model.fk_pos(q_curr)).flatten()
        target_pos = start_pos + np.array([0, 0, 0.10])
        
        print(f"Planning from {np.round(start_pos, 3)} to {np.round(target_pos, 3)}...")
        
        path = planner.plan_cartesian(
            q_start=q_curr,
            target_pos=target_pos,
            duration=2.0,
            speed_ratio=0.5,
            check_floor=True
        )
        
        if path:
            print(f"Plan Success! Generated {len(path)} waypoints.")
            
            # 5. Execute via ZMQ Client
            # The new RobotClient has a trajectory execution method or we can stream
            # For simplicity, let's use the raw joint command for now or check for trajectory support
            print("Executing trajectory...")
            
            # Format waypoints for robot.arm.move_trajectory or similar
            # If the client doesn't have a high-level trajectory yet, we use execute_stream
            # robot._client.send_command('execute_stream_trajectory', ...)
            
            Q = [p['position'] for p in path]
            Qd = [p['velocity'] for p in path]
            Qdd = [p['acceleration'] for p in path]
            
            robot._client.send_command('execute_stream_trajectory', {
                'Q': Q,
                'Qd': Qd,
                'Qdd': Qdd
            })
            
            print("Execution finished.")
        else:
            print("Planning FAILED.")

    except Exception as e:
        print(f"Error: {e}")
    finally:
        robot.close()

if __name__ == "__main__":
    run_test()
