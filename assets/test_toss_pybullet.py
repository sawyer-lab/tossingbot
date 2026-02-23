
import os
import time
import argparse
import numpy as np
import pybullet as p
import pybullet_data
from tossingbot.tossing.motion_planner import TossingPlanner
from tossingbot import config as cfg

def run_toss_sim(speed=1.5):
    # 1. Setup PyBullet
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    
    # Match physics step to planner dt
    dt = 0.01 # We'll set this explicitly
    p.setTimeStep(dt)
    
    p.loadURDF("plane.urdf")
    
    # Resolve URDF
    script_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(script_dir, "urdf", "sawyer_tabletop_pneumatic.urdf")
    
    robot_id = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=True)
    
    # 2. Map Joints and Links
    num_joints = p.getNumJoints(robot_id)
    joint_map = {}
    link_map = {}
    for i in range(num_joints):
        info = p.getJointInfo(robot_id, i)
        joint_name = info[1].decode("utf-8")
        link_name = info[12].decode("utf-8")
        joint_map[joint_name] = i
        link_map[link_name] = i
        
    active_joints = [f"right_j{i}" for i in range(7)]
    joint_indices = [joint_map[name] for name in active_joints]
    
    # 3. Plan Trajectory
    print(f"Planning toss at {speed} m/s...")
    # Move to ready position first
    for i, q in enumerate(cfg.TOSS_READY_POS):
        p.resetJointState(robot_id, joint_indices[i], q)
        
    q0_3dof = np.array([cfg.TOSS_READY_POS[1], cfg.TOSS_READY_POS[3], cfg.TOSS_READY_POS[5]])
    toss_planner = TossingPlanner(profile="express", angle_deg=45, q0=q0_3dof)
    
    sol_3d = toss_planner.get_trajectory(speed)
    sol_7d = toss_planner.map_to_7dof(
        sol_3d["Q"], sol_3d["Qd"], sol_3d["Qdd"],
        cfg.TOSS_READY_POS[0], cfg.TOSS_READY_POS[2], 
        cfg.TOSS_READY_POS[4], cfg.TOSS_READY_POS[6]
    )
    
    # 4. Run Execution Loop
    print("\nExecuting toss trajectory via Velocity Control...")
    time.sleep(1.0)
    
    # Trajectory Data
    Q_traj = sol_7d['Q']
    V_traj = sol_7d['Qd']
    dt = toss_planner.dt # usually 0.01s (100Hz)
    
    num_steps = Q_traj.shape[0]
    tip_link_idx = link_map["right_gripper_tip"]
    
    actual_vels = []
    
    try:
        # Step through the trajectory
        for k in range(num_steps):
            # Target Velocities for this step
            target_vels = V_traj[k]
            
            # Apply Velocity Control to all 7 joints
            for i, joint_idx in enumerate(joint_indices):
                p.setJointMotorControl2(
                    bodyIndex=robot_id,
                    jointIndex=joint_idx,
                    controlMode=p.VELOCITY_CONTROL,
                    targetVelocity=target_vels[i],
                    force=200 # Max effort
                )
            
            p.stepSimulation()
            
            # Measure Actual Tip Velocity
            # computeLinkVelocity=1 ensures we get linear/angular velocity in world frame
            state = p.getLinkState(robot_id, tip_link_idx, computeLinkVelocity=1)
            linear_vel = state[6]
            speed_mag = np.linalg.norm(linear_vel)
            actual_vels.append(speed_mag)
            
            # We must sleep to match the planner's dt
            time.sleep(dt)
            
            if k == sol_3d["index"]:
                print(f"\n>>> RELEASE POINT REACHED <<<")
                print(f"    Planned Speed: {speed:.3f} m/s")
                print(f"    Actual Speed:  {speed_mag:.3f} m/s")
                print(f"    Error:         {abs(speed_mag - speed):.3f} m/s")

        print("\nTrajectory finished. Holding final position...")
        while True:
            p.stepSimulation()
            time.sleep(1./240.)
            
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        p.disconnect()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Toss Simulation in PyBullet")
    parser.add_argument("--speed", type=float, default=1.5, help="Toss speed in m/s")
    args = parser.parse_args()
    
    run_toss_sim(args.speed)
