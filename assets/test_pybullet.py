
import os
import time
import argparse
import pybullet as p
import pybullet_data

def run_sim(model_name):
    # Connect to PyBullet with GUI
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    
    # Load a plane
    p.loadURDF("plane.urdf")
    
    # Resolve the URDF path
    script_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(script_dir, "urdf", model_name)
    
    if not os.path.exists(urdf_path):
        print(f"Error: {urdf_path} does not exist.")
        p.disconnect()
        return

    print(f"Loading {model_name} into PyBullet...")
    
    # Load the robot
    # useFixedBase=True keeps it from falling over
    # flags=p.URDF_USE_INERTIA_FROM_FILE helps with accurate physics
    robot_id = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=True)
    
    num_joints = p.getNumJoints(robot_id)
    print(f"Robot loaded with {num_joints} joints.")
    
    # Set up some simple sliders to control the joints
    joint_indices = []
    for i in range(num_joints):
        info = p.getJointInfo(robot_id, i)
        joint_name = info[1].decode("utf-8")
        joint_type = info[2]
        if joint_type == p.JOINT_REVOLUTE:
            param_id = p.addUserDebugParameter(joint_name, -3.14, 3.14, 0)
            joint_indices.append((i, param_id))

    print("Simulation running. Use the sliders in the right panel to move the robot.")
    print("Press Ctrl+C in this terminal to exit.")
    
    try:
        while True:
            # Update joint positions from sliders
            for joint_idx, param_id in joint_indices:
                target_pos = p.readUserDebugParameter(param_id)
                p.setJointMotorControl2(robot_id, joint_idx, p.POSITION_CONTROL, target_pos)
            
            p.stepSimulation()
            time.sleep(1./240.)
    except KeyboardInterrupt:
        print("\nExiting simulation...")
    finally:
        p.disconnect()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualize Sawyer in PyBullet")
    parser.add_argument(
        "--model", 
        choices=["default", "tabletop_electric", "full_pneumatic", "arm_only"], 
        default="default",
        help="Which configuration to visualize"
    )
    
    args = parser.parse_args()
    mapping = {
        "default": "sawyer_tabletop_pneumatic.urdf",
        "tabletop_electric": "sawyer_tabletop_electric.urdf",
        "full_pneumatic": "sawyer_full_pneumatic.urdf",
        "arm_only": "sawyer_arm_only.urdf"
    }
    
    run_sim(mapping[args.model])
