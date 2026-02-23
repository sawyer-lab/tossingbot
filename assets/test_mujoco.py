
import os
import argparse
import mujoco
import mujoco.viewer
import time

def run_mujoco(model_name):
    # Resolve the URDF path
    script_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(script_dir, "urdf", model_name)
    
    if not os.path.exists(urdf_path):
        print(f"Error: {urdf_path} does not exist.")
        return

    print(f"Loading {model_name} into MuJoCo...")
    
    # Load the URDF using MuJoCo's URDF importer
    try:
        model = mujoco.MjModel.from_xml_path(urdf_path)
        data = mujoco.MjData(model)
    except Exception as e:
        print(f"Failed to load URDF: {e}")
        print("Note: MuJoCo requires the URDF and all meshes to be valid and locally accessible.")
        return

    # Start the interactive viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        print("MuJoCo Viewer launched.")
        print("Controls:")
        print("- Click and drag to move")
        print("- Scroll to zoom")
        print("- Double-click a body to select it")
        
        while viewer.is_running():
            step_start = time.time()

            # Advance simulation
            mujoco.mj_step(model, data)

            # Update viewer
            viewer.sync()

            # Maintain real-time
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualize Sawyer in MuJoCo")
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
    
    run_mujoco(mapping[args.model])
