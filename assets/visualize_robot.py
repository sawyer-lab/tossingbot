
import os
import argparse
import yourdfpy

def visualize(model_name):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(script_dir, "urdf", model_name)
    
    if not os.path.exists(urdf_path):
        print(f"Error: {urdf_path} does not exist.")
        return

    print(f"\n--- Starting yourdfpy Viewer for: {model_name} ---")
    print("Controls:")
    print("- Use Sliders to move joints (if pyrender backend is active)")
    print("- Left Click: Rotate camera")
    print("- Right Click: Pan camera")
    print("- Scroll: Zoom")
    
    # Load and show
    robot = yourdfpy.URDF.load(urdf_path)
    robot.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualize Sawyer Robot URDF")
    parser.add_argument(
        "--model", 
        choices=["default", "tabletop_electric", "full_pneumatic", "arm_only", "pedestal"], 
        default="default",
        help="Which configuration to visualize"
    )
    
    args = parser.parse_args()
    mapping = {
        "default": "sawyer_tabletop_pneumatic.urdf",
        "tabletop_electric": "sawyer_tabletop_electric.urdf",
        "full_pneumatic": "sawyer_full_pneumatic.urdf",
        "arm_only": "sawyer_arm_only.urdf",
        "pedestal": "sawyer_pedestal.urdf"
    }
    
    visualize(mapping[args.model])
