
import os
import argparse
import time
import numpy as np

# Pydrake imports
from pydrake.all import (
    DiagramBuilder,
    Meshcat,
    MeshcatVisualizer,
    MultibodyPlant,
    Parser,
    Simulator,
    StartMeshcat,
    AddMultibodyPlantSceneGraph,
    BodyIndex,
)

def run_drake(model_name):
    # Start meshcat visualizer server
    meshcat = StartMeshcat()
    print(f"Meshcat URL: {meshcat.web_url()}")
    print("Open this link in your browser to see the robot.")

    builder = DiagramBuilder()
    
    # Add MultibodyPlant (the physics engine part)
    # 0.001 is the discrete time step
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)
    
    # Resolve the URDF path
    script_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(script_dir, "urdf", model_name)
    
    if not os.path.exists(urdf_path):
        print(f"Error: {urdf_path} does not exist.")
        return

    # Load the robot from URDF
    parser = Parser(plant)
    parser.AddModels(urdf_path)
    
    # Weld the base link to the world (so it doesn't fall)
    # Drake uses BodyIndex for get_body
    all_body_names = []
    for i in range(plant.num_bodies()):
        all_body_names.append(plant.get_body(BodyIndex(i)).name())
    
    base_link_name = None
    if "base" in all_body_names:
        base_link_name = "base"
    elif "right_arm_base_link" in all_body_names:
        base_link_name = "right_arm_base_link"
        
    if base_link_name:
        base_link = plant.GetBodyByName(base_link_name)
        plant.WeldFrames(plant.world_frame(), base_link.body_frame())

    # Finalize the plant
    plant.Finalize()

    # Add Visualizer
    visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    
    # Build diagram and setup simulator
    diagram = builder.Build()
    simulator = Simulator(diagram)
    
    print("\nDrake Simulator running.")
    print("Press Ctrl+C to exit.")
    
    try:
        simulator.set_target_realtime_rate(1.0)
        # Simulation loop
        while True:
            simulator.AdvanceTo(simulator.get_context().get_time() + 0.1)
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nExiting simulation...")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualize Sawyer in Pydrake")
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
    
    run_drake(mapping[args.model])
