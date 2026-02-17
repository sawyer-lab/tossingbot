#!/usr/bin/env python3
"""
Gazebo Object Management Demo

Demonstrates all Gazebo object management features:
- Spawning objects (with and without colors)
- Querying and setting poses
- Batch spawning with collision avoidance
- Listing objects and available models
- Despawning objects

Usage:
    python gazebo_demo.py

Requirements:
    - Gazebo simulator running in container
    - ZMQ server running in container
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from robot_client import GazeboClient
import time

# Path to models on the host filesystem
MODELS_PATH = os.path.join(os.path.dirname(__file__), '..', '..', '..',
                           'ros', 'environments', 'models')


def print_section(title):
    """Print section header."""
    print("\n" + "="*70)
    print(f"  {title}")
    print("="*70)


def main():
    print_section("Gazebo Object Management Demo")

    # Connect
    print("\n[1] Connecting to Gazebo client...")
    gazebo = GazeboClient(
        protocol='zmq',
        host='localhost',
        model_paths=[os.path.abspath(MODELS_PATH)]
    )
    print("    ✓ Connected")

    # Show available models
    print_section("Available Models")
    models = gazebo.available_models()
    print(f"\n   Found {len(models)} models:")
    for i, model in enumerate(models[:20], 1):
        print(f"   {i:2d}. {model}")
    if len(models) > 20:
        print(f"   ... and {len(models) - 20} more")

    # Clean up any existing objects
    print_section("Cleanup")
    print("\n   Removing any existing objects...")
    count = gazebo.despawn_all()
    print(f"   ✓ Removed {count} objects")
    time.sleep(1)

    # Spawn individual objects with colors
    print_section("Spawn Individual Objects")

    print("\n   [1] Spawning red cube...")
    success = gazebo.spawn_with_color(
        model_type='cube',
        name='red_cube',
        position=[0.6, -0.1, 0.8],
        color='Gazebo/Red'
    )
    print(f"       ✓ Success: {success}")
    time.sleep(0.5)

    print("\n   [2] Spawning blue sphere...")
    success = gazebo.spawn_with_color(
        model_type='sphere',
        name='blue_sphere',
        position=[0.6, 0.0, 0.8],
        color='Gazebo/Blue'
    )
    print(f"       ✓ Success: {success}")
    time.sleep(0.5)

    print("\n   [3] Spawning green cylinder...")
    success = gazebo.spawn_with_color(
        model_type='cylinder',
        name='green_cylinder',
        position=[0.6, 0.1, 0.8],
        color='Gazebo/Green'
    )
    print(f"       ✓ Success: {success}")
    time.sleep(0.5)

    # List spawned objects
    print_section("List Spawned Objects")
    objects = gazebo.list_objects()
    print(f"\n   Current objects ({len(objects)}):")
    for obj in objects:
        print(f"   - {obj}")

    # Get and display poses
    print_section("Query Object Poses")
    for obj_name in ['red_cube', 'blue_sphere', 'green_cylinder']:
        pose = gazebo.get_pose(obj_name)
        if pose:
            pos = pose['position']
            print(f"\n   {obj_name}:")
            print(f"     Position: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
        else:
            print(f"\n   {obj_name}: Not found")

    # Move an object
    print_section("Move Object")
    print("\n   Moving red_cube up by 0.2m...")
    pose = gazebo.get_pose('red_cube')
    if pose:
        new_pos = pose['position'].copy()
        new_pos[2] += 0.2
        success = gazebo.set_pose('red_cube', new_pos)
        print(f"   ✓ Moved: {success}")
        time.sleep(1)

        new_pose = gazebo.get_pose('red_cube')
        if new_pose:
            new_p = new_pose['position']
            print(f"   New position: [{new_p[0]:.3f}, {new_p[1]:.3f}, {new_p[2]:.3f}]")

    # Check if object exists
    print_section("Check Object Existence")
    test_names = ['red_cube', 'blue_sphere', 'nonexistent_object']
    for name in test_names:
        exists = gazebo.is_spawned(name)
        status = "EXISTS" if exists else "NOT FOUND"
        print(f"   {name:20s} : {status}")

    # Spawn randomized scene
    print_section("Randomized Batch Spawning")
    print("\n   Spawning randomized scene with collision avoidance...")

    spawned = gazebo.spawn_randomized(
        model_types=['cube', 'sphere', 'cylinder'],
        area_x=[0.5, 0.8],
        area_y=[-0.3, 0.3],
        z_height=0.8,
        min_dist=0.08,
        instances_per_type=3
    )

    print(f"\n   ✓ Spawned {len(spawned)} objects:")
    for obj in spawned:
        print(f"     - {obj}")

    time.sleep(3)

    # List all objects
    print_section("Final Object List")
    all_objects = gazebo.list_objects()
    print(f"\n   Total objects: {len(all_objects)}")
    for obj in all_objects:
        print(f"   - {obj}")

    # Clean up
    print_section("Cleanup")
    print("\n   Removing all objects...")
    count = gazebo.despawn_all()
    print(f"   ✓ Removed {count} objects")

    remaining = gazebo.list_objects()
    print(f"   Remaining objects: {len(remaining)}")

    print_section("Demo Complete")
    print("\n   All Gazebo object management features demonstrated successfully!")
    print()

    gazebo.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n[Interrupted] Demo stopped by user")
    except Exception as e:
        print(f"\n\n[ERROR] Demo failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
