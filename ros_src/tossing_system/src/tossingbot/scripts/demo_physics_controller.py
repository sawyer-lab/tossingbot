#!/usr/bin/env python3.8
"""
Standalone demo of the PhysicsController ballistic trajectory calculations.
This doesn't require ROS or robot hardware - just pure physics calculations.
"""
import numpy as np
from tossingbot.tossing.physics_controller import PhysicsController


def demo_physics_calculations():
    """Demonstrate physics controller calculations for various targets."""

    print("\n" + "="*70)
    print("PHYSICS CONTROLLER DEMO")
    print("Ballistic Trajectory Calculations (No Robot Required)")
    print("="*70)

    # Create physics controller with simulation parameters
    physics = PhysicsController(
        release_radius=0.7,   # cd = 0.7m (distance from robot base)
        release_height=0.04,  # ch = 0.04m (release height above base)
        gravity=9.8,
        max_speed=2.0,
        min_speed=0.5
    )

    # Test targets (in robot frame, z=0 is ground level)
    # Note: Robot base is at z=1.0 in world frame, so table at z=0.76 world = z=-0.24 robot
    test_targets = [
        ("Close Center", [0.825, 0.0, -0.24]),
        ("Center Bin", [1.10, 0.15, -0.24]),
        ("Far Target", [1.50, 0.0, -0.24]),
        ("Left Offset", [1.10, 0.25, -0.24]),
        ("Too Close", [0.5, 0.0, -0.24]),  # Should fail
        ("Too Far", [2.5, 0.0, -0.24]),   # Should require high speed
    ]

    print(f"\nPhysics Parameters:")
    print(f"  Release Radius (cd): {physics.cd} m")
    print(f"  Release Height (ch): {physics.ch} m")
    print(f"  Gravity: {physics.g} m/s²")
    print(f"  Speed Limits: {physics.min_speed} - {physics.max_speed} m/s")
    print(f"  Release Angle: {physics.release_angle_deg}°")

    print("\n" + "="*70)
    print("TESTING DIFFERENT TARGET POSITIONS")
    print("="*70)

    for name, target in test_targets:
        print(f"\n--- {name} ---")
        params = physics.calculate_release_params(target)
        physics.print_trajectory_info(target, params)

        if params['valid']:
            # Show velocity components
            vel = physics.get_release_velocity_components(
                params['release_speed'], params['theta']
            )
            print(f"Horizontal velocity: {vel['v_horizontal']:.3f} m/s")
            print(f"Components: vx={vel['vx']:.3f}, vy={vel['vy']:.3f}, vz={vel['vz']:.3f}")

    print("\n" + "="*70)
    print("PHYSICS VALIDATION TEST")
    print("="*70)

    # Test round-trip: calculate speed for target, then verify landing position
    target = np.array([1.10, 0.15, -0.24])
    print(f"\nTarget: [{target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}]")

    params = physics.calculate_release_params(target)
    if params['valid']:
        print(f"Calculated speed: {params['release_speed']:.3f} m/s")

        # Forward prediction
        predicted = physics.predict_landing_position(
            params['release_pos'],
            params['release_speed'],
            params['theta']
        )

        print(f"Release position: [{params['release_pos'][0]:.3f}, {params['release_pos'][1]:.3f}, {params['release_pos'][2]:.3f}]")
        print(f"Predicted landing: [{predicted[0]:.3f}, {predicted[1]:.3f}, {predicted[2]:.3f}]")

        # Calculate error
        error_xy = np.linalg.norm(predicted[:2] - target[:2])
        print(f"XY Prediction Error: {error_xy*1000:.2f} mm")

        if error_xy < 0.001:  # Less than 1mm error
            print("✓ Physics equations are CONSISTENT!")
        else:
            print("✗ WARNING: Prediction error detected")

    print("\n" + "="*70)
    print("Demo Complete!")
    print("="*70 + "\n")


if __name__ == "__main__":
    demo_physics_calculations()
