"""
Coordinate transformation utilities for grasp analysis.
Transforms between world frame and object-relative frame.
"""
import numpy as np


def quaternion_to_yaw(quat):
    """
    Extract yaw (rotation around Z axis) from quaternion.
    
    Args:
        quat: [qx, qy, qz, qw] or [qw, qx, qy, qz]
    
    Returns:
        yaw in radians
    """
    # Assume [qx, qy, qz, qw] format (Gazebo standard)
    if len(quat) == 4:
        qx, qy, qz, qw = quat
    else:
        raise ValueError(f"Invalid quaternion format: {quat}")
    
    # Convert to yaw (rotation around Z)
    # Formula: yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2))
    yaw = np.arctan2(2.0 * (qw * qz + qx * qy),
                     1.0 - 2.0 * (qy**2 + qz**2))
    
    return yaw


def world_to_object_frame_2d(grasp_xy, object_pose):
    """
    Transform 2D grasp point from world frame to object frame.
    
    Args:
        grasp_xy: [x, y] in world frame (meters)
        object_pose: dict with:
            "position": [x, y, z]
            "orientation": [qx, qy, qz, qw]
    
    Returns:
        [x_obj, y_obj] in object frame (meters)
    """
    # Extract object position (2D)
    obj_pos = object_pose["position"]
    ox, oy = obj_pos[0], obj_pos[1]
    
    # Extract yaw from quaternion
    quat = object_pose["orientation"]
    yaw = quaternion_to_yaw(quat)
    
    # Step 1: Translate to object center
    dx = grasp_xy[0] - ox
    dy = grasp_xy[1] - oy
    
    # Step 2: Rotate by -yaw (inverse rotation)
    cos_y = np.cos(-yaw)
    sin_y = np.sin(-yaw)
    
    x_obj = dx * cos_y - dy * sin_y
    y_obj = dx * sin_y + dy * cos_y
    
    return np.array([x_obj, y_obj])


def pixel_to_world(u, v, roi_x, roi_y, img_h, img_w):
    """
    Convert pixel coordinates to world coordinates.
    
    This replicates the logic from VisionProcessor.pixel_to_world()
    
    Args:
        u: row pixel (0 to IMG_H-1)
        v: column pixel (0 to IMG_W-1)
        roi_x: [min_x, max_x] workspace bounds in X
        roi_y: [min_y, max_y] workspace bounds in Y
        img_h: image height in pixels
        img_w: image width in pixels
    
    Returns:
        [world_x, world_y] in meters
    """
    min_x, max_x = roi_x
    min_y, max_y = roi_y
    
    # Percentage through image
    pct_u = float(u) / img_h
    pct_v = float(v) / img_w
    
    # Map to world coordinates
    world_x = max_x - (pct_u * (max_x - min_x))
    world_y = max_y - (pct_v * (max_y - min_y))
    
    return np.array([world_x, world_y])


def test_transforms():
    """Test coordinate transformations"""
    print("Testing coordinate transforms...")
    
    # Test 1: Quaternion to yaw
    quat_0 = [0, 0, 0, 1]  # No rotation
    yaw_0 = quaternion_to_yaw(quat_0)
    print(f"Quaternion {quat_0} -> yaw: {np.degrees(yaw_0):.2f}°")
    assert np.abs(yaw_0) < 0.01, "Zero rotation should give zero yaw"
    
    # Test 2: 90 degree rotation
    quat_90 = [0, 0, 0.7071, 0.7071]  # 90° around Z
    yaw_90 = quaternion_to_yaw(quat_90)
    print(f"Quaternion {quat_90} -> yaw: {np.degrees(yaw_90):.2f}°")
    assert np.abs(yaw_90 - np.pi/2) < 0.01, "90° rotation failed"
    
    # Test 3: World to object frame (no rotation)
    object_pose = {
        "position": [0.6, 0.0, 0.76],
        "orientation": [0, 0, 0, 1]
    }
    grasp_world = [0.65, 0.05]
    grasp_obj = world_to_object_frame_2d(grasp_world, object_pose)
    print(f"World {grasp_world} -> Object {grasp_obj}")
    expected = [0.05, 0.05]  # 5cm offset in both directions
    assert np.allclose(grasp_obj, expected, atol=0.001), f"Expected {expected}, got {grasp_obj}"
    
    # Test 4: Pixel to world
    roi_x = [0.4, 0.8]
    roi_y = [-0.2, 0.2]
    img_h, img_w = 80, 80
    
    # Center pixel should map to center of workspace
    u_center, v_center = 40, 40
    world_center = pixel_to_world(u_center, v_center, roi_x, roi_y, img_h, img_w)
    expected_center = [0.6, 0.0]
    print(f"Pixel ({u_center}, {v_center}) -> World {world_center}")
    assert np.allclose(world_center, expected_center, atol=0.01), f"Center pixel failed"
    
    print("✓ All tests passed!")


if __name__ == "__main__":
    test_transforms()
