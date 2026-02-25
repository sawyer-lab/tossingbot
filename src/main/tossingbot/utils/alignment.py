import numpy as np

def get_base_rotation(target_x, target_y, offset_y):
    """
    Calculates the required base rotation (J0) to aim a laterally offset arm at a target.
    Exact solution: theta = atan2(y, x) - arcsin(d / R)
    
    Args:
        target_x (float): Target X in robot frame.
        target_y (float): Target Y in robot frame.
        offset_y (float): Lateral offset of the tossing arm (positive = left shift).
        
    Returns:
        float: Required J0 angle in radians.
    """
    dist_to_target = np.sqrt(target_x**2 + target_y**2)

    # Safety check: Target must be outside the offset circle
    if dist_to_target < abs(offset_y):
        # Fallback to simple approximation if target is inside the offset circle
        return np.arctan2(target_y - offset_y, target_x)

    return np.arctan2(target_y, target_x) - np.arcsin(offset_y / dist_to_target)
