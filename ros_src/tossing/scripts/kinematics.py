import numpy as np
from config import l1, l2, l3, base_offset_x, base_offset_z, gripper

def forward_kinematics_3r(q):
    t1 = -q[0]
    t2 = -q[0] - q[1]
    t3 = -q[0] - q[1] - q[2]
    x = base_offset_x + l1 * np.cos(t1) + l2 * np.cos(t2) + (l3 + gripper) * np.cos(t3)
    z = l1 * np.sin(t1) + l2 * np.sin(t2) + (l3 + gripper) * np.sin(t3) + base_offset_z
    return np.array([x, z, t3])

def inverse_kinematics_3r(pose):
    x, z, t3 = pose
    # subtract base offsets
    x_rel = x - base_offset_x
    z_rel = z - base_offset_z
    
    # wrist position
    x_wrist = x_rel - (l3 + gripper) * np.cos(t3)
    z_wrist = z_rel - (l3 + gripper) * np.sin(t3)

    # compute D
    D = (x_wrist**2 + z_wrist**2 - l1**2 - l2**2) / (2.0 * l1 * l2)
    if abs(D) > 1:
        raise ValueError("Position is unreachable, D={}".format(D))

    q2 = np.arctan2(-np.sqrt(1 - D**2), D)
    q1 = np.arctan2(z_wrist, x_wrist) - np.arctan2(l2 * np.sin(q2), l1 + l2 * np.cos(q2))
    q3 = t3 - q1 - q2

    # match FK sign convention
    return np.array([-q1, -q2, -q3])
