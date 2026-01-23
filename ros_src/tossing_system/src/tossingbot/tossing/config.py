import numpy as np

ROBOT_PARAMS = {
    "l1": 0.4,
    "l2": 0.4,
    "l3": 0.13375,
    "gripper_len": 0.1944,
    "base_offset": np.array([0.081, 0.0, 0.317]),
    "global_gripper_width": 0.01,
    "gripper_width": 0.02,
}

TRAJECTORY_CONFIG = {
    "dt": 0.01,
    "weights": {"accel": 1.0, "vel": 1.0, "pos": 1.0},
    "joint_limits": {
        "pos": {
            "min": np.array([-3.8095, -3.0439, -2.9761]),
            "max": np.array([2.2736, 3.0439, 2.9761]),
        },
        "slow": {
            "vel": np.array([0.2825, 0.415, 0.74]),
            "accel": np.array([1.5, 3.0, 3.0]),
        },
        "medium": {
            "vel": np.array([0.678, 0.996, 1.776]),
            "accel": np.array([2.5, 5.0, 5.0]),
        },
        "fast": {
            "vel": np.array([1.13, 1.66, 2.96]),
            "accel": np.array([5.0, 8.0, 8.0]),
        },
        "express": {
            "vel": np.array([1.13, 1.66, 2.96]),
            "accel": np.array([8.0, 10.0, 12.0]),
        },
    }
}