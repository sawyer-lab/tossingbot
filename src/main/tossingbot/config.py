import os
import numpy as np

import sawyer_assets

# Gets directory: .../tossingbot
_CONFIG_DIR = os.path.dirname(os.path.abspath(__file__))

# Go up 1 level to project root (from tossingbot/tossingbot/ to tossingbot/)
PROJECT_ROOT = os.path.abspath(os.path.join(_CONFIG_DIR, ".."))

# Session Management
SESSION_BASE_DIR = os.path.join(PROJECT_ROOT, "sessions")

# Asset Paths (Using sawyer-assets package)
SAWYER_PNEUMATIC_URDF = sawyer_assets.get_urdf_path("sawyer_tabletop_pneumatic.urdf")
SAWYER_ELECTRIC_URDF = sawyer_assets.get_urdf_path("sawyer_tabletop_electric.urdf")

# Robot Parameters
BASE_LINK = "right_arm_base_link"
END_LINK = "right_gripper_tip"

# Workstation Parameters
TABLE_HEIGHT = 0.75
WORKSPACE_SIZE = 0.40
CENTER_X = 0.60
CENTER_Y = 0.1363

# Define ROI in Robot Frame
ROI_X = [CENTER_X - WORKSPACE_SIZE/2, CENTER_X + WORKSPACE_SIZE/2]
ROI_Y = [CENTER_Y - WORKSPACE_SIZE/2, CENTER_Y + WORKSPACE_SIZE/2]
ROI_Z = [-0.249, -0.199]

# Safety Heights
GRASP_Z = -0.25
SAFE_LIFT_HEIGHT = 0.15
NEUTRAL_JOINT_POS = [0.0, 0.175, 0.0, 2.06, 0.0, 0.0, 1.57]

# Tossing Configuration
# Optimal Wind-up found via Grid Search for 3.5m/s @ [0.95, 0.0, 0.65]
TOSS_READY_POS = [0.0, -1.01986851, 0.0, 2.0424116, 0.0, 0.57185201, 1.766]

# Point Cloud Parameters
VOXEL_SIZE = 0.005
GRID_RES = 0.005

# Auto-calculate Image Size
IMG_W = int((ROI_X[1] - ROI_X[0]) / GRID_RES)
IMG_H = int((ROI_Y[1] - ROI_Y[0]) / GRID_RES)

# Object Spawning Parameters
INSTANCES_PER_TYPE = None
MIN_OBJECT_DISTANCE = 0.08

# Training Hyperparameters
LEARNING_RATE = 1e-4
MOMENTUM = 0.9
WEIGHT_DECAY = 2e-5
BATCH_SIZE = 16
BUFFER_CAPACITY = 2000
NUM_ROTATIONS = 4

# Exploration Strategy
EXPLORE_START = 0.5
EXPLORE_END = 0.1
EXPLORE_STEPS = 1000

SAVE_INTERVAL = 50

# Grasping Parameters
TOTAL_DEG = 180

# Object Sets
ALL_OBJECTS = [
    "L_shape", "I_shape", "T_shape", "C_shape", "cross",
    "cube", "cylinder", "bar", "sphere", "puck", "bolt"
]
COMPLEX_OBJECTS = ["L_shape", "I_shape", "T_shape", "C_shape", "cross"]
SIMPLE_OBJECTS = ["cube", "cylinder", "bar", "sphere", "puck", "bolt"]
DEFAULT_TRAIN_OBJECTS = ["L_shape", "I_shape", "T_shape"]
DEFAULT_TEST_OBJECTS = None
