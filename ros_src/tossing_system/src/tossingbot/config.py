import os
import numpy as np

# Gets directory: .../ros_src/tossing_system/src/tossingbot
_CONFIG_DIR = os.path.dirname(os.path.abspath(__file__))

# Go up 2 levels to: .../ros_src/tossing_system
PACKAGE_ROOT = os.path.abspath(os.path.join(_CONFIG_DIR, "../../"))

# Session Management
SESSION_BASE_DIR = os.path.join(PACKAGE_ROOT, "sessions")

# Legacy paths (for reference, not used with session system)
# WEIGHTS_DIR = os.path.join(PACKAGE_ROOT, "weights")
# LOGS_DIR = os.path.join(PACKAGE_ROOT, "logs")

# Workstation Parameters
TABLE_HEIGHT = 0.75
WORKSPACE_SIZE = 0.40 
CENTER_X = 0.60 
CENTER_Y = 0.1363

# Define ROI in Robot Frame
ROI_X = [CENTER_X - WORKSPACE_SIZE/2, CENTER_X + WORKSPACE_SIZE/2]
ROI_Y = [CENTER_Y - WORKSPACE_SIZE/2, CENTER_Y + WORKSPACE_SIZE/2]
ROI_Z = [-0.249, -0.199]  # Raised 1mm to trim table from image

# Safety Heights
GRASP_Z = -0.25
SAFE_LIFT_HEIGHT = 0.15
NEUTRAL_JOINT_POS = [0.0, -1.27, 0.0, 2.06, 0.0, 0.0, 1.57]

# Tossing Configuration
# Toss-ready position: J0=0 (will be rotated to align with target), J2=0, J4=0, J6=1.766
# J1, J3, J5 correspond to the default toss start position from TossingPlanner
# This ensures the robot is in the planar configuration before rotating J0
TOSS_READY_POS = [0.0, -0.89, 0.0, 1.57, 0.0, 0.68, 1.766]  # [J0, J1, J2, J3, J4, J5, J6]

# Point Cloud Parameters
VOXEL_SIZE = 0.005 # 5mm per pixel
GRID_RES = 0.005 

# Auto-calculate Image Size 
IMG_W = int((ROI_X[1] - ROI_X[0]) / GRID_RES)
IMG_H = int((ROI_Y[1] - ROI_Y[0]) / GRID_RES)

# Object Spawning Parameters
INSTANCES_PER_TYPE = None  # None = fill workspace, or integer for fixed count
MIN_OBJECT_DISTANCE = 0.08  # Minimum distance between objects (reduced for smaller objects)

# Training Hyperparameters
LEARNING_RATE = 1e-4
MOMENTUM = 0.9
WEIGHT_DECAY = 2e-5
BATCH_SIZE = 16
BUFFER_CAPACITY = 2000
NUM_ROTATIONS = 4    # Enable 4 rotations: 0°, 45°, 90°, 135°    

# Exploration Strategy
EXPLORE_START = 0.5   
EXPLORE_END = 0.1     
EXPLORE_STEPS = 1000

SAVE_INTERVAL = 50
WEIGHT_DECAY = 2e-5

# GRASPING PARAMETERS
TOTAL_DEG = 180

# ============================================================================
# OBJECT SET CONFIGURATION (Train/Test Split System)
# ============================================================================

# All available objects (11 total)
ALL_OBJECTS = [
    "L_shape", "I_shape", "T_shape", "C_shape", "cross",
    "cube", "cylinder", "bar", "sphere", "puck", "bolt"
]

# Object categories for easy configuration
COMPLEX_OBJECTS = ["L_shape", "I_shape", "T_shape", "C_shape", "cross"]
SIMPLE_OBJECTS = ["cube", "cylinder", "bar", "sphere", "puck", "bolt"]

# Default: Use a sensible default if user doesn't specify
DEFAULT_TRAIN_OBJECTS = ["L_shape", "I_shape", "T_shape"]
DEFAULT_TEST_OBJECTS = None
