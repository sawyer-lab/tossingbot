import os
import numpy as np

# Gets directory: .../ros_src/tossing_system/src/tossingbot
_CONFIG_DIR = os.path.dirname(os.path.abspath(__file__))

# Go up 2 levels to: .../ros_src/tossing_system
PACKAGE_ROOT = os.path.abspath(os.path.join(_CONFIG_DIR, "../../"))

# Weights folder is in tossing_system/weights
WEIGHTS_DIR = os.path.join(PACKAGE_ROOT, "weights")
SAVE_PATH = os.path.join(WEIGHTS_DIR, "tossingbot_auto.pth")
BUFFER_PATH = os.path.join(WEIGHTS_DIR, "tossingbot_auto_buffer.pkl")

# Workstation Parameters
TABLE_HEIGHT = 0.75
WORKSPACE_SIZE = 0.40 
CENTER_X = 0.60 
CENTER_Y = 0.0

# Define ROI in Robot Frame
ROI_X = [CENTER_X - WORKSPACE_SIZE/2, CENTER_X + WORKSPACE_SIZE/2]
ROI_Y = [CENTER_Y - WORKSPACE_SIZE/2, CENTER_Y + WORKSPACE_SIZE/2]
ROI_Z = [0.0, 0.5]

# Safety Heights
GRASP_Z = 0.00       
SAFE_LIFT_HEIGHT = 0.15
NEUTRAL_JOINT_POS = [0.0, -1.27, 0.0, 2.06, 0.0, 0.0, 0.0]

# Point Cloud Parameters
VOXEL_SIZE = 0.005 # 5mm per pixel
GRID_RES = 0.005 

# Auto-calculate Image Size 
IMG_W = int((ROI_X[1] - ROI_X[0]) / GRID_RES)
IMG_H = int((ROI_Y[1] - ROI_Y[0]) / GRID_RES)

# Training Hyperparameters
LEARNING_RATE = 1e-4
MOMENTUM = 0.9
WEIGHT_DECAY = 2e-5
BATCH_SIZE = 16
BUFFER_CAPACITY = 2000
NUM_ROTATIONS = 1    

# Exploration Strategy
EXPLORE_START = 0.5   
EXPLORE_END = 0.1     
EXPLORE_STEPS = 1000

SAVE_INTERVAL = 50
WEIGHT_DECAY = 2e-5

# GRASPING PARAMETERS
TOTAL_DEG = 180
