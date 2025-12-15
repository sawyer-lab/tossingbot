import numpy as np

TABLE_HEIGHT = 0.75

# --- WORKSPACE DEFINITIONS ---
# Center of the table relative to robot base
CENTER_X = 0.60 
CENTER_Y = 0.0

# SQUARE SIZE (Critical for Rotation)
# 0.40 means a 40cm x 40cm workspace.
# This prevents data loss when rotating 90 degrees.
WORKSPACE_SIZE = 0.40 

# --- BOUNDS (Robot Frame) ---
ROI_X = [CENTER_X - WORKSPACE_SIZE/2, CENTER_X + WORKSPACE_SIZE/2] # [0.40, 0.80]
ROI_Y = [CENTER_Y - WORKSPACE_SIZE/2, CENTER_Y + WORKSPACE_SIZE/2] # [-0.20, 0.20]
ROI_Z = [-0.1, 0.5]

# --- RESOLUTION ---
# 0.005 (5mm) per pixel
# 40cm / 0.005 = 80x80 pixel image
VOXEL_SIZE = 0.005
GRID_RES = 0.005 

# --- DIMENSIONS ---
IMG_W = int((ROI_X[1] - ROI_X[0]) / GRID_RES)
IMG_H = int((ROI_Y[1] - ROI_Y[0]) / GRID_RES)

# Height to attempt grasping (Simulated table surface + object radius)
GRASP_Z = 0.00

# Sanity Check
if IMG_W != IMG_H:
    print(f"[CONFIG WARNING] Image is not square ({IMG_W}x{IMG_H}). Rotations will be distorted!")