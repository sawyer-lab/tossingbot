# Workspace Bounds (Robot Frame)
ROI_X = [0.45, 0.7]    
ROI_Y = [-0.3, 0.3] 
ROI_Z = [-0.1, 0.5]

# Resolution
VOXEL_SIZE = 0.005
GRID_RES = 0.005 

# Calculated Dimensions
IMG_W = int((ROI_X[1] - ROI_X[0]) / GRID_RES)
IMG_H = int((ROI_Y[1] - ROI_Y[0]) / GRID_RES)

# Grasping Height
GRASP_Z = 0.0