"""
Object dimensions parsed from SDF files.
Used for visualizing grasp distributions relative to object geometry.
All dimensions in meters, representing 2D bounding box in top-down view.
"""

# Parsed from ros_src/environments/models/<object>/model.sdf
# Each object is a composite of boxes with poses relative to object center

OBJECT_DIMENSIONS = {
    "L_shape": {
        # Long part: 0.10 x 0.025 at pose (0, 0.03, 0)
        # Short part: 0.025 x 0.06 at pose (0.0375, -0.0125, 0)
        # Combined bounding box (approximate):
        "width": 0.10,   # X dimension
        "length": 0.09,  # Y dimension (0.03+0.025/2 + 0.0125+0.06/2)
        "parts": [
            {"size": [0.10, 0.025], "pose": [0, 0.03]},      # long horizontal
            {"size": [0.025, 0.06], "pose": [0.0375, -0.0125]}  # short vertical
        ]
    },
    
    "I_shape": {
        # Web: 0.08 x 0.025 at pose (0, 0, 0)
        # Left flange: 0.02 x 0.06 at pose (-0.04, 0, 0)
        # Right flange: 0.02 x 0.06 at pose (0.04, 0, 0)
        # Combined bounding box:
        "width": 0.10,   # X dimension (0.08 + 2*0.01)
        "length": 0.06,  # Y dimension
        "parts": [
            {"size": [0.08, 0.025], "pose": [0, 0]},         # web
            {"size": [0.02, 0.06], "pose": [-0.04, 0]},      # left flange
            {"size": [0.02, 0.06], "pose": [0.04, 0]}        # right flange
        ]
    },
    
    "T_shape": {
        # Stem: 0.08 x 0.025 at pose (-0.02, 0, 0)
        # Top: 0.025 x 0.08 at pose (0.035, 0, 0)
        # Combined bounding box:
        "width": 0.10,   # X dimension (-0.02-0.04 to 0.035+0.0125)
        "length": 0.08,  # Y dimension
        "parts": [
            {"size": [0.08, 0.025], "pose": [-0.02, 0]},     # stem
            {"size": [0.025, 0.08], "pose": [0.035, 0]}      # top bar
        ]
    },
    
    "cube": {
        # Simple 0.06 cube
        "width": 0.06,
        "length": 0.06,
        "parts": [
            {"size": [0.06, 0.06], "pose": [0, 0]}
        ]
    },
    
    "bar": {
        # Long thin bar: 0.12 x 0.025 (typical)
        "width": 0.12,
        "length": 0.025,
        "parts": [
            {"size": [0.12, 0.025], "pose": [0, 0]}
        ]
    },
    
    "cylinder": {
        # Diameter 0.05
        "width": 0.05,
        "length": 0.05,
        "parts": [
            {"size": [0.05, 0.05], "pose": [0, 0], "type": "cylinder"}
        ]
    },
    
    "cross": {
        # Two intersecting bars: 0.08 x 0.025 each
        "width": 0.08,
        "length": 0.08,
        "parts": [
            {"size": [0.08, 0.025], "pose": [0, 0]},         # horizontal
            {"size": [0.025, 0.08], "pose": [0, 0]}          # vertical
        ]
    }
}


def get_object_outline(object_name):
    """
    Get object outline for plotting.
    
    Returns:
        List of rectangles [(x_min, y_min, width, height), ...]
        in object-relative coordinates.
    """
    if object_name not in OBJECT_DIMENSIONS:
        return []
    
    parts = OBJECT_DIMENSIONS[object_name].get("parts", [])
    rectangles = []
    
    for part in parts:
        size = part["size"]
        pose = part["pose"]
        
        # Rectangle centered at pose
        x_min = pose[0] - size[0] / 2
        y_min = pose[1] - size[1] / 2
        rectangles.append((x_min, y_min, size[0], size[1]))
    
    return rectangles


def get_object_bounds(object_name):
    """
    Get overall bounding box for object.
    
    Returns:
        (x_min, x_max, y_min, y_max) in meters
    """
    if object_name not in OBJECT_DIMENSIONS:
        return (-0.05, 0.05, -0.05, 0.05)  # Default 10cm box
    
    dims = OBJECT_DIMENSIONS[object_name]
    w = dims["width"]
    l = dims["length"]
    
    return (-w/2, w/2, -l/2, l/2)
