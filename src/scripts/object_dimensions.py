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
        # Stem (horizontal): 0.12 x 0.025 at pose (0, 0, 0)
        # Crossbar (vertical): 0.025 x 0.08 at pose (0.03, 0, 0)
        "width": 0.12,   # X dimension (stem length)
        "length": 0.08,  # Y dimension (crossbar length)
        "parts": [
            {"size": [0.12, 0.025], "pose": [0, 0]},          # horizontal stem
            {"size": [0.025, 0.08], "pose": [0.03, 0]}        # vertical crossbar (offset right)
        ]
    },
    
    "C_shape": {
        # Spine (vertical): 0.025 x 0.10 at pose (-0.02, 0, 0)
        # Top arm: 0.06 x 0.025 at pose (0.0225, 0.0375, 0)
        # Bottom arm: 0.06 x 0.025 at pose (0.0225, -0.0375, 0)
        "width": 0.08,   # X dimension (-0.02-0.0125 to 0.0225+0.03)
        "length": 0.10,  # Y dimension (full spine length)
        "parts": [
            {"size": [0.025, 0.10], "pose": [-0.02, 0]},           # spine
            {"size": [0.06, 0.025], "pose": [0.0225, 0.0375]},     # top arm
            {"size": [0.06, 0.025], "pose": [0.0225, -0.0375]}     # bottom arm
        ]
    },
    
    "puck": {
        # Cylinder: radius 0.03, height 0.035
        "width": 0.06,   # Diameter
        "length": 0.06,
        "parts": [
            {"size": [0.06, 0.06], "pose": [0, 0], "type": "cylinder"}
        ]
    },
    
    "bolt": {
        # Three overlapping boxes at different rotations (60 degrees apart)
        # Each box: 0.025 x 0.045 x 0.08 (viewed from top: 0.025 x 0.045)
        # Creates hexagonal-ish bolt head in top view
        # Approximate as circle with diameter ~0.05
        "width": 0.05,
        "length": 0.05,
        "parts": [
            {"size": [0.05, 0.05], "pose": [0, 0], "type": "circle"}
        ]
    },
    
    "sphere": {
        # Radius 0.0175
        "width": 0.035,
        "length": 0.035,
        "parts": [
            {"size": [0.035, 0.035], "pose": [0, 0], "type": "circle"}
        ]
    },
    
    # ==========================================
    # SMALL VERSIONS (75% scale of originals)
    # ==========================================
    
    "L_shape_small": {
        "width": 0.075,   # 0.10 * 0.75
        "length": 0.0675,  # 0.09 * 0.75
        "parts": [
            {"size": [0.075, 0.01875], "pose": [0, 0.0225]},
            {"size": [0.01875, 0.045], "pose": [0.028125, -0.009375]}
        ]
    },
    
    "I_shape_small": {
        "width": 0.075,   # 0.10 * 0.75
        "length": 0.045,  # 0.06 * 0.75
        "parts": [
            {"size": [0.06, 0.01875], "pose": [0, 0]},
            {"size": [0.015, 0.045], "pose": [-0.03, 0]},
            {"size": [0.015, 0.045], "pose": [0.03, 0]}
        ]
    },
    
    "T_shape_small": {
        "width": 0.075,   # 0.10 * 0.75
        "length": 0.06,   # 0.08 * 0.75
        "parts": [
            {"size": [0.06, 0.01875], "pose": [-0.015, 0]},
            {"size": [0.01875, 0.06], "pose": [0.02625, 0]}
        ]
    },
    
    "C_shape_small": {
        "width": 0.06,    # 0.08 * 0.75
        "length": 0.075,  # 0.10 * 0.75
        "parts": [
            {"size": [0.01875, 0.075], "pose": [-0.015, 0]},
            {"size": [0.045, 0.01875], "pose": [0.016875, 0.028125]},
            {"size": [0.045, 0.01875], "pose": [0.016875, -0.028125]}
        ]
    },
    
    "cube_small": {
        "width": 0.045,   # 0.06 * 0.75
        "length": 0.045,
        "parts": [
            {"size": [0.045, 0.045], "pose": [0, 0]}
        ]
    },
    
    "bar_small": {
        "width": 0.09,    # 0.12 * 0.75
        "length": 0.01875,  # 0.025 * 0.75
        "parts": [
            {"size": [0.09, 0.01875], "pose": [0, 0]}
        ]
    },
    
    "cross_small": {
        "width": 0.09,    # 0.12 * 0.75
        "length": 0.06,   # 0.08 * 0.75
        "parts": [
            {"size": [0.09, 0.01875], "pose": [0, 0]},
            {"size": [0.01875, 0.06], "pose": [0.0225, 0]}
        ]
    }
}


def get_object_outline(object_name):
    """
    Get object outline for plotting.
    
    Returns:
        List of parts: [{'type': 'rect'|'circle', 'data': ...}, ...]
        For rectangles: data = (x_min, y_min, width, height)
        For circles: data = (center_x, center_y, radius)
    """
    if object_name not in OBJECT_DIMENSIONS:
        return []
    
    parts = OBJECT_DIMENSIONS[object_name].get("parts", [])
    outlines = []
    
    for part in parts:
        size = part["size"]
        pose = part["pose"]
        part_type = part.get("type", "box")
        
        if part_type in ["cylinder", "circle"]:
            # Circle/cylinder: size[0] is diameter
            radius = size[0] / 2
            outlines.append({
                'type': 'circle',
                'data': (pose[0], pose[1], radius)
            })
        else:
            # Rectangle: centered at pose
            x_min = pose[0] - size[0] / 2
            y_min = pose[1] - size[1] / 2
            outlines.append({
                'type': 'rect',
                'data': (x_min, y_min, size[0], size[1])
            })
    
    return outlines


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
