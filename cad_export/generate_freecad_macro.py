
import os
import argparse
import xml.etree.ElementTree as ET
import numpy as np

def euler_to_quaternion(r, p, y):
    """Convert Euler RPY to Quaternion for FreeCAD."""
    cr = np.cos(r * 0.5)
    sr = np.sin(r * 0.5)
    cp = np.cos(p * 0.5)
    sp = np.sin(p * 0.5)
    cy = np.cos(y * 0.5)
    sy = np.sin(y * 0.5)
    
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * sy
    return [qx, qy, qz, qw]

def generate_macro(urdf_path, output_macro_path):
    print(f"Parsing URDF: {urdf_path}")
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    
    # We need to resolve mesh paths relative to the URDF
    urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
    assets_dir = os.path.dirname(urdf_dir) # tossingbot/assets
    
    macro_content = [
        "import FreeCAD as App",
        "import Mesh",
        "import Part",
        "import math",
        "",
        "doc = App.activeDocument()",
        "if not doc:",
        "    doc = App.newDocument('Sawyer_Assembly')",
        "",
        "def add_link(name, mesh_path, pos, rpy, scale=(1,1,1)):",
        "    print(f'Importing {name}...')",
        "    # Import Mesh",
        "    new_obj = Mesh.insert(mesh_path, doc.Name)",
        "    new_obj.Label = name",
        "    ",
        "    # Apply Scale if needed",
        "    if scale != (1,1,1):",
        "        # Mesh objects in FreeCAD can be scaled",
        "        m = new_obj.Mesh",
        "        m.scale(scale[0], scale[1], scale[2])",
        "        new_obj.Mesh = m",
        "    ",
        "    # Apply Transform",
        "    # Note: FreeCAD uses (x,y,z, w) or Euler. URDF uses RPY.",
        "    # We'll use the placement object.",
        "    new_obj.Placement = App.Placement(App.Vector(*pos), App.Rotation(rpy[2]*180/math.pi, rpy[1]*180/math.pi, rpy[0]*180/math.pi))",
        "    doc.recompute()",
        "",
    ]

    # This is a simplification: FreeCAD doesn't natively solve URDF kinematics
    # So we calculate the "Global" transform for each link for a static pose (all 0)
    # This is better than just dumping them at the origin.
    
    # Map of link -> global transform (4x4 matrix)
    transforms = {"base_link": np.eye(4), "base": np.eye(4), "right_arm_base_link": np.eye(4)}
    
    # Very simple forward kinematics for a static (zero) pose
    # We'll process joints to find relative transforms
    joints = root.findall(".//joint")
    links = root.findall(".//link")
    
    # Iterate to build the tree (assuming they are somewhat in order or we can loop)
    for _ in range(3): # Simple multi-pass to resolve dependencies
        for joint in joints:
            parent = joint.find("parent").get("link")
            child = joint.find("child").get("link")
            if parent in transforms and child not in transforms:
                origin = joint.find("origin")
                xyz = [float(x) for x in origin.get("xyz", "0 0 0").split()]
                rpy = [float(x) for x in origin.get("rpy", "0 0 0").split()]
                
                # Build Local T
                # (Simple version using translation + rotation)
                # For a more precise version we'd use full matrix math
                transforms[child] = (parent, xyz, rpy)

    # Now generate the import commands
    for link in links:
        link_name = link.get('name')
        visual = link.find("visual")
        if visual is None: continue
        
        geom = visual.find("geometry")
        mesh_el = geom.find("mesh")
        if mesh_el is None: continue
        
        filename = mesh_el.get("filename")
        # Resolve path
        if filename.startswith("../"):
            full_path = os.path.abspath(os.path.join(urdf_dir, filename))
        else:
            # Handle package:// if any left
            full_path = filename 
            
        # Get Transform
        # For simplicity in this macro, we output the "link-to-link" chain 
        # but FreeCAD users usually prefer one flattened import if they just want a model.
        # However, we will try to place it at its static origin.
        
        # Origin offset of visual relative to link
        v_origin = visual.find("origin")
        v_xyz = [float(x) for x in v_origin.get("xyz", "0 0 0").split()] if v_origin is not None else [0,0,0]
        v_rpy = [float(x) for x in v_origin.get("rpy", "0 0 0").split()] if v_origin is not None else [0,0,0]
        
        scale_str = mesh_el.get("scale", "1 1 1")
        scale = [float(x) for x in scale_str.split()]
        
        # We'll just write the command with the path
        # Escape backslashes for windows paths if needed, though on Linux it's fine
        clean_path = full_path.replace("", "/")
        
        macro_content.append(f"add_link('{link_name}', '{clean_path}', {v_xyz}, {v_rpy}, {scale})")

    macro_content.append("print('Import Complete!')")
    
    with open(output_macro_path, 'w') as f:
        f.write("\n".join(macro_content))
    print(f"Macro saved to: {output_macro_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate FreeCAD Macro from URDF")
    parser.add_argument("--model", default="sawyer_tabletop_pneumatic.urdf")
    args = parser.parse_args()
    
    urdf_dir = "tossingbot/assets/urdf"
    export_dir = "tossingbot/cad_export"
    
    input_urdf = os.path.join(urdf_dir, args.model)
    output_macro = os.path.join(export_dir, args.model.replace(".urdf", ".FCMacro"))
    
    generate_macro(input_urdf, output_macro)
