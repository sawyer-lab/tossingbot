
import os
import xml.etree.ElementTree as ET

def make_paths_absolute(urdf_path):
    print(f"Fixing paths in {urdf_path}...")
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    
    urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
    
    count = 0
    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename")
        if filename.startswith("../"):
            # Resolve relative to URDF dir
            abs_path = os.path.abspath(os.path.join(urdf_dir, filename))
            mesh.set("filename", abs_path)
            count += 1
            
    if count > 0:
        tree.write(urdf_path, xml_declaration=True, encoding='UTF-8')
        print(f"  Updated {count} mesh paths to absolute.")
    else:
        print("  No relative paths found.")

if __name__ == "__main__":
    urdf_dir = "tossingbot/assets/urdf"
    for f in os.listdir(urdf_dir):
        if f.endswith(".urdf"):
            make_paths_absolute(os.path.join(urdf_dir, f))
