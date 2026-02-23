import os
import xml.etree.ElementTree as ET
import yourdfpy

def clean_merge(base_path, tool_path, output_path, mounting_link='right_hand'):
    print(f"Merging {base_path} + {tool_path}")
    
    # 1. Parse
    base_tree = ET.parse(base_path)
    tool_tree = ET.parse(tool_path)
    base_root = base_tree.getroot()
    tool_root = tool_tree.getroot()
    
    # Identify Tool Root
    tool_links_all = {l.get('name') for l in tool_root.findall('.//link')}
    tool_children = set()
    for j in tool_root.findall('.//joint'):
        c = j.find('child')
        if c is not None: tool_children.add(c.get('link'))
    tool_potential_roots = tool_links_all - tool_children
    
    if 'right_gripper_base' in tool_links_all:
        tool_root_link = 'right_gripper_base'
    elif tool_potential_roots:
        tool_root_link = sorted(list(tool_potential_roots), key=len)[0]
    else:
        tool_root_link = list(tool_links_all)[0]
        
    print(f"  Tool Root detected: {tool_root_link}")
    
    # 2. Setup New Robot
    new_robot = ET.Element('robot', {'name': 'sawyer'})
    
    # Common Materials
    common_materials = [
        ('TossingBot/Black', '0 0 0 1'),
        ('TossingBot/Gray', '0.5 0.5 0.5 1'),
        ('TossingBot/Grey', '0.5 0.5 0.5 1'),
        ('TossingBot/DarkGrey', '0.2 0.2 0.2 1'),
        ('TossingBot/DarkGray', '0.2 0.2 0.2 1'),
        ('TossingBot/LightGrey', '0.8 0.8 0.8 1'),
        ('TossingBot/LightGray', '0.8 0.8 0.8 1'),
        ('TossingBot/White', '1 1 1 1'),
        ('TossingBot/Red', '1 0 0 1'),
        ('black', '0.05 0.05 0.05 1'),
        ('sawyer_red', '0.5 0.1 0.1 1'),
        ('darkred', '0.5 0.1 0.1 1'),
        ('darkgray', '0.35 0.35 0.35 1'),
        ('sawyer_gray', '0.75 0.75 0.75 1'),
    ]
    
    added_mats = set()
    for name, rgba in common_materials:
        mat = ET.SubElement(new_robot, 'material', {'name': name})
        ET.SubElement(mat, 'color', {'rgba': rgba})
        added_mats.add(name)

    # 3. Add Elements
    base_link_names = {l.get('name') for l in base_root.findall('.//link')}
    base_joint_names = {j.get('name') for j in base_root.findall('.//joint')}
    
    # From Base
    for child in list(base_root):
        if child.tag == 'material':
            if child.get('name') in added_mats: continue
            added_mats.add(child.get('name'))
        new_robot.append(child)
        
    # From Tool
    for child in list(tool_root):
        if child.tag == 'material':
            if child.get('name') in added_mats: continue
            added_mats.add(child.get('name'))
        elif child.tag == 'link':
            if child.get('name') in base_link_names: continue
        elif child.tag == 'joint':
            if child.get('name') in base_joint_names: continue
            # Remove redundant internal tool joint pointing to its own root
            if child.find('child').get('link') == tool_root_link: continue
        elif child.tag in ['transmission', 'gazebo']:
            pass # We'll add them later
        else:
            continue
        new_robot.append(child)

    # 4. Cleanup Artifacts (spheres)
    for link in new_robot.findall('.//link'):
        for coll in link.findall('collision'):
            geom = coll.find('geometry')
            if geom is not None and geom.find('sphere') is not None:
                print(f"  Removing sphere collision from {link.get('name')}")
                link.remove(coll)

    # 5. Connect
    conn_name = f"attachment_{mounting_link}_to_{tool_root_link}"
    conn = ET.SubElement(new_robot, 'joint', {'name': conn_name, 'type': 'fixed'})
    ET.SubElement(conn, 'parent', {'link': mounting_link})
    ET.SubElement(conn, 'child', {'link': tool_root_link})
    ET.SubElement(conn, 'origin', {'xyz': '0 0 0', 'rpy': '0 0 0'})
    
    # 6. Save and Verify
    ET.ElementTree(new_robot).write(output_path, xml_declaration=True, encoding='UTF-8')
    try:
        yourdfpy.URDF.load(output_path)
        print("  Verification: SUCCESS")
    except Exception as e:
        print(f"  Verification: FAILED - {e}")

if __name__ == "__main__":
    urdf_dir = "tossingbot/assets/urdf"
    
    clean_merge(
        os.path.join(urdf_dir, "sawyer_arm_only.urdf"),
        os.path.join(urdf_dir, "pneumatic_gripper.urdf"),
        os.path.join(urdf_dir, "sawyer_tabletop_pneumatic.urdf")
    )
    
    clean_merge(
        os.path.join(urdf_dir, "sawyer_arm_only.urdf"),
        os.path.join(urdf_dir, "sawyer_electric_gripper.urdf"),
        os.path.join(urdf_dir, "sawyer_tabletop_electric.urdf")
    )
