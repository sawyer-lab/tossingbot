
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
    
    # Ensure robot name is consistent
    base_root.set('name', 'sawyer')
    
    # Identify Tool Root
    tool_links = {l.get('name') for l in tool_root.findall('.//link')}
    
    tool_children = set()
    for j in tool_root.findall('.//joint'):
        child_el = j.find('child')
        if child_el is not None:
            tool_children.add(child_el.get('link'))
            
    tool_potential_roots = tool_links - tool_children
    
    if 'right_gripper_base' in tool_links:
        tool_root_link = 'right_gripper_base'
    elif tool_potential_roots:
        tool_root_link = sorted(list(tool_potential_roots), key=len)[0]
    else:
        tool_root_link = list(tool_links)[0]
        
    print(f"  Tool Root detected: {tool_root_link}")
    
    # 2. Add common materials that might be missing (like TossingBot/Black)
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
        ('TossingBot/Green', '0 1 0 1'),
        ('TossingBot/Blue', '0 0 1 1'),
        ('black', '0.05 0.05 0.05 1'),
    ]
    
    # Track all material names from base and tool
    all_source_mats = {m.get('name') for m in base_root.findall('.//material')}
    all_source_mats.update({m.get('name') for m in tool_root.findall('.//material')})

    new_robot = ET.Element('robot', {'name': 'sawyer'})
    
    added_mats = set()
    for name, rgba in common_materials:
        if name not in all_source_mats:
            mat = ET.SubElement(new_robot, 'material', {'name': name})
            ET.SubElement(mat, 'color', {'rgba': rgba})
            added_mats.add(name)
        else:
            # Still track it so we don't add it again if it appears in base/tool
            added_mats.add(name)

    # Trackers
    base_link_names = {l.get('name') for l in base_root.findall('.//link')}
    base_joint_names = {j.get('name') for j in base_root.findall('.//joint')}
    
    # Add from base
    for child in list(base_root):
        if child.tag == 'material':
            name = child.get('name')
            if name in added_mats: continue
            added_mats.add(name)
        new_robot.append(child)
        
    # Add from tool
    for child in list(tool_root):
        if child.tag == 'material':
            if child.get('name') in added_mats: continue
        elif child.tag == 'link':
            if child.get('name') in base_link_names: continue
        elif child.tag == 'joint':
            if child.find('child').get('link') == tool_root_link:
                continue
            if child.get('name') in base_joint_names: continue
            
        new_robot.append(child)
        
    # 3. Connection
    conn_name = f"attachment_{mounting_link}_to_{tool_root_link}"
    conn = ET.SubElement(new_robot, 'joint', {'name': conn_name, 'type': 'fixed'})
    ET.SubElement(conn, 'parent', {'link': mounting_link})
    ET.SubElement(conn, 'child', {'link': tool_root_link})
    ET.SubElement(conn, 'origin', {'xyz': '0 0 0', 'rpy': '0 0 0'})
    
    # 4. Save and Verify
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
