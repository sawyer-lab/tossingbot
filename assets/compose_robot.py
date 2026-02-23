import os
import xml.etree.ElementTree as ET
import yourdfpy

ROBOSUITE_MESH_DIR = "/home/fausto/Projects/robosuite/robosuite/models/assets/robots/sawyer/obj_meshes"

def get_robosuite_meshes(link_name):
    mapping = {'right_arm_base_link': 'base', 'right_l0': 'l0', 'head': 'head', 'right_l1': 'l1', 'right_l2': 'l2', 'right_l3': 'l3', 'right_l4': 'l4', 'right_l5': 'l5', 'right_l6': 'l6'}
    folder = mapping.get(link_name)
    if not folder: return []
    base_path = os.path.join(ROBOSUITE_MESH_DIR, folder)
    if not os.path.exists(base_path): return []
    return [os.path.join(base_path, f) for f in sorted(os.listdir(base_path)) if f.endswith(".obj")]

def clean_merge(base_path, tool_path, output_path, mounting_link='right_hand', use_robosuite=True):
    print(f"Merging {base_path} + {tool_path}")
    base_root = ET.parse(base_path).getroot()
    tool_root = ET.parse(tool_path).getroot()
    
    # Identify Tool Root
    tool_links_all = {l.get('name') for l in tool_root.findall('.//link')}
    tool_children = {j.find('child').get('link') for j in tool_root.findall('.//joint') if j.find('child') is not None}
    tool_root_link = 'right_gripper_base' if 'right_gripper_base' in tool_links_all else list(tool_links_all - tool_children)[0]
    
    new_robot = ET.Element('robot', {'name': 'sawyer'})
    
    # Materials
    common_materials = [
        ('TossingBot/Black', '0 0 0 1'), ('TossingBot/Gray', '0.5 0.5 0.5 1'),
        ('black', '0.05 0.05 0.05 1'), ('sawyer_red', '0.5 0.1 0.1 1'),
        ('darkred', '0.5 0.1 0.1 1'), ('darkgray', '0.35 0.35 0.35 1'),
    ]
    added_mats = set()
    for name, rgba in common_materials:
        mat = ET.SubElement(new_robot, 'material', {'name': name})
        ET.SubElement(mat, 'color', {'rgba': rgba})
        added_mats.add(name)

    base_link_names = {l.get('name') for l in base_root.findall('.//link')}
    base_joint_names = {j.get('name') for j in base_root.findall('.//joint')}
    
    # 1. Base Elements (Strip _2 artifacts)
    for child in list(base_root):
        if child.tag == 'link':
            name = child.get('name')
            if name.endswith('_2'): continue
            if use_robosuite:
                objs = get_robosuite_meshes(name)
                if objs:
                    for v in child.findall('visual'): child.remove(v)
                    for obj_path in objs:
                        v = ET.SubElement(child, 'visual')
                        ET.SubElement(ET.SubElement(v, 'geometry'), 'mesh', {'filename': obj_path})
            else:
                # IMPORTANT: For PyBullet, we keep the original visuals
                pass 
        elif child.tag == 'joint':
            if child.get('name').endswith('_2'): continue
        elif child.tag == 'material':
            if child.get('name') in added_mats: continue
            added_mats.add(child.get('name'))
        new_robot.append(child)
        
    # 2. Tool Elements
    for child in list(tool_root):
        if child.tag == 'link':
            if child.get('name') not in base_link_names and not child.get('name').endswith('_2'):
                new_robot.append(child)
        elif child.tag == 'joint':
            if child.get('name') not in base_joint_names and not child.get('name').endswith('_2'):
                if child.find('child').get('link') != tool_root_link:
                    new_robot.append(child)
        elif child.tag == 'material':
            if child.get('name') not in added_mats:
                new_robot.append(child)
                added_mats.add(child.get('name'))

    # 3. Cleanup: Strip non-mesh collisions and fix inertias
    for link in new_robot.findall('.//link'):
        for coll in link.findall('collision'):
            if coll.find('geometry').find('mesh') is None:
                link.remove(coll)
        
        inertial = link.find('inertial')
        if inertial is None:
            inertial = ET.SubElement(link, 'inertial')
            ET.SubElement(inertial, 'mass', {'value': '1e-04'})
            ET.SubElement(inertial, 'inertia', {'ixx': '1e-06', 'iyy': '1e-06', 'izz': '1e-06', 'ixy': '0', 'ixz': '0', 'iyz': '0'})
        else:
            mass = inertial.find('mass')
            if mass is not None and float(mass.get('value', 0)) < 1e-4: mass.set('value', '1e-04')
            inertia = inertial.find('inertia')
            if inertia is not None:
                for d in ['ixx', 'iyy', 'izz']:
                    if float(inertia.get(d, 0)) < 1e-6: inertia.set(d, '1e-06')

    # Fix specific mesh filenames
    for mesh in new_robot.findall('.//mesh'):
        f = mesh.get('filename')
        if f and 'pneumatic_finger_tip_flat.STL' in f:
            mesh.set('filename', f.replace('pneumatic_finger_tip_flat.STL', 'pneumatic_finger_tip.STL'))

    # 4. Final Connection
    conn = ET.SubElement(new_robot, 'joint', {'name': f'attachment_to_{tool_root_link}', 'type': 'fixed'})
    ET.SubElement(conn, 'parent', {'link': mounting_link})
    ET.SubElement(conn, 'child', {'link': tool_root_link})
    ET.SubElement(conn, 'origin', {'xyz': '0 0 0', 'rpy': '0 0 0'})
    
    ET.ElementTree(new_robot).write(output_path, xml_declaration=True, encoding='UTF-8')

if __name__ == "__main__":
    urdf_dir = "tossingbot/assets/urdf"
    
    # MUJOCO VERSION (With Robosuite meshes)
    clean_merge(os.path.join(urdf_dir, "sawyer_arm_only.urdf"), os.path.join(urdf_dir, "pneumatic_gripper.urdf"), os.path.join(urdf_dir, "sawyer_tabletop_pneumatic.urdf"), use_robosuite=True)
    
    # PYBULLET VERSION (Original Meshes, Cleaned)
    clean_merge(os.path.join(urdf_dir, "sawyer_arm_only.urdf"), os.path.join(urdf_dir, "pneumatic_gripper.urdf"), os.path.join(urdf_dir, "sawyer_tabletop_pneumatic_pybullet.urdf"), use_robosuite=False)
    
    clean_merge(os.path.join(urdf_dir, "sawyer_arm_only.urdf"), os.path.join(urdf_dir, "sawyer_electric_gripper.urdf"), os.path.join(urdf_dir, "sawyer_tabletop_electric.urdf"), use_robosuite=True)

