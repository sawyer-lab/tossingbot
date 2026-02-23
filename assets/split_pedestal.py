
import os
import xml.etree.ElementTree as ET

def split_pedestal():
    urdf_dir = "tossingbot/assets/urdf"
    input_path = os.path.join(urdf_dir, "sawyer_arm.urdf")
    
    tree = ET.parse(input_path)
    root = tree.getroot()
    
    # Links/Joints related to pedestal
    pedestal_links = {'base', 'torso', 'pedestal', 'pedestal_feet', 'controller_box'}
    pedestal_joints = {'pedestal_fixed', 'pedestal_feet_fixed', 'controller_box_fixed', 'torso_t0'}
    
    # 1. Create Pedestal URDF
    pedestal_root = ET.Element('robot', {'name': 'sawyer_pedestal'})
    
    # Materials (Copy all)
    for mat in root.findall('material'):
        pedestal_root.append(ET.fromstring(ET.tostring(mat)))
        
    added_links = set()
    for link in root.findall('link'):
        name = link.get('name')
        if name in pedestal_links and name not in added_links:
            pedestal_root.append(ET.fromstring(ET.tostring(link)))
            added_links.add(name)
            
    added_joints = set()
    for joint in root.findall('joint'):
        name = joint.get('name')
        if name in pedestal_joints and name not in added_joints:
            pedestal_root.append(ET.fromstring(ET.tostring(joint)))
            added_joints.add(name)
            
    with open(os.path.join(urdf_dir, "sawyer_pedestal.urdf"), 'wb') as f:
        f.write(ET.tostring(pedestal_root, encoding='UTF-8', xml_declaration=True))
        
    # 2. Create Arm-Only URDF
    arm_root = ET.Element('robot', {'name': 'sawyer_arm_only'})
    
    # Materials (Copy all)
    for mat in root.findall('material'):
        arm_root.append(ET.fromstring(ET.tostring(mat)))
        
    added_links = set()
    for link in root.findall('link'):
        name = link.get('name')
        if name not in pedestal_links and name not in added_links:
            arm_root.append(ET.fromstring(ET.tostring(link)))
            added_links.add(name)
            
    added_joints = set()
    for joint in root.findall('joint'):
        name = joint.get('name')
        # Skip pedestal joints and the mount joint itself (composer will add mount)
        if name not in pedestal_joints and name != 'right_arm_mount' and name not in added_joints:
            arm_root.append(ET.fromstring(ET.tostring(joint)))
            added_joints.add(name)
            
    with open(os.path.join(urdf_dir, "sawyer_arm_only.urdf"), 'wb') as f:
        f.write(ET.tostring(arm_root, encoding='UTF-8', xml_declaration=True))
    
    print("Split into clean sawyer_pedestal.urdf and sawyer_arm_only.urdf")

if __name__ == "__main__":
    split_pedestal()
