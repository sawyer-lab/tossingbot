
import os
import time
import argparse
import numpy as np
import xml.etree.ElementTree as ET
import mujoco
import mujoco.viewer
from scipy.spatial.transform import Rotation as R

ROBOSUITE_DIR = "/home/fausto/Projects/robosuite/robosuite/models/assets"
ROBOSUITE_SAWYER_PATH = os.path.join(ROBOSUITE_DIR, "robots/sawyer/robot.xml")
ROBOSUITE_ELECTRIC_PATH = os.path.join(ROBOSUITE_DIR, "grippers/rethink_gripper.xml")

def get_quat(rpy):
    vals = [float(x) for x in rpy.split()]
    q = R.from_euler('xyz', vals).as_quat()
    return f"{q[3]} {q[0]} {q[1]} {q[2]}"

def graft_recursive(parent_el, link_name, links_dict, joints_by_parent):
    link = links_dict.get(link_name)
    if link is None: return
    for visual in link.findall('visual'):
        mesh = visual.find('.//mesh')
        if mesh is not None:
            mesh_name = os.path.basename(mesh.get('filename'))
            origin = visual.find('origin')
            pos = origin.get('xyz', '0 0 0') if origin is not None else '0 0 0'
            rpy = origin.get('rpy', '0 0 0') if origin is not None else '0 0 0'
            ET.SubElement(parent_el, 'geom', {
                'type': 'mesh', 'mesh': mesh_name, 'pos': pos, 'quat': get_quat(rpy), 'rgba': '1 1 1 1'
            })
    for joint in joints_by_parent.get(link_name, []):
        child_el = joint.find('child')
        if child_el is None: continue
        child_link_name = child_el.get('link')
        origin = joint.find('origin')
        pos = origin.get('xyz', '0 0 0') if origin is not None else '0 0 0'
        rpy = origin.get('rpy', '0 0 0') if origin is not None else '0 0 0'
        child_body = ET.SubElement(parent_el, 'body', {'name': child_link_name, 'pos': pos, 'quat': get_quat(rpy)})
        graft_recursive(child_body, child_link_name, links_dict, joints_by_parent)

def create_scene(gripper_type="pneumatic"):
    robot_root = ET.parse(ROBOSUITE_SAWYER_PATH).getroot()
    scene = ET.Element('mujoco', {'model': 'sawyer_scene'})
    ET.SubElement(scene, 'compiler', {'angle': 'radian', 'meshdir': os.path.dirname(ROBOSUITE_SAWYER_PATH)})
    
    asset_el = ET.SubElement(scene, 'asset')
    for a in robot_root.find('asset'):
        if a.tag == 'mesh':
            a.set('inertia', 'shell')
        asset_el.append(a)
    
    worldbody = ET.SubElement(scene, 'worldbody')
    ET.SubElement(worldbody, 'light', {'pos': '0 0 3', 'dir': '0 0 -1'})
    ET.SubElement(worldbody, 'geom', {'type': 'plane', 'size': '2 2 0.1', 'rgba': '0.8 0.9 0.8 1'})
    
    for base_body in robot_root.find('worldbody').findall('body'):
        def strip_artifacts(body):
            for geom in body.findall('geom'):
                if geom.get('type') != 'mesh': body.remove(geom)
            for child in body.findall('body'): strip_artifacts(child)
        strip_artifacts(base_body)
        worldbody.append(base_body)

    right_hand = scene.find(".//body[@name='right_hand']")
    
    if gripper_type == "electric":
        elec_root = ET.parse(ROBOSUITE_ELECTRIC_PATH).getroot()
        for a in elec_root.find('asset'):
            if a.tag == 'mesh':
                f = a.get('file')
                if not f.startswith("/"): a.set('file', os.path.join(os.path.dirname(ROBOSUITE_ELECTRIC_PATH), f))
                a.set('inertia', 'shell')
            asset_el.append(a)
        gripper_body = elec_root.find(".//body[@name='gripper_base']")
        gripper_body.set('quat', '0.7071068 0 0 0.7071068')
        right_hand.append(gripper_body)
    else:
        print("Grafting Pneumatic Gripper from URDF...")
        gripper_urdf = os.path.join(os.path.dirname(os.path.abspath(__file__)), "urdf", "sawyer_tabletop_pneumatic.urdf")
        gripper_root = ET.parse(gripper_urdf).getroot()
        
        # Add assets
        for link in gripper_root.findall('.//link'):
            for visual in link.findall('visual'):
                mesh = visual.find('.//mesh')
                if mesh is not None:
                    fname = mesh.get('filename')
                    m_name = os.path.basename(fname)
                    if asset_el.find(f".//mesh[@name='{m_name}']") is None:
                        ET.SubElement(asset_el, 'mesh', {
                            'name': m_name, 'file': fname, 
                            'scale': '0.001 0.001 0.001',
                            'inertia': 'shell'
                        })
        
        links_dict = {l.get('name'): l for l in gripper_root.findall('.//link')}
        joints_by_parent = {}
        for j in gripper_root.findall('.//joint'):
            p_el = j.find('parent')
            if p_el is not None:
                p = p_el.get('link')
                if p not in joints_by_parent: joints_by_parent[p] = []
                joints_by_parent[p].append(j)
        
        pneumatic_base = ET.SubElement(right_hand, 'body', {'name': 'pneumatic_root', 'quat': '0.7071068 0 0 0.7071068'})
        # The true root link of the pneumatic gripper in our URDF is 'right_gripper_base'
        graft_recursive(pneumatic_base, "right_gripper_base", links_dict, joints_by_parent)

    actuators = ET.SubElement(scene, 'actuator')
    for act in robot_root.find('actuator'): actuators.append(act)
    return ET.tostring(scene, encoding='unicode')

def run():
    parser = argparse.ArgumentParser()
    parser.add_argument("--gripper", choices=["pneumatic", "electric"], default="pneumatic")
    args = parser.parse_args()
    xml = create_scene(args.gripper)
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)

if __name__ == "__main__":
    run()
