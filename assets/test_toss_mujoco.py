import os
import time
import argparse
import numpy as np
import xml.etree.ElementTree as ET
import mujoco
import mujoco.viewer
from scipy.spatial.transform import Rotation as R
from tossingbot.tossing.motion_planner import TossingPlanner
from tossingbot import config as cfg

ROBOSUITE_DIR = "/home/fausto/Projects/robosuite/robosuite/models/assets"
ROBOSUITE_SAWYER_PATH = os.path.join(ROBOSUITE_DIR, "robots/sawyer/robot.xml")

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

def create_toss_scene(gripper_urdf):
    robot_root = ET.parse(ROBOSUITE_SAWYER_PATH).getroot()
    scene = ET.Element('mujoco', {'model': 'sawyer_toss'})
    ET.SubElement(scene, 'compiler', {'angle': 'radian', 'meshdir': os.path.dirname(ROBOSUITE_SAWYER_PATH)})
    
    asset_el = ET.SubElement(scene, 'asset')
    for a in robot_root.find('asset'):
        if a.tag == 'mesh': a.set('inertia', 'shell')
        asset_el.append(a)
    
    gripper_root = ET.parse(gripper_urdf).getroot()
    for link in gripper_root.findall('.//link'):
        for visual in link.findall('visual'):
            mesh = visual.find('.//mesh')
            if mesh is not None:
                fname = mesh.get('filename')
                m_name = os.path.basename(fname)
                if asset_el.find(f".//mesh[@name='{m_name}']") is None:
                    ET.SubElement(asset_el, 'mesh', {'name': m_name, 'file': fname, 'scale': '0.001 0.001 0.001', 'inertia': 'shell'})

    worldbody = ET.SubElement(scene, 'worldbody')
    ET.SubElement(worldbody, 'light', {'pos': '0 0 3', 'dir': '0 0 -1'})
    ET.SubElement(worldbody, 'geom', {'type': 'plane', 'size': '2 2 0.1', 'rgba': '0.8 0.9 0.8 1'})
    
    for base_body in robot_root.find('worldbody').findall('body'):
        if base_body.get('name') == 'base':
            base_body.set('pos', '0 0 1.0')
            # Pedestal
            ET.SubElement(base_body, 'geom', {'type': 'cylinder', 'size': '0.1 0.5', 'pos': '0 0 -0.5', 'rgba': '0.2 0.2 0.2 1'})

        def strip_artifacts(body):
            for geom in body.findall('geom'):
                if geom.get('type') != 'mesh': body.remove(geom)
            for child in body.findall('body'): strip_artifacts(child)
        strip_artifacts(base_body)
        worldbody.append(base_body)

    right_hand = scene.find(".//body[@name='right_hand']")
    links_dict = {l.get('name'): l for l in gripper_root.findall('.//link')}
    joints_by_parent = {}
    for j in gripper_root.findall('.//joint'):
        p_el = j.find('parent')
        if p_el is not None:
            p = p_el.get('link')
            if p not in joints_by_parent: joints_by_parent[p] = []
            joints_by_parent[p].append(j)
    
    pneumatic_base = ET.SubElement(right_hand, 'body', {'name': 'pneumatic_root', 'quat': '0.7071068 0 0 0.7071068'})
    graft_recursive(pneumatic_base, "right_gripper_base", links_dict, joints_by_parent)

    # 2. Velocity Actuators (Stable Version)
    actuators = ET.SubElement(scene, 'actuator')
    for i in range(7):
        ET.SubElement(actuators, 'velocity', {
            'name': f'vel_j{i}',
            'joint': f'right_j{i}',
            'kv': '20',
            'forcerange': '-80 80',
            'ctrlrange': '-10 10'
        })

    return ET.tostring(scene, encoding='unicode')

def run_toss_sim(speed=3.5):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    gripper_urdf = os.path.join(script_dir, "urdf", "sawyer_tabletop_pneumatic.urdf")
    xml = create_toss_scene(gripper_urdf)
    
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    
    act_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f'vel_j{i}') for i in range(7)]
    tip_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "right_gripper_tip")
    
    print(f"Planning toss at {speed} m/s...")
    q0_3dof = np.array([cfg.TOSS_READY_POS[1], cfg.TOSS_READY_POS[3], cfg.TOSS_READY_POS[5]])
    # Use optimal release target found in grid search: [0.95, 0.0, 0.65]
    toss_planner = TossingPlanner(profile="express", angle_deg=45, q0=q0_3dof, xT=np.array([0.95, 0.0, 0.65]))
    sol_3d = toss_planner.get_trajectory(speed)
    sol_7d = toss_planner.map_to_7dof(
        sol_3d["Q"], sol_3d["Qd"], sol_3d["Qdd"],
        cfg.TOSS_READY_POS[0], cfg.TOSS_READY_POS[2], 
        cfg.TOSS_READY_POS[4], cfg.TOSS_READY_POS[6]
    )
    
    V_traj = sol_7d['Qd']
    dt_control = toss_planner.dt
    model.opt.timestep = 0.002 # 500Hz for stability

    with mujoco.viewer.launch_passive(model, data) as viewer:
        print("\nExecuting toss trajectory...")
        time.sleep(1.0)
        data.qpos[:7] = sol_7d['Q'][0]
        mujoco.mj_forward(model, data)
        
        for k in range(len(V_traj)):
            step_start = time.time()
            for _ in range(int(dt_control / model.opt.timestep)):
                data.ctrl[act_ids] = V_traj[k]
                mujoco.mj_step(model, data)
            viewer.sync()
            
            if k == sol_3d["index"]:
                tip_vel = data.cvel[tip_body_id][3:6]
                speed_mag = np.linalg.norm(tip_vel)
                print(f"\n>>> RELEASE POINT REACHED <<<")
                print(f"    Planned Speed: {speed:.3f} m/s")
                print(f"    Actual Speed:  {speed_mag:.3f} m/s")
                print(f"    Error:         {abs(speed_mag - speed):.3f} m/s")

            elapsed = time.time() - step_start
            if elapsed < dt_control:
                time.sleep(dt_control - elapsed)

        print("\nTrajectory finished. Holding...")
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--speed", type=float, default=1.5)
    args = parser.parse_args()
    run_toss_sim(args.speed)
