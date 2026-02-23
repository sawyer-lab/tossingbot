import xml.etree.ElementTree as ET
import numpy as np
import casadi as ca
from typing import List, Dict, Tuple

class CasadiKinematics:
    def __init__(self, urdf_path: str, base_link: str, end_link: str):
        self.urdf_path = urdf_path
        self.base_link = base_link
        self.end_link = end_link
        
        # Robot properties
        self.active_joint_names: List[str] = [] # Only the 7 moving joints
        self.q_min: np.ndarray = None
        self.q_max: np.ndarray = None
        self.n_dof: int = 0
        
        # Internal chain storage (Moving + Fixed joints)
        self._chain_data = [] 
        
        # CasADi Functions
        self.fk_pos: ca.Function = None
        self.fk_rot: ca.Function = None
        self.fk_full: ca.Function = None
        
        # Initialization
        self._parse_urdf()
        self._build_functions()

    def _parse_urdf(self):
        """Parses URDF. Includes FIXED joints in geometry, excludes them from DOF."""
        tree = ET.parse(self.urdf_path)
        root = tree.getroot()
        
        parent_map = {}
        joint_map = {}
        
        for joint in root.findall('joint'):
            child = joint.find('child').attrib['link']
            parent = joint.find('parent').attrib['link']
            parent_map[child] = parent
            joint_map[child] = joint

        # Backwards trace
        chain = []
        curr = self.end_link
        while curr != self.base_link:
            if curr not in parent_map:
                raise ValueError(f"Link {curr} not found in chain to {self.base_link}")
            j_xml = joint_map[curr]
            chain.append(j_xml)
            curr = parent_map[curr]
        chain.reverse()

        # Extract Data
        q_min, q_max = [], []
        
        for j in chain:
            j_type = j.attrib['type']
            name = j.attrib['name']
            
            # Origin (XYZ/RPY) - We need this for ALL joints (Fixed or Moving)
            origin = j.find('origin')
            xyz, rpy = [0,0,0], [0,0,0]
            if origin is not None:
                if 'xyz' in origin.attrib: xyz = [float(x) for x in origin.attrib['xyz'].split()]
                if 'rpy' in origin.attrib: rpy = [float(x) for x in origin.attrib['rpy'].split()]

            # Axis (Only matters for moving joints)
            axis = j.find('axis')
            ax = [1,0,0]
            if axis is not None and 'xyz' in axis.attrib:
                ax = [float(x) for x in axis.attrib['xyz'].split()]

            # Store Joint Data
            joint_info = {
                'name': name,
                'type': j_type,
                'xyz': xyz,
                'rpy': rpy,
                'axis': ax
            }
            self._chain_data.append(joint_info)

            # --- SEPARATE ACTIVE FROM FIXED ---
            if j_type != 'fixed':
                self.active_joint_names.append(name)
                
                # Limits
                limit = j.find('limit')
                lower, upper = -np.pi, np.pi
                if limit is not None:
                    if 'lower' in limit.attrib: lower = float(limit.attrib['lower'])
                    if 'upper' in limit.attrib: upper = float(limit.attrib['upper'])
                q_min.append(lower)
                q_max.append(upper)

        self.n_dof = len(self.active_joint_names)
        self.q_min = np.array(q_min)
        self.q_max = np.array(q_max)

    def _build_functions(self):
        """Generates symbolic CasADi functions."""
        q = ca.SX.sym('q', self.n_dof)
        T = ca.SX.eye(4)
        
        dof_index = 0 # Counter for moving joints
        
        for data in self._chain_data:
            # 1. Apply Static Transform (Origin) - Happens for Fixed AND Moving
            R_static = self._rpy_to_rot(data['rpy'])
            p_static = ca.DM(data['xyz'])
            T_static = self._homogeneous(R_static, p_static)
            
            # 2. Apply Joint Motion (Only for Moving)
            T_joint = ca.SX.eye(4)
            
            if data['type'] != 'fixed':
                axis = ca.DM(data['axis'])
                qi = q[dof_index] # Grab the next variable
                
                if data['type'] in ['revolute', 'continuous']:
                    R_joint = self._rodrigues(axis, qi)
                    T_joint = self._homogeneous(R_joint, ca.DM([0,0,0]))
                elif data['type'] == 'prismatic':
                    T_joint = self._homogeneous(ca.SX.eye(3), axis * qi)
                
                dof_index += 1 # Consume one DOF
            
            # Combine: T_new = T_old * Static_Offset * Motion
            T = ca.mtimes([T, T_static, T_joint])
            
        p = T[0:3, 3]
        R = T[0:3, 0:3]
        quat = self._rot_to_quat(R)
        
        self.fk_pos = ca.Function('fk_pos', [q], [p])
        self.fk_rot = ca.Function('fk_rot', [q], [quat])
        self.fk_full = ca.Function('fk_full', [q], [ca.vertcat(p, quat)])

    # --- MATH HELPERS ---
    def _rpy_to_rot(self, rpy):
        roll, pitch, yaw = rpy
        sx, cx = ca.sin(roll), ca.cos(roll)
        Rx = ca.vertcat(ca.horzcat(1,0,0), ca.horzcat(0,cx,-sx), ca.horzcat(0,sx,cx))
        sy, cy = ca.sin(pitch), ca.cos(pitch)
        Ry = ca.vertcat(ca.horzcat(cy,0,sy), ca.horzcat(0,1,0), ca.horzcat(-sy,0,cy))
        sz, cz = ca.sin(yaw), ca.cos(yaw)
        Rz = ca.vertcat(ca.horzcat(cz,-sz,0), ca.horzcat(sz,cz,0), ca.horzcat(0,0,1))
        return ca.mtimes([Rz, Ry, Rx])

    def _homogeneous(self, R, p):
        T = ca.SX.eye(4)
        T[0:3, 0:3] = R
        T[0:3, 3] = p
        return T

    def _rodrigues(self, axis, theta):
        axis = axis / (ca.norm_2(axis) + 1e-16)
        x, y, z = axis[0], axis[1], axis[2]
        K = ca.vertcat(ca.horzcat(0,-z,y), ca.horzcat(z,0,-x), ca.horzcat(-y,x,0))
        return ca.SX.eye(3) + ca.sin(theta)*K + (1-ca.cos(theta))*ca.mtimes(K,K)

    def _rot_to_quat(self, R):
        t = R[0,0] + R[1,1] + R[2,2]
        w = ca.sqrt(ca.fmax(0, 1+t))/2.0
        x = ca.sqrt(ca.fmax(0, 1+R[0,0]-R[1,1]-R[2,2]))/2.0
        y = ca.sqrt(ca.fmax(0, 1-R[0,0]+R[1,1]-R[2,2]))/2.0
        z = ca.sqrt(ca.fmax(0, 1-R[0,0]-R[1,1]+R[2,2]))/2.0
        x = ca.copysign(x, R[2,1]-R[1,2])
        y = ca.copysign(y, R[0,2]-R[2,0])
        z = ca.copysign(z, R[1,0]-R[0,1])
        return ca.vertcat(x,y,z,w)