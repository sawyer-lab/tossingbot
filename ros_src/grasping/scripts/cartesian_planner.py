#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
CasADi Cartesian Path Planner (Python 2.7 Compatible)
Integrates URDF parsing, Forward Kinematics, and Trajectory Optimization.
"""

import xml.etree.ElementTree as ET
import numpy as np
import casadi as ca
import math

# ==============================================================================
# 1. CASADI MATH UTILITIES
# ==============================================================================

def rpy_to_rot(rpy):
    """Fixed-axis RPY to Rotation Matrix"""
    roll, pitch, yaw = rpy[0], rpy[1], rpy[2]
    
    # Rx
    sx, cx = ca.sin(roll), ca.cos(roll)
    Rx = ca.vertcat(ca.horzcat(1, 0, 0), ca.horzcat(0, cx, -sx), ca.horzcat(0, sx, cx))
    
    # Ry
    sy, cy = ca.sin(pitch), ca.cos(pitch)
    Ry = ca.vertcat(ca.horzcat(cy, 0, sy), ca.horzcat(0, 1, 0), ca.horzcat(-sy, 0, cy))
    
    # Rz
    sz, cz = ca.sin(yaw), ca.cos(yaw)
    Rz = ca.vertcat(ca.horzcat(cz, -sz, 0), ca.horzcat(sz, cz, 0), ca.horzcat(0, 0, 1))
    
    return ca.mtimes([Rz, Ry, Rx])

def homogeneous(R, p):
    """Build 4x4 Homogeneous Transform"""
    T = ca.SX.eye(4)
    T[0:3, 0:3] = R
    T[0:3, 3] = p
    return T

def rodrigues(axis, theta):
    """Axis-Angle to Rotation Matrix"""
    # Normalize axis safely
    axis = axis / (ca.norm_2(axis) + 1e-16)
    x, y, z = axis[0], axis[1], axis[2]
    K = ca.vertcat(
        ca.horzcat(0, -z, y),
        ca.horzcat(z, 0, -x),
        ca.horzcat(-y, x, 0)
    )
    I = ca.SX.eye(3)
    return I + ca.sin(theta) * K + (1 - ca.cos(theta)) * ca.mtimes(K, K)

def rot_to_quat(R):
    """Rotation Matrix to Quaternion [x, y, z, w]"""
    t = R[0,0] + R[1,1] + R[2,2]
    
    # Branchless logic for stability using fmax
    w = ca.sqrt(ca.fmax(0, 1 + t)) / 2.0
    x = ca.sqrt(ca.fmax(0, 1 + R[0,0] - R[1,1] - R[2,2])) / 2.0
    y = ca.sqrt(ca.fmax(0, 1 - R[0,0] + R[1,1] - R[2,2])) / 2.0
    z = ca.sqrt(ca.fmax(0, 1 - R[0,0] - R[1,1] + R[2,2])) / 2.0
    
    x = ca.copysign(x, R[2,1] - R[1,2])
    y = ca.copysign(y, R[0,2] - R[2,0])
    z = ca.copysign(z, R[1,0] - R[0,1])
    
    return ca.vertcat(x, y, z, w)

# ==============================================================================
# 2. URDF PARSER (PYTHON 2 COMPATIBLE)
# ==============================================================================

class KinematicModel(object):
    def __init__(self, urdf_path, base_link, end_link):
        self.urdf_path = urdf_path
        self.base_link = base_link
        self.end_link = end_link
        self.joints = {}
        self.joint_names = []
        self.q_min = []
        self.q_max = []
        
        self._parse_urdf()
        self._build_casadi_functions()

    def _parse_urdf(self):
        tree = ET.parse(self.urdf_path)
        root = tree.getroot()
        
        # 1. Map links and parents
        parent_map = {} # child -> parent
        joint_map = {}  # child -> joint_xml
        
        for joint in root.findall('joint'):
            child = joint.find('child').attrib['link']
            parent = joint.find('parent').attrib['link']
            parent_map[child] = parent
            joint_map[child] = joint

        # 2. Trace chain backwards
        chain = []
        curr = self.end_link
        while curr != self.base_link:
            if curr not in parent_map:
                raise ValueError("Chain broken! Could not find parent for " + curr)
            
            j_xml = joint_map[curr]
            chain.append(j_xml)
            curr = parent_map[curr]
        
        chain.reverse() # Base -> Tip
        
        # 3. Extract Joint Details
        for j in chain:
            j_type = j.attrib['type']
            if j_type == 'fixed':
                continue # Skip fixed joints in DOF list (but keep transform logic if needed)
                
            name = j.attrib['name']
            
            # Origin
            origin = j.find('origin')
            xyz = [0,0,0]
            rpy = [0,0,0]
            if origin is not None:
                if 'xyz' in origin.attrib: xyz = [float(x) for x in origin.attrib['xyz'].split()]
                if 'rpy' in origin.attrib: rpy = [float(x) for x in origin.attrib['rpy'].split()]
            
            # Axis
            axis = j.find('axis')
            ax = [1,0,0]
            if axis is not None and 'xyz' in axis.attrib:
                ax = [float(x) for x in axis.attrib['xyz'].split()]
            
            # Limits
            limit = j.find('limit')
            lower = -np.pi
            upper = np.pi
            if limit is not None:
                if 'lower' in limit.attrib: lower = float(limit.attrib['lower'])
                if 'upper' in limit.attrib: upper = float(limit.attrib['upper'])
            
            self.joints[name] = {
                'xyz': xyz, 'rpy': rpy, 'axis': ax, 'type': j_type
            }
            self.joint_names.append(name)
            self.q_min.append(lower)
            self.q_max.append(upper)

        self.n_dof = len(self.joint_names)
        self.q_min = np.array(self.q_min)
        self.q_max = np.array(self.q_max)
        print "Parsed URDF. Found {} Active Joints.".format(self.n_dof)

    def _build_casadi_functions(self):
        # Symbolic Joint Variables
        q = ca.SX.sym('q', self.n_dof)
        
        T = ca.SX.eye(4)
        
        for i, name in enumerate(self.joint_names):
            data = self.joints[name]
            
            # Fixed Offset
            R_static = rpy_to_rot(data['rpy'])
            p_static = ca.DM(data['xyz'])
            T_static = homogeneous(R_static, p_static)
            
            # Joint Motion
            axis = ca.DM(data['axis'])
            qi = q[i]
            
            if data['type'] in ['revolute', 'continuous']:
                R_joint = rodrigues(axis, qi)
                T_joint = homogeneous(R_joint, ca.DM([0,0,0]))
            elif data['type'] == 'prismatic':
                T_joint = homogeneous(ca.SX.eye(3), axis * qi)
            else:
                T_joint = ca.SX.eye(4)
                
            T = ca.mtimes([T, T_static, T_joint])
            
        # Outputs
        p = T[0:3, 3]
        R = T[0:3, 0:3]
        quat = rot_to_quat(R)
        
        self.fk_T = ca.Function('fk_T', [q], [T])
        self.fk_pos_quat = ca.Function('fk_pos_quat', [q], [ca.vertcat(p, quat)])

# ==============================================================================
# 3. CARTESIAN PATH PLANNER
# ==============================================================================

class CasadiIKPlanner(object):
    def __init__(self, urdf_path, base_link="base", end_link="right_gripper_tip"):
        self.model = KinematicModel(urdf_path, base_link, end_link)
        
    def plan_path(self, q_start, target_pos, target_quat=[0,1,0,0], steps=20):
        """
        Generates a sequence of joint angles to move end-effector in a straight line.
        Orientation is a SOFT constraint.
        """
        opti = ca.Opti()
        
        # Variables: N_dof x Steps
        # We optimize the trajectory from k=1 to k=Steps (k=0 is fixed at q_start)
        Q = opti.variable(self.model.n_dof, steps)

        TABLE_HEIGHT = 0.02
        
        total_cost = 0
        
        # Weights
        W_pos = 1000.0  # High weight for tracking the line
        W_ori = 10.0    # Soft weight for orientation
        W_vel = 1.0     # Minimize joint motion
        W_reg = 0.01    # Regularize towards zero (optional)

        # Interpolate linear target positions
        start_pos_fk = self.model.fk_pos_quat(q_start)[0:3]
        
        # To avoid computing start_pos_fk symbolically inside python loop if possible, 
        # we treat it as numeric here for the waypoint generation
        p0 = np.array(start_pos_fk).flatten()
        pf = np.array(target_pos)
        
        prev_q = ca.DM(q_start)
        
        for k in range(steps):
            # 1. Current Joint Config
            q_k = Q[:, k]
            
            # 2. Calculate Forward Kinematics
            pose = self.model.fk_pos_quat(q_k)
            pos_k = pose[0:3]
            quat_k = pose[3:7]

            opti.subject_to(pos_k[2] >= TABLE_HEIGHT)
            
            # 3. Cartesian Linear Target for this step
            alpha = float(k + 1) / float(steps)
            pos_ref = p0 * (1 - alpha) + pf * alpha
            
            # 4. Cost: Position Error (Hard-ish requirement)
            err_pos = pos_k - ca.DM(pos_ref)
            total_cost += W_pos * ca.dot(err_pos, err_pos)
            
            # 5. Cost: Orientation Error (Soft Constraint)
            # Use dot product of quaternions: 1 - (q1.q2)^2
            # This handles the double-cover property (q and -q are same)
            # target_quat must be normalized!
            q_target = ca.DM(target_quat)
            dot_prod = ca.dot(quat_k, q_target)
            err_ori = 1.0 - (dot_prod * dot_prod)
            total_cost += W_ori * err_ori
            
            # 6. Cost: Joint Velocity (Smoothness)
            diff_q = q_k - prev_q
            total_cost += W_vel * ca.dot(diff_q, diff_q)
            
            # 7. Constraint: Joint Limits
            opti.subject_to(opti.bounded(self.model.q_min, q_k, self.model.q_max))
            
            prev_q = q_k

        # Minimize
        opti.minimize(total_cost)
        
        # Initial Guess (Linear interpolation in Joint Space)
        # Often safer than zeros
        for k in range(steps):
            opti.set_initial(Q[:, k], q_start)

        # Solver Options (IPOPT)
        opts = {
            'ipopt.print_level': 0, 
            'print_time': 0,
            'ipopt.sb': 'yes'
        }
        opti.solver('ipopt', opts)
        
        try:
            sol = opti.solve()
            # Return list of joint configurations (excluding start, including end)
            res_q = sol.value(Q)
            
            # Format output: List of lists [[j0...j6], [j0...j6]]
            path = []
            for k in range(steps):
                path.append(res_q[:, k].tolist())
            return path
            
        except RuntimeError as e:
            print "Solver failed or max iter reached. Returning partial debug info."
            return None

# ==============================================================================
# 4. HOW TO USE IN YOUR GRASPING MODULE
# ==============================================================================
if __name__ == "__main__":
    # Example Usage
    print "Initializing Planner..."
    
    # 1. Path to your Sawyer URDF (Make sure this file exists!)
    # You can get this via: rosrun xacro xacro --inorder sawyer.urdf.xacro > sawyer_generated.urdf
    urdf_file = "/home/kid/ros_ws/src/grasping/sawyer_model.urdf" 
    
    try:
        planner = CasadiIKPlanner(urdf_file)
        
        # 2. Define Start (Current Joint State)
        q_start = [0.0, -0.5, 0.0, 1.0, 0.0, 1.0, 0.0] # 7 DOF
        
        # 3. Define Target (Where to grasp)
        target_pos = [0.65, 0.1, 0.1]
        target_quat = [0, 1, 0, 0] # Downward
        
        print "Solving for path..."
        path = planner.plan_path(q_start, target_pos, target_quat, steps=10)
        
        if path:
            print "Path found with {} steps!".format(len(path))
            print "Final Config:", path[-1]
        else:
            print "Path planning failed."
            
    except Exception as e:
        print "Error during test:", e
        print "Make sure 'sawyer_generated.urdf' exists for this test to run."