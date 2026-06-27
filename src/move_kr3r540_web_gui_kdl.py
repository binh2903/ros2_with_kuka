#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time
import json
import threading
import numpy as np
from http.server import SimpleHTTPRequestHandler, HTTPServer
import os
import PyKDL  # type: ignore

# Helper functions for 3D homogeneous transformations
def rx(theta):
    c = math.cos(theta)
    s = math.sin(theta)
    return np.array([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, c,  -s,   0.0],
        [0.0, s,   c,   0.0],
        [0.0, 0.0, 0.0, 1.0]
    ])

def ry(theta):
    c = math.cos(theta)
    s = math.sin(theta)
    return np.array([
        [ c,   0.0, s,   0.0],
        [ 0.0, 1.0, 0.0, 0.0],
        [-s,   0.0, c,   0.0],
        [ 0.0, 0.0, 0.0, 1.0]
    ])

def rz(theta):
    c = math.cos(theta)
    s = math.sin(theta)
    return np.array([
        [c,   -s,   0.0, 0.0],
        [s,    c,   0.0, 0.0],
        [0.0,  0.0, 1.0, 0.0],
        [0.0,  0.0, 0.0, 1.0]
    ])

def trans(x, y, z):
    return np.array([
        [1.0, 0.0, 0.0, x],
        [0.0, 1.0, 0.0, y],
        [0.0, 0.0, 1.0, z],
        [0.0, 0.0, 0.0, 1.0]
    ])

def rotation_matrix_to_euler(R):
    sy = math.sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0])
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2,1], R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else:
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    return math.degrees(x), math.degrees(y), math.degrees(z)

# KDL Chain construction for KUKA KR3 R540
def build_kdl_chain():
    chain = PyKDL.Chain()
    
    # Base segment: base_link -> joint_a1 origin
    joint_base = PyKDL.Joint("joint_base", PyKDL.Joint.Fixed)
    frame_base = PyKDL.Frame(PyKDL.Rotation.Identity(), PyKDL.Vector(0.0, 0.0, 0.345))
    segment_base = PyKDL.Segment("base", joint_base, frame_base)
    chain.addSegment(segment_base)
    
    # Segment 1: link_1 (joint_a1 to joint_a2 origin)
    joint1 = PyKDL.Joint("joint_a1", PyKDL.Vector(0, 0, 0), PyKDL.Vector(0, 0, -1), PyKDL.Joint.RotAxis)
    frame1 = PyKDL.Frame(PyKDL.Rotation.Identity(), PyKDL.Vector(0.020, 0.0, 0.0))
    segment1 = PyKDL.Segment("link_1", joint1, frame1)
    chain.addSegment(segment1)
    
    # Segment 2: link_2 (joint_a2 to joint_a3 origin)
    joint2 = PyKDL.Joint("joint_a2", PyKDL.Vector(0, 0, 0), PyKDL.Vector(0, 1, 0), PyKDL.Joint.RotAxis)
    frame2 = PyKDL.Frame(PyKDL.Rotation.Identity(), PyKDL.Vector(0.260, 0.0, 0.0))
    segment2 = PyKDL.Segment("link_2", joint2, frame2)
    chain.addSegment(segment2)
    
    # Segment 3: link_3 (joint_a3 to joint_a4 origin)
    joint3 = PyKDL.Joint("joint_a3", PyKDL.Vector(0, 0, 0), PyKDL.Vector(0, 1, 0), PyKDL.Joint.RotAxis)
    frame3 = PyKDL.Frame(PyKDL.Rotation.Identity(), PyKDL.Vector(0.0, 0.0, 0.020))
    segment3 = PyKDL.Segment("link_3", joint3, frame3)
    chain.addSegment(segment3)
    
    # Segment 4: link_4 (joint_a4 to joint_a5 origin)
    joint4 = PyKDL.Joint("joint_a4", PyKDL.Vector(0, 0, 0), PyKDL.Vector(-1, 0, 0), PyKDL.Joint.RotAxis)
    frame4 = PyKDL.Frame(PyKDL.Rotation.Identity(), PyKDL.Vector(0.260, 0.0, 0.0))
    segment4 = PyKDL.Segment("link_4", joint4, frame4)
    chain.addSegment(segment4)
    
    # Segment 5: link_5 (joint_a5 to joint_a6 origin)
    joint5 = PyKDL.Joint("joint_a5", PyKDL.Vector(0, 0, 0), PyKDL.Vector(0, 1, 0), PyKDL.Joint.RotAxis)
    frame5 = PyKDL.Frame(PyKDL.Rotation.Identity(), PyKDL.Vector(0.075, 0.0, 0.0))
    segment5 = PyKDL.Segment("link_5", joint5, frame5)
    chain.addSegment(segment5)
    
    # Segment 6: link_6 (joint_a6 to tool0)
    joint6 = PyKDL.Joint("joint_a6", PyKDL.Vector(0, 0, 0), PyKDL.Vector(-1, 0, 0), PyKDL.Joint.RotAxis)
    rot_tool0 = PyKDL.Rotation.RotY(math.pi / 2.0)
    frame6 = PyKDL.Frame(rot_tool0, PyKDL.Vector(0.0, 0.0, 0.0))
    segment6 = PyKDL.Segment("tool0", joint6, frame6)
    chain.addSegment(segment6)
    
    return chain

# Initialize global KDL Chain
kdl_chain = build_kdl_chain()

# Forward Kinematics solver using PyKDL
def forward_kinematics(q):
    fk_solver = PyKDL.ChainFkSolverPos_recursive(kdl_chain)
    kdl_q = PyKDL.JntArray(6)
    for i in range(6):
        kdl_q[i] = q[i]
    kdl_frame = PyKDL.Frame()
    fk_solver.JntToCart(kdl_q, kdl_frame)
    
    T = np.eye(4)
    for i in range(3):
        T[i, 3] = kdl_frame.p[i]
        for j in range(3):
            T[i, j] = kdl_frame.M[i, j]
            
    # Return structure compatible with previous code [T01, T02, ..., T_tool0]
    return [None, None, None, None, None, None, T]

# Jacobian solver using PyKDL
def compute_jacobian(q):
    jac_solver = PyKDL.ChainJntToJacSolver(kdl_chain)
    kdl_q = PyKDL.JntArray(6)
    for i in range(6):
        kdl_q[i] = q[i]
    kdl_jac = PyKDL.Jacobian(6)
    jac_solver.JntToJac(kdl_q, kdl_jac)
    
    J = np.zeros((6, 6))
    for i in range(6):
        for j in range(6):
            J[i, j] = kdl_jac[i, j]
    return J

# Inverse Kinematics solver using PyKDL Numerical Newton-Raphson Solver with Joint Limits
def solve_ik(p_target, R_target, q_init):
    fk_solver = PyKDL.ChainFkSolverPos_recursive(kdl_chain)
    ik_vel_solver = PyKDL.ChainIkSolverVel_pinv(kdl_chain)
    
    # Retrieve dynamic soft joint limits
    q_min = PyKDL.JntArray(6)
    q_max = PyKDL.JntArray(6)
    if ros2_node is not None and hasattr(ros2_node, 'soft_limits'):
        limits = ros2_node.soft_limits
    else:
        limits = [
            (-2.967, 2.967),
            (-2.967, 0.872),
            (-1.919, 2.705),
            (-3.054, 3.054),
            (-2.094, 2.094),
            (-6.108, 6.108)
        ]
    for i in range(6):
        q_min[i] = limits[i][0]
        q_max[i] = limits[i][1]
        
    ik_pos_solver = PyKDL.ChainIkSolverPos_NR_JL(kdl_chain, q_min, q_max, fk_solver, ik_vel_solver, 100, 1e-4)
    
    # Target frame construction
    target_frame = PyKDL.Frame(
        PyKDL.Rotation(
            R_target[0,0], R_target[0,1], R_target[0,2],
            R_target[1,0], R_target[1,1], R_target[1,2],
            R_target[2,0], R_target[2,1], R_target[2,2]
        ),
        PyKDL.Vector(p_target[0], p_target[1], p_target[2])
    )
    
    # Initial joints state
    kdl_q_init = PyKDL.JntArray(6)
    for i in range(6):
        kdl_q_init[i] = q_init[i]
        
    # Solve IK
    kdl_q_out = PyKDL.JntArray(6)
    result = ik_pos_solver.CartToJnt(kdl_q_init, target_frame, kdl_q_out)
    
    success = (result >= 0)
    q_solved = np.zeros(6)
    for i in range(6):
        q_solved[i] = kdl_q_out[i]
        
    return q_solved, success

def solve_ik_robust(p_target, R_target, q_init):
    # Try from current position first
    q_sol, success = solve_ik(p_target, R_target, q_init)
    if success:
        return q_sol, True
        
    # Try with small perturbations around current position
    for _ in range(10):
        q_pert = q_init + np.random.uniform(-0.3, 0.3, 6)
        q_sol, success = solve_ik(p_target, R_target, q_pert)
        if success:
            return q_sol, True
    
    # Try many diverse non-singular seed configurations
    seeds = [
        np.array([0.0, -0.5, 0.5, 0.0, 0.5, 0.0]),
        np.array([0.5, -0.2, 0.2, 0.5, 0.5, 0.0]),
        np.array([-0.5, -0.2, 0.2, -0.5, 0.5, 0.0]),
        np.array([0.0, -1.0, 1.5, 0.0, -0.5, 0.0]),
        np.array([0.0, -0.3, 0.8, 0.0, 0.8, 0.0]),
        np.array([0.0, -1.2, 1.8, 0.0, 0.3, 0.0]),
        np.array([1.0, -0.5, 0.5, 0.0, 0.5, 0.0]),
        np.array([-1.0, -0.5, 0.5, 0.0, 0.5, 0.0]),
        np.array([0.0, -0.8, 1.0, 0.0, 1.0, 0.0]),
        np.array([0.0, 0.0, 0.0, 0.0, 1.57, 0.0]),
        np.array([0.0, -0.5, 1.0, 1.57, 0.5, 0.0]),
        np.array([0.0, -0.5, 1.0, -1.57, 0.5, 0.0]),
    ]
    for seed in seeds:
        q_sol, success = solve_ik(p_target, R_target, seed)
        if success:
            return q_sol, True
    
    # Last resort: large random search
    for _ in range(20):
        q_rand = np.array([
            np.random.uniform(-2.9, 2.9),
            np.random.uniform(-2.9, 0.8),
            np.random.uniform(-1.9, 2.7),
            np.random.uniform(-3.0, 3.0),
            np.random.uniform(-2.0, 2.0),
            np.random.uniform(-6.0, 6.0)
        ])
        q_sol, success = solve_ik(p_target, R_target, q_rand)
        if success:
            return q_sol, True
            
    return q_sol, False

class WebTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('kr3r540_web_trajectory_publisher_kdl')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.q_curr = np.zeros(6)
        self.joint_names = ['joint_a1', 'joint_a2', 'joint_a3', 'joint_a4', 'joint_a5', 'joint_a6']
        self.trajectory_points = []
        self.traj_index = 0
        self.is_running = False
        self.last_web_command_time = 0.0
        
        # Soft limits for joint angles in radians
        self.soft_limits = [
            [-2.967, 2.967],
            [-2.967, 0.872],
            [-1.919, 2.705],
            [-3.054, 3.054],
            [-2.094, 2.094],
            [-6.108, 6.108]
        ]
        
        # Subscribe to joint_states to update q_curr when external nodes (like joint_state_publisher_gui) publish
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )
        
        # Publish current joint state at 20 Hz when idle
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info('Web Trajectory ROS 2 node (KDL version) initialized.')

    def joint_state_callback(self, msg):
        # Ignore external joint states when we are active (running a trajectory) or recently received web commands
        if self.is_running or (time.time() - self.last_web_command_time < 5.0):
            return
        
        any_updated = False
        new_q = np.copy(self.q_curr)
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                if idx < len(msg.position):
                    new_q[i] = msg.position[idx]
                    any_updated = True
        
        if any_updated:
            self.q_curr = new_q

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        
        if self.is_running and self.trajectory_points:
            if self.traj_index < len(self.trajectory_points):
                self.q_curr = np.array(self.trajectory_points[self.traj_index])
                self.traj_index += 1
            else:
                self.is_running = False
                self.get_logger().info('Trajectory execution completed.')
                
        msg.position = self.q_curr.tolist()
        self.publisher_.publish(msg)

    def set_joint_names(self, names):
        self.joint_names = names

    def execute_trajectory(self, q_list):
        self.trajectory_points = q_list
        self.traj_index = 0
        self.is_running = True

    def stop_trajectory(self):
        self.is_running = False

# Global reference to ROS 2 Node
ros2_node = None
web_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'web')

class WebServerHandler(SimpleHTTPRequestHandler):
    def translate_path(self, path):
        # Override path translation to serve files from the custom web folder
        relative_path = path.lstrip('/')
        if not relative_path:
            relative_path = 'index.html'
        return os.path.join(web_dir, relative_path)

    def do_GET(self):
        # Serve current joint states dynamically to sync Web with ROS 2
        if self.path == '/api/joint_states':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            
            # Compute Forward Kinematics for current q to get TCP position and orientation
            T_all = forward_kinematics(ros2_node.q_curr)
            T_tcp = T_all[-1]
            tcp_pos = T_tcp[:3, 3].tolist() # [x, y, z] in meters
            r, p, y = rotation_matrix_to_euler(T_tcp[:3, :3])
            
            response = {
                "q": ros2_node.q_curr.tolist(),
                "is_running": ros2_node.is_running,
                "tcp_pos": tcp_pos,
                "tcp_euler": [r, p, y]
            }
            self.wfile.write(json.dumps(response).encode('utf-8'))
            return

        elif self.path == '/api/limits':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            limits_deg = [[math.degrees(l[0]), math.degrees(l[1])] for l in ros2_node.soft_limits]
            self.wfile.write(json.dumps({"soft_limits": limits_deg}).encode('utf-8'))
            return

        # Serve URDF files dynamically
        elif self.path.startswith('/urdf/'):
            robot_type = self.path.split('/')[-1]
            if robot_type == 'C':
                file_path = '/home/binh/ros2_ws/src/kuka_kr3r540/urdf/kuka_kr3r540.urdf'
            else:  # B
                file_path = '/home/binh/ros2_ws/src/kr3_viewer/urdf/kr3r540.urdf'
            
            if os.path.exists(file_path):
                self.send_response(200)
                self.send_header('Content-Type', 'application/xml')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                with open(file_path, 'rb') as f:
                    self.wfile.write(f.read())
            else:
                self.send_response(404)
                self.end_headers()
            return

        # Serve package meshes dynamically
        elif self.path.startswith('/package/'):
            parts = self.path.split('/')
            package_name = parts[2]
            sub_path = '/'.join(parts[3:])
            file_path = os.path.join('/home/binh/ros2_ws/src', package_name, sub_path)
            
            if os.path.exists(file_path):
                self.send_response(200)
                if file_path.lower().endswith('.stl'):
                    self.send_header('Content-Type', 'application/octet-stream')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                with open(file_path, 'rb') as f:
                    self.wfile.write(f.read())
            else:
                self.send_response(404)
                self.end_headers()
            return

        # Default handler for HTML/JS/CSS assets
        super().do_GET()

    def do_POST(self):
        global ros2_node
        if self.path == '/api/limits':
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            data = json.loads(post_data.decode('utf-8'))
            new_limits_deg = data.get('soft_limits', [])
            if len(new_limits_deg) == 6:
                ros2_node.soft_limits = [[math.radians(l[0]), math.radians(l[1])] for l in new_limits_deg]
                response = {"status": "success", "soft_limits": new_limits_deg}
            else:
                response = {"status": "error", "message": "Invalid limits data size"}
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(response).encode('utf-8'))
            return

        elif self.path == '/api/move_to_pose':
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            data = json.loads(post_data.decode('utf-8'))
            
            tx = float(data.get('x', 450.0)) / 1000.0
            ty = float(data.get('y', 0.0)) / 1000.0
            tz = float(data.get('z', 200.0)) / 1000.0
            roll = math.radians(float(data.get('roll', 0.0)))
            pitch = math.radians(float(data.get('pitch', 90.0)))
            yaw = math.radians(float(data.get('yaw', 0.0)))
            robot_type = data.get('robot_type', 'B')
            
            if robot_type == 'C':
                ros2_node.set_joint_names(['link_1', 'link_2', 'link_3', 'link_4', 'link_5', 'link_6'])
            else:
                ros2_node.set_joint_names(['joint_a1', 'joint_a2', 'joint_a3', 'joint_a4', 'joint_a5', 'joint_a6'])

            R_target = rz(yaw)[:3, :3] @ ry(pitch)[:3, :3] @ rx(roll)[:3, :3]
            p_target = np.array([tx, ty, tz])
            
            ros2_node.last_web_command_time = time.time()
            q_solved, success = solve_ik_robust(p_target, R_target, ros2_node.q_curr)
            
            if success:
                q_start = np.copy(ros2_node.q_curr)
                q_list = []
                for step in range(41):
                    ratio = step / 40.0
                    q_list.append(((1.0 - ratio) * q_start + ratio * q_solved).tolist())
                ros2_node.execute_trajectory(q_list)
                response = {"status": "success", "message": "Moving to target pose", "q": q_solved.tolist()}
            else:
                response = {"status": "error", "message": "IK Solver failed to find a valid solution"}
                
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(response).encode('utf-8'))
            return

        elif self.path == '/api/move_to_joints':
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            data = json.loads(post_data.decode('utf-8'))
            
            target_joints_deg = data.get('joints', [])
            robot_type = data.get('robot_type', 'B')
            
            if len(target_joints_deg) == 6:
                q_target = np.array([math.radians(j) for j in target_joints_deg])
                
                if robot_type == 'C':
                    ros2_node.set_joint_names(['link_1', 'link_2', 'link_3', 'link_4', 'link_5', 'link_6'])
                else:
                    ros2_node.set_joint_names(['joint_a1', 'joint_a2', 'joint_a3', 'joint_a4', 'joint_a5', 'joint_a6'])
                
                # Interpolate 15 steps for direct joint control
                ros2_node.last_web_command_time = time.time()
                q_start = np.copy(ros2_node.q_curr)
                q_list = []
                for step in range(16):
                    ratio = step / 15.0
                    q_list.append(((1.0 - ratio) * q_start + ratio * q_target).tolist())
                ros2_node.execute_trajectory(q_list)
                response = {"status": "success", "message": "Moving to manual joints"}
            else:
                response = {"status": "error", "message": "Invalid joints data size"}
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(response).encode('utf-8'))
            return

        elif self.path == '/api/execute_joints':
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            data = json.loads(post_data.decode('utf-8'))
            
            trajectory_deg = data.get('trajectory', [])
            robot_type = data.get('robot_type', 'B')
            
            if trajectory_deg:
                # Interpolate between each taught point for smooth transition
                ros2_node.last_web_command_time = time.time()
                q_list = []
                q_prev = np.copy(ros2_node.q_curr)
                
                for idx, step_deg in enumerate(trajectory_deg):
                    q_target = np.array([math.radians(j) for j in step_deg])
                    if idx > 0:
                        steps = 20
                        for s in range(steps):
                            ratio = s / float(steps)
                            q_list.append(((1.0 - ratio) * q_prev + ratio * q_target).tolist())
                    else:
                        steps = 20
                        for s in range(steps):
                            ratio = s / float(steps)
                            q_list.append(((1.0 - ratio) * q_prev + ratio * q_target).tolist())
                    q_prev = q_target
                
                if robot_type == 'C':
                    ros2_node.set_joint_names(['link_1', 'link_2', 'link_3', 'link_4', 'link_5', 'link_6'])
                else:
                    ros2_node.set_joint_names(['joint_a1', 'joint_a2', 'joint_a3', 'joint_a4', 'joint_a5', 'joint_a6'])
                
                ros2_node.execute_trajectory(q_list)
                response = {"status": "success", "message": "Executing taught trajectory"}
            else:
                response = {"status": "error", "message": "Empty trajectory"}
                
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(response).encode('utf-8'))
            return

        elif self.path == '/api/execute' or self.path == '/api/preview':
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            data = json.loads(post_data.decode('utf-8'))
            
            points_2d = data.get('points', [])
            plane_x = float(data.get('plane_x', 0.0))
            plane_y = float(data.get('plane_y', 0.0))
            plane_z = float(data.get('plane_z', 0.0))
            plane_roll = math.radians(float(data.get('plane_roll', 90.0)))
            plane_pitch = math.radians(float(data.get('plane_pitch', 0.0)))
            plane_yaw = math.radians(float(data.get('plane_yaw', 0.0)))
            robot_type = data.get('robot_type', 'B') # 'B' (joint_a1-6) or 'C' (link_1-6)
            speed_factor = float(data.get('speed', 1.0)) # 1.0 = normal, higher = faster
            
            ros2_node.last_web_command_time = time.time()
            
            # Configure joint names
            if robot_type == 'C':
                ros2_node.set_joint_names(['link_1', 'link_2', 'link_3', 'link_4', 'link_5', 'link_6'])
            else:
                ros2_node.set_joint_names(['joint_a1', 'joint_a2', 'joint_a3', 'joint_a4', 'joint_a5', 'joint_a6'])

            # Compute Work Plane to Robot Base transformation matrix T_base_plane
            T_base_plane = (trans(plane_x, plane_y, plane_z) @ 
                            rz(plane_yaw) @ 
                            ry(plane_pitch) @ 
                            rx(plane_roll))
            
            # Compute target EEF orientation from plane normal
            plane_z_axis = T_base_plane[:3, 2]  # normal of the plane
            plane_x_axis = T_base_plane[:3, 0]  # X direction on plane
            plane_y_axis = T_base_plane[:3, 1]  # Y direction on plane
            R_target = np.column_stack([plane_x_axis, plane_y_axis, plane_z_axis])
            
            # Generate Cartesian 3D waypoints in robot base frame
            q_list = []
            q_prev = np.copy(ros2_node.q_curr)
            ik_failures = 0
            
            for pt in points_2d:
                # Local coordinate in work plane frame (pt[0] = x_local, pt[1] = y_local, z_local = 0)
                p_local = np.array([pt[0], pt[1], 0.0, 1.0])
                p_base = T_base_plane @ p_local
                
                # Solve IK with robust solver
                q_solved, success = solve_ik_robust(p_base[:3], R_target, q_prev)
                if not success:
                    ik_failures += 1
                q_list.append(q_solved.tolist())
                q_prev = q_solved
            
            # Interpolate for smoother transition (velocity check)
            interpolated_q_list = []
            if len(q_list) > 1:
                # Distance-based interpolation
                steps_per_segment = max(1, int(10 / speed_factor))
                for i in range(len(q_list) - 1):
                    q_start = np.array(q_list[i])
                    q_end = np.array(q_list[i+1])
                    for step in range(steps_per_segment):
                        ratio = step / steps_per_segment
                        q_interp = (1.0 - ratio) * q_start + ratio * q_end
                        interpolated_q_list.append(q_interp.tolist())
                interpolated_q_list.append(q_list[-1])
            else:
                interpolated_q_list = q_list

            # If it's a real execution request
            if self.path == '/api/execute':
                ros2_node.execute_trajectory(interpolated_q_list)
                response = {"status": "success", "points_count": len(interpolated_q_list)}
            else: # Preview request: return joint states to let UI show simulated motions
                response = {"status": "success", "joint_trajectory": interpolated_q_list}
                
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(response).encode('utf-8'))
            
        elif self.path == '/api/stop':
            ros2_node.stop_trajectory()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "success"}).encode('utf-8'))
            
        elif self.path == '/api/home':
            # Smoothly transition to Home pose
            q_home = np.zeros(6)
            steps = 40
            q_start = np.copy(ros2_node.q_curr)
            q_list = []
            for step in range(steps + 1):
                ratio = step / float(steps)
                q_list.append(((1.0 - ratio) * q_start + ratio * q_home).tolist())
            ros2_node.execute_trajectory(q_list)
            
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "success"}).encode('utf-8'))
        else:
            self.send_response(404)
            self.end_headers()

def run_web_server():
    server_address = ('', 8080)
    httpd = HTTPServer(server_address, WebServerHandler)
    print(f"Web server running at http://localhost:8080")
    httpd.serve_forever()

def main(args=None):
    global ros2_node
    rclpy.init(args=args)
    ros2_node = WebTrajectoryPublisher()
    
    # Start Web Server in a daemon thread
    web_thread = threading.Thread(target=run_web_server, daemon=True)
    web_thread.start()
    
    try:
        rclpy.spin(ros2_node)
    except KeyboardInterrupt:
        pass
    finally:
        ros2_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
