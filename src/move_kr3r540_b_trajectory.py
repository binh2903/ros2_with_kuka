#!/usr/bin/env python3
import rclpy
import math
import time
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState

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

# Động học thuận (Forward Kinematics) cho Robot B (Kuka KR3 R540 tiêu chuẩn)
# Lưu ý: Nếu dùng robot khác có kích thước, số khớp hoặc cấu trúc động học khác,
# bạn CẦN cập nhật hàm này để tính toán đúng ma trận chuyển đổi T cho từng khớp.
def forward_kinematics(q):
    # Joint 1: base_link -> link_1. origin [0, 0, 0.345], axis [0, 0, -1]
    T01 = trans(0.0, 0.0, 0.345) @ rz(-q[0])
    
    # Joint 2: link_1 -> link_2. origin [0.020, 0, 0], axis [0, 1, 0]
    T12 = trans(0.020, 0.0, 0.0) @ ry(q[1])
    T02 = T01 @ T12
    
    # Joint 3: link_2 -> link_3. origin [0.260, 0, 0], axis [0, 1, 0]
    T23 = trans(0.260, 0.0, 0.0) @ ry(q[2])
    T03 = T02 @ T23
    
    # Joint 4: link_3 -> link_4. origin [0, 0, 0.020], axis [-1, 0, 0]
    T34 = trans(0.0, 0.0, 0.020) @ rx(-q[3])
    T04 = T03 @ T34
    
    # Joint 5: link_4 -> link_5. origin [0.260, 0, 0], axis [0, 1, 0]
    T45 = trans(0.260, 0.0, 0.0) @ ry(q[4])
    T05 = T04 @ T45
    
    # Joint 6: link_5 -> link_6. origin [0.075, 0, 0], axis [-1, 0, 0]
    T56 = trans(0.075, 0.0, 0.0) @ rx(-q[5])
    T06 = T05 @ T56
    
    # tool0: cố định so với flange (link_6). origin rpy="0 1.5707963267948966 0" xyz="0 0 0"
    T_flange_tool0 = ry(math.pi / 2.0)
    T0_tool0 = T06 @ T_flange_tool0
    
    return [T01, T02, T03, T04, T05, T06, T0_tool0]

# Tính toán ma trận Jacobian 6D
def compute_jacobian(q):
    T01, T02, T03, T04, T05, T06, T0_tool0 = forward_kinematics(q)
    p = T0_tool0[:3, 3]
    
    # Trục xoay cục bộ của các khớp (định nghĩa từ file URDF)
    u = [
        np.array([0.0, 0.0, -1.0]),
        np.array([0.0, 1.0, 0.0]),
        np.array([0.0, 1.0, 0.0]),
        np.array([-1.0, 0.0, 0.0]),
        np.array([0.0, 1.0, 0.0]),
        np.array([-1.0, 0.0, 0.0])
    ]
    
    # Hệ tọa độ của khớp so với gốc base
    R = [
        np.eye(3),
        T01[:3, :3],
        T02[:3, :3],
        T03[:3, :3],
        T04[:3, :3],
        T05[:3, :3]
    ]
    
    o = [
        np.array([0.0, 0.0, 0.345]),
        T01[:3, 3],
        T02[:3, 3],
        T03[:3, 3],
        T04[:3, 3],
        T05[:3, 3]
    ]
    
    J = np.zeros((6, 6))
    for i in range(6):
        a_i = R[i] @ u[i]
        J[:3, i] = np.cross(a_i, p - o[i])
        J[3:, i] = a_i
        
    return J

def compute_orientation_error(R_curr, R_target):
    err = np.zeros(3)
    for i in range(3):
        err += np.cross(R_curr[:, i], R_target[:, i])
    return 0.5 * err

# Động học ngược (Inverse Kinematics) sử dụng Damped Least Squares
def solve_ik(p_target, R_target, q_init, max_iters=50, tol=1e-4):
    q = np.array(q_init, dtype=float)
    damping = 0.02
    
    for _ in range(max_iters):
        T_all = forward_kinematics(q)
        T0_tool = T_all[-1]
        p_curr = T0_tool[:3, 3]
        R_curr = T0_tool[:3, :3]
        
        pos_error = p_target - p_curr
        rot_error = compute_orientation_error(R_curr, R_target)
        
        error = np.concatenate([pos_error, rot_error])
        if np.linalg.norm(error) < tol:
            return q, True
            
        J = compute_jacobian(q)
        
        JJT = J @ J.T + (damping ** 2) * np.eye(6)
        dq = J.T @ np.linalg.solve(JJT, error)
        
        step_limit = 0.3
        step_norm = np.linalg.norm(dq)
        if step_norm > step_limit:
            dq = dq * (step_limit / step_norm)
            
        q += dq
        
        # Giới hạn khớp của Robot B (KR3 R540)
        # Nếu đổi sang robot khác, bạn CẦN cập nhật các giới hạn góc xoay (Radian) này
        limits = [
            (-2.967, 2.967),
            (-2.967, 0.872),
            (-1.919, 2.705),
            (-3.054, 3.054),
            (-2.094, 2.094),
            (-6.108, 6.108)
        ]
        for i in range(6):
            q[i] = np.clip(q[i], limits[i][0], limits[i][1])
            
    return q, False

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('kr3r540_b_trajectory_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.start_time = time.time()
        
        # =========================================================================
        # PHẦN QUAN TRỌNG NHẤT KHI ĐỔI SANG ROBOT KHÁC:
        # Tên của các khớp (joint names) trong mảng dưới đây phải TRÙNG KHỚP HOÀN TOÀN 
        # với tên các khớp (revolute / continuous) được định nghĩa trong file URDF.
        # - Robot B (kr3_viewer/urdf/kr3r540.urdf) sử dụng các tên:
        #   ['joint_a1', 'joint_a2', 'joint_a3', 'joint_a4', 'joint_a5', 'joint_a6']
        # =========================================================================
        self.joint_names = ['joint_a1', 'joint_a2', 'joint_a3', 'joint_a4', 'joint_a5', 'joint_a6']
        
        # Tham số quỹ đạo
        self.center = np.array([0.45, 0.0, 0.45])
        self.square_side = 0.15
        self.circle_radius = 0.1
        
        _, _, _, _, _, _, T_home = forward_kinematics(np.zeros(6))
        self.R_target = T_home[:3, :3]
        
        self.q_prev = np.array([0.0, -0.5, 1.0, 0.0, 0.5, 0.0])
        
        # Các đỉnh của hình vuông
        half_side = self.square_side / 2.0
        self.P1 = self.center + np.array([0.0, -half_side, -half_side])
        self.P2 = self.center + np.array([0.0,  half_side, -half_side])
        self.P3 = self.center + np.array([0.0,  half_side,  half_side])
        self.P4 = self.center + np.array([0.0, -half_side,  half_side])
        
        # Tính trước quỹ đạo góc khớp tại các góc (Move J)
        self.get_logger().info('Precomputing joint coordinates for Robot B square corners...')
        self.Q1, _ = solve_ik(self.P1, self.R_target, self.q_prev)
        self.Q2, _ = solve_ik(self.P2, self.R_target, self.Q1)
        self.Q3, _ = solve_ik(self.P3, self.R_target, self.Q2)
        self.Q4, _ = solve_ik(self.P4, self.R_target, self.Q3)
        
        # Tính trước quỹ đạo góc khớp cho hình tròn (Move J)
        self.get_logger().info('Precomputing joint coordinates for Robot B circle waypoints...')
        C1 = self.center + np.array([0.0, self.circle_radius, 0.0])
        C2 = self.center + np.array([0.0, 0.0, self.circle_radius])
        C3 = self.center + np.array([0.0, -self.circle_radius, 0.0])
        C4 = self.center + np.array([0.0, 0.0, -self.circle_radius])
        
        self.QC1, _ = solve_ik(C1, self.R_target, self.q_prev)
        self.QC2, _ = solve_ik(C2, self.R_target, self.QC1)
        self.QC3, _ = solve_ik(C3, self.R_target, self.QC2)
        self.QC4, _ = solve_ik(C4, self.R_target, self.QC3)
        
        self.MODES = [
            'MOVE_L_SQUARE',
            'MOVE_J_SQUARE',
            'MOVE_L_CIRCLE',
            'MOVE_J_CIRCLE'
        ]
        self.current_mode_idx = 0
        self.phase_duration = 10.0
        self.phase_start_time = time.time()
        
        self.log_mode_transition()

    def log_mode_transition(self):
        mode_name = self.MODES[self.current_mode_idx]
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'ROBOT B - MODE: {mode_name}')
        self.get_logger().info('=' * 60)
        self.get_logger().info('')

    def timer_callback(self):
        t_now = time.time()
        t_phase = t_now - self.phase_start_time
        
        if t_phase >= self.phase_duration:
            self.current_mode_idx = (self.current_mode_idx + 1) % len(self.MODES)
            self.phase_start_time = t_now
            t_phase = 0.0
            self.log_mode_transition()
            
        mode_name = self.MODES[self.current_mode_idx]
        q = np.copy(self.q_prev)
        
        if mode_name == 'MOVE_L_SQUARE':
            s = (t_phase / self.phase_duration) * 4.0
            if s < 1.0:
                u = s
                P = (1.0 - u) * self.P1 + u * self.P2
            elif s < 2.0:
                u = s - 1.0
                P = (1.0 - u) * self.P2 + u * self.P3
            elif s < 3.0:
                u = s - 2.0
                P = (1.0 - u) * self.P3 + u * self.P4
            else:
                u = s - 3.0
                P = (1.0 - u) * self.P4 + u * self.P1
            q, success = solve_ik(P, self.R_target, self.q_prev)
            
        elif mode_name == 'MOVE_J_SQUARE':
            s = (t_phase / self.phase_duration) * 4.0
            if s < 1.0:
                u = s
                q = (1.0 - u) * self.Q1 + u * self.Q2
            elif s < 2.0:
                u = s - 1.0
                q = (1.0 - u) * self.Q2 + u * self.Q3
            elif s < 3.0:
                u = s - 2.0
                q = (1.0 - u) * self.Q3 + u * self.Q4
            else:
                u = s - 3.0
                q = (1.0 - u) * self.Q4 + u * self.Q1
                
        elif mode_name == 'MOVE_L_CIRCLE':
            theta = (t_phase / self.phase_duration) * 2.0 * math.pi
            P = self.center + np.array([
                0.0,
                self.circle_radius * math.cos(theta),
                self.circle_radius * math.sin(theta)
            ])
            q, success = solve_ik(P, self.R_target, self.q_prev)
            
        elif mode_name == 'MOVE_J_CIRCLE':
            s = (t_phase / self.phase_duration) * 4.0
            if s < 1.0:
                u = s
                q = (1.0 - u) * self.QC1 + u * self.QC2
            elif s < 2.0:
                u = s - 1.0
                q = (1.0 - u) * self.QC2 + u * self.QC3
            elif s < 3.0:
                u = s - 2.0
                q = (1.0 - u) * self.QC3 + u * self.QC4
            else:
                u = s - 3.0
                q = (1.0 - u) * self.QC4 + u * self.QC1
                
        self.q_prev = q
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = q.tolist()
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
