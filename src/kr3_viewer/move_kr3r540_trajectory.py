#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('kr3r540_trajectory_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        timer_period = 0.05  # 20 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.start_time = time.time()
        # Joint names for KR3 R540
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        self.get_logger().info('KR3 R540 Trajectory Publisher started. Publishing smooth joint trajectories...')

    def timer_callback(self):
        t = time.time() - self.start_time
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        
        # Calculate smooth sine trajectories for each joint of KR3 R540
        msg.position = [
            1.0 * math.sin(0.5 * t),          # joint_1
            0.5 * math.sin(0.3 * t),          # joint_2
            0.8 * math.sin(0.4 * t),          # joint_3
            1.2 * math.sin(0.6 * t),          # joint_4
            1.0 * math.sin(0.2 * t),          # joint_5
            1.5 * math.sin(0.8 * t)           # joint_6 
        ]
        
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
