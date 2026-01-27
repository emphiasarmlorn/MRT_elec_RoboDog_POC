#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState

class DogCommandNode(Node):

    def __init__(self):
        super().__init__('dog_command_node')

        # SUBSCRIBE to high-level command
        self.subscription = self.create_subscription(
            String,
            '/dog_goal',
            self.goal_callback,
            10
        )

        # PUBLISH joint angles
        self.pub = self.create_publisher(
            JointState,
            '/joint_targets',
            10
        )

        self.get_logger().info('Dog Command Node started')

    def goal_callback(self, msg):
        self.get_logger().info(f'Received goal: {msg.data}')

        # Example: hardcoded angles for POC
        joint_msg = JointState()
        joint_msg.name = [
            'front_left_hip', 'front_left_knee',
            'front_right_hip', 'front_right_knee'
        ]
        joint_msg.position = [0.5, 1.0, 0.5, 1.0]

        self.pub.publish(joint_msg)
        self.get_logger().info('Published joint targets')


def main(args=None):
    rclpy.init(args=args)
    node = DogCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
