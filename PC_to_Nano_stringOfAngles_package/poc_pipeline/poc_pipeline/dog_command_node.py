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

    ########################## NEW CODE FOR IMPLEMENTING INVERSE KINEMATICS FOR A 2-DOF LEG
    '''

    def publish_joint_targets(self, joint_angles):
        msg = JointState()

        for leg, (hip, knee) in joint_angles.items():
            msg.name.extend([f'{leg}_hip', f'{leg}_knee'])
            msg.position.extend([hip, knee])

        self.pub.publish(msg)

    def goal_to_foot_targets(self, goal):
        if goal == "stand":
            return {
                'front_left':  (0.1, 0.0, -0.2),
                'front_right': (0.1, 0.0, -0.2),
            }
        elif goal == "sit":
            return {
                'front_left':  (0.5, -0.5, -0.5),
                'front_right': (0.5, -0.5, -0.5),
            }
        else:
            return {
                'front_left':  (0.0, 0.0, 0.0),
                'front_right': (0.0, 0.0, 0.0),
            }
        
    def inverse_kinematics(self, foot_targets):
        L1 = 0.1  # thigh length (m)
        L2 = 0.1  # shin length (m)

        joint_angles = {}

        for leg, (x, y, z) in foot_targets.items():
            r = (x**2 + z**2)**0.5

            # Knee angle
            cos_knee = (r**2 - L1**2 - L2**2) / (2 * L1 * L2)
            knee = math.acos(cos_knee)

            # Hip angle
            hip = math.atan2(z, x) - math.atan2(
                L2 * math.sin(knee),
                L1 + L2 * math.cos(knee)
            )

            joint_angles[leg] = (hip, knee)

        return joint_angles
    
    def goal_callback(self, msg):
        foot_targets = self.goal_to_foot_targets(msg.data)
        joint_angles = self.inverse_kinematics(foot_targets)
        self.publish_joint_targets(joint_angles)

    '''
    ############################## END OF NEW CODE

    def goal_callback(self, msg):
        self.get_logger().info(f'Received goal: {msg.data}')

        # EXAMPLE: HARDCODED angles for POC
        joint_msg = JointState()
        joint_msg.name = [
            'front_left_hip', 'front_left_knee', # TO BE UPDATED ACCORDINGLY
            'front_right_hip', 'front_right_knee'
        ]
        joint_msg.position = [0.5, 1.0, 0.5, 1.0] # TO BE UPDATED ACCORDINGLY

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
