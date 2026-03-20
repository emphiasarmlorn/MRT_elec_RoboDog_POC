#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import placo
import numpy as np
from placo_utils.visualization import robot_viz, robot_frame_viz, frame_viz, points_viz
from placo_utils.tf import tf

class DogCommandNode(Node):
    def __init__(self):
        super().__init__('dog_command_node')

        self.subscription = self.create_subscription(
            String, '/dog_goal', self.goal_callback, 10
        )
        self.pub = self.create_publisher(JointState, '/joint_targets', 10)

        # Loading the robot
        self.robot = placo.RobotWrapper("/home/aaravb/Downloads/testing.urdf", placo.Flags.ignore_collisions)
        self.solver = placo.KinematicsSolver(self.robot)
        self.solver.mask_fbase(True)

        self.effector_task = self.solver.add_frame_task("foot", np.eye(4))
        self.effector_task.configure("foot", "soft", 10.0, 1.0)
        self.solver.enable_velocity_limits(True)

        self.viz = robot_viz(self.robot)
        self.t = 0.0
        self.dt = 0.01
        self.solver.dt = self.dt
        self.last_targets = []
        self.last_target_t = 0.0
        self.latest_joint_angles = [0.0, 0.0, 0.0, 0.0]

        # ROS timer replaces @schedule + run_loop
        self.timer = self.create_timer(self.dt, self.loop)

        self.get_logger().info('Dog Command Node started')

    def loop(self):
        self.t += self.dt

        target = [(self.t - np.sin(self.t)) / 50, 0.1, (1 - np.cos(self.t)) / 50 - 0.3]
        self.effector_task.T_world_frame = tf.translation_matrix(target)

        self.solver.solve(True)
        self.robot.update_kinematics()

        self.viz.display(self.robot.state.q)
        robot_frame_viz(self.robot, "foot")
        frame_viz("target", self.effector_task.T_world_frame)

        if self.t - self.last_target_t > 0.1:
            self.last_target_t = self.t
            self.last_targets.append(target)
            self.last_targets = self.last_targets[-50:]
            points_viz("targets", self.last_targets, color=0xaaff00)

        # Update latest joint angles
        self.latest_joint_angles = [float(a) for a in self.robot.state.q[7:10]] + [0.0]

        # Publish joint angles as list
        self.publish_joint_angles()  # called every tick

        self.get_logger().info(f'Published joint targets: {self.latest_joint_angles}')

        # COMMENTED ANGLES PRINTING CODE
        # for joint_name, angle in zip(self.robot.joint_names()[:4], self.robot.state.q[7:11]):
        #     print(f"{joint_name}: {angle:.2f}", end=", ")
        # print()

    def publish_joint_angles(self):
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = ['a1', 'a2', 'a3', 'b1']
        joint_msg.position = self.latest_joint_angles
        self.pub.publish(joint_msg)

    def goal_callback(self, msg):
        # Now handles high-level commands only, not publishing
        self.get_logger().info(f'Received goal: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = DogCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
