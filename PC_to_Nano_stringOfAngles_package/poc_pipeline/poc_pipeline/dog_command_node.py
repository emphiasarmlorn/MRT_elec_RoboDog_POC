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

        self.robot = placo.RobotWrapper(
            "/home/aaravb/Downloads/urdf/robodog_urdf_visual/robodog_urdf_visual.urdf",
            placo.Flags.ignore_collisions
        )
        self.solver = placo.KinematicsSolver(self.robot)
        self.solver.mask_fbase(True)
        self.solver.enable_velocity_limits(True)

        print("Joint names:", self.robot.joint_names())

        self.robot.update_kinematics()
        for leg in ['LFfoot', 'RFfoot', 'LRfoot', 'RRfoot']:
            T = self.robot.get_T_world_frame(leg)
            print(f"{leg} position: {T[:3, 3]}")

        # --- Each leg has its own x (fixed), y_center, z_center, phase, and step_length ---
        # step_length controls how far forward (in y) each leg steps per cycle
        # step_height controls how high (in z) each leg lifts — can differ per leg if needed
        self.leg_configs = {
            'LFfoot': {
                'x': 0.56791464,
                'y_center': -1.2279557,
                'z_center': 0.67243946,
                'phase': 0.0,
                'step_length': 0.02,   # how far forward in y per cycle
                'step_height': 0.02,   # how high in z during swing
            },
            'RRfoot': {
                'x': 0.74281485,
                'y_center': -1.52181324,
                'z_center': 0.66312896,
                'phase': 0.0,          # in phase with LF (trot)
                'step_length': 0.02,
                'step_height': 0.02,
            },
            'RFfoot': {
                'x': 0.75971348,
                'y_center': -1.26101703,
                'z_center': 0.67581485,
                'phase': np.pi,        # opposite phase to LF (trot)
                'step_length': 0.02,
                'step_height': 0.02,
            },
            'LRfoot': {
                'x': 0.56492413,
                'y_center': -1.53388919,
                'z_center': 0.64826712,
                'phase': np.pi,        # opposite phase to RR (trot)
                'step_length': 0.02,
                'step_height': 0.02,
            },
        }

        self.effector_tasks = {}
        for leg, cfg in self.leg_configs.items():
            task = self.solver.add_frame_task(leg, np.eye(4))
            task.configure(leg, "soft", 10.0, 1.0)
            self.effector_tasks[leg] = task

        self.viz = robot_viz(self.robot)
        self.t = 0.0
        self.dt = 0.01
        self.solver.dt = self.dt
        self.last_targets = []
        self.last_target_t = 0.0
        self.latest_joint_angles = [0.0] * 12

        self.timer = self.create_timer(self.dt, self.loop)
        self.get_logger().info('Dog Command Node started')

    def loop(self):
        self.t += self.dt

        for leg, cfg in self.leg_configs.items():
            phase = self.t + cfg['phase']
            sl = cfg['step_length']
            sh = cfg['step_height']

            # Cycloid in each leg's own YZ plane:
            # y: progresses forward relative to this leg's y_center
            # z: lifts relative to this leg's z_center
            # x: always fixed to this leg's x
            y = cfg['y_center'] + sl * (phase - np.sin(phase)) / (2 * np.pi)
            z = cfg['z_center'] + sh * (1 - np.cos(phase)) / 2  # /2 so it lifts by sh, not 2*sh

            target = [cfg['x'], y, z]
            self.effector_tasks[leg].T_world_frame = tf.translation_matrix(target)

        self.solver.solve(True)
        self.robot.update_kinematics()

        self.viz.display(self.robot.state.q)

        for leg in self.leg_configs:
            robot_frame_viz(self.robot, leg)

        if self.t - self.last_target_t > 0.1:
            self.last_target_t = self.t
            T = self.effector_tasks['LFfoot'].T_world_frame
            lf_target = [T[0, 3], T[1, 3], T[2, 3]]
            self.last_targets.append(lf_target)
            self.last_targets = self.last_targets[-50:]
            points_viz("targets", self.last_targets, color=0xaaff00)

        self.latest_joint_angles = [float(a) for a in self.robot.state.q[7:19]]
        self.publish_joint_angles()

        if int(self.t / self.dt) % 100 == 0:
            self.get_logger().info(f'Published joint targets: {self.latest_joint_angles}')

    def publish_joint_angles(self):
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = [
            'LFshoulder1', 'LFshoulder', 'LFknee',
            'LRshoulder1', 'LRshoulder', 'LRknee',
            'RFshoulder1', 'RFshoulder', 'RFknee',
            'RRshoulder1', 'RRshoulder', 'RRknee'
        ]
        joint_msg.position = self.latest_joint_angles
        self.pub.publish(joint_msg)

    def goal_callback(self, msg):
        self.get_logger().info(f'Received goal: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = DogCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
