#!/usr/bin/env python3

import sys

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JointController(Node):
    def __init__(self, joint_angles):
        super().__init__("joint_controller")

        self.joint_names = [
            "shoulder_pan_joint",
            "wrist_2_joint",
            "wrist_3_joint",
            "wrist_1_joint",
            "elbow_joint",
            "shoulder_lift_joint",
        ]

        self.joint_angles = joint_angles

        self.publisher = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_validated",
            10,
        )

        self.publish_trajectory()

    def publish_trajectory(self):
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = list(self.joint_angles)
        point.velocities = [0.0] * 6
        point.time_from_start = Duration(seconds=5).to_msg()

        trajectory.points.append(point)

        self.publisher.publish(trajectory)


def main(args=None):
    if len(sys.argv) != 7:
        sys.exit(1)

    joint_angles = [float(angle) for angle in sys.argv[1:]]

    rclpy.init(args=args)
    node = JointController(joint_angles)

    rclpy.spin_once(node, timeout_sec=2.0)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
