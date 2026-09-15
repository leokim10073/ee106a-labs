#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


class TrajectoryValidator(Node):
    def __init__(self):
        super().__init__("trajectory_validator")

        self.valid_joint_names = [
            "shoulder_pan_joint",
            "wrist_2_joint",
            "wrist_3_joint",
            "wrist_1_joint",
            "elbow_joint",
            "shoulder_lift_joint",
        ]

        self.tuck_position = [
            4.7296,
            1.5817,
            -3.1329,
            -1.4021,
            -1.4291,
            -1.8401,
        ]

        self.create_subscription(
            JointTrajectory,
            "/joint_trajectory_validated",
            self.joint_trajectory_callback,
            10,
        )

        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )

        self.pub = self.create_publisher(
            JointTrajectory,
            "/scaled_joint_trajectory_controller/joint_trajectory",
            10,
        )

        self.get_logger().info("Trajectory validator node is running!")

    def joint_trajectory_callback(self, msg: JointTrajectory):
        if msg.joint_names != self.valid_joint_names:
            self.get_logger().error(
                "Joint names do not match expected joint names!"
            )
            return

        if len(msg.points) != 1:
            self.get_logger().error(
                "Trajectory should only have one point!"
            )
            return

        point = msg.points[0]

        if len(point.positions) != 6:
            self.get_logger().error(
                "Trajectory must contain six joint positions!"
            )
            return

        if len(point.velocities) != 6:
            self.get_logger().error(
                "Trajectory must contain six joint velocities!"
            )
            return

        target_positions = point.positions

        if any(
            abs(tuck_pos - joint_pos) > 0.5
            for tuck_pos, joint_pos in zip(
                self.tuck_position,
                target_positions,
            )
        ):
            self.get_logger().error(
                "Joint positions may be unsafe!"
            )
            return

        if any(joint_vel != 0 for joint_vel in point.velocities):
            self.get_logger().error(
                "Joint velocities should be zero!"
            )
            return

        self.pub.publish(msg)

        self.get_logger().info(
            f"Published target positions: {list(target_positions)}"
        )

    def joint_state_callback(self, msg: JointState):
        positions_by_name = dict(zip(msg.name, msg.position))

        if not all(
            name in positions_by_name
            for name in self.valid_joint_names
        ):
            return

        current_positions = [
            positions_by_name[name]
            for name in self.valid_joint_names
        ]

        if any(
            abs(tuck_pos - joint_pos) > 0.5
            for tuck_pos, joint_pos in zip(
                self.tuck_position,
                current_positions,
            )
        ):
            self.get_logger().error(
                "Joint positions may be unsafe!"
            )
            return


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryValidator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()