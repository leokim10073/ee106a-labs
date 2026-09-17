#!/usr/bin/env python3

import os
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState

from forward_kinematics.kin_func_skeleton import forward_kinematics

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


class ForwardKinematicsNode(Node):
    def __init__(self):
        super().__init__("forward_kinematics_node")

        self.twists, self.gst0 = self.load_kinematics_description()

        self.subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )

        self.get_logger().info("Forward kinematics node started.")

    def load_kinematics_description(self):
        """
        Load q_i, omega_i, and g_st(0) from the provided flattened
        UR7e kinematics description.

        Returns:
            twists: (6, 6) ndarray with one twist per column
            gst0: (4, 4) ndarray
        """

        package_share = get_package_share_directory("forward_kinematics")
        path = os.path.join(package_share, "urdf", "ur7e_flattened.urdf")

        root = ET.parse(path).getroot()

        joints = {j.attrib["name"]: j for j in root.findall("joint")}

        # joint_data: lst[((3,3) ndarray, (3,3) ndarray)]
        joint_data = []
        for name in JOINT_NAMES:
            joint = joints[name]
            q = np.fromstring(joint.find("origin").attrib["xyz"], sep=" ")
            omega = np.fromstring(joint.find("axis").attrib["xyz"], sep=" ")
            joint_data.append((q, omega))

        zero_config = root.find("zero_configuration")

        # R: (3, 3) ndarray
        R = np.array(
            [
                np.fromstring(row.attrib["values"], sep=" ")
                for row in zero_config.find("rotation").findall("row")
            ]
        )

        # p: (3, 1) ndarray
        p = np.fromstring(
            zero_config.find("translation").attrib["xyz"],
            sep=" ",
        )

        # Build the twist xi_i = [v_i; omega_i] = [-omega_i x q_i; omega_i]
        # for each joint, then stack them as columns of a (6, 6) matrix.
        twist_columns = []
        for q, omega in joint_data:
            v = -np.cross(omega, q)
            xi_i = np.concatenate([v, omega])
            twist_columns.append(xi_i)

        twists = np.column_stack(twist_columns)

        # g_st(0) = [[R, p], [0, 1]]
        gst0 = np.eye(4)
        gst0[:3, :3] = R
        gst0[:3, 3] = p

        return twists, gst0

    def joint_state_callback(self, msg):
        """
        Compute g_st(theta) whenever a new JointState message arrives.
        """

        # msg.name and msg.position are parallel arrays, but not
        # necessarily in JOINT_NAMES order, so map by name first.
        name_to_pos = dict(zip(msg.name, msg.position))

        try:
            theta = np.array([name_to_pos[name] for name in JOINT_NAMES])
        except KeyError as e:
            self.get_logger().warn(
                f"JointState message is missing joint {e}; skipping."
            )
            return

        poe = forward_kinematics(self.twists, theta)
        gst = poe @ self.gst0

        self.get_logger().info(f"g_st(theta):\n{gst}")


def main(args=None):
    rclpy.init(args=args)

    node = ForwardKinematicsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
