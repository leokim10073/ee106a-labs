#!/usr/bin/env python3

import math

import numpy as np
import rclpy
import tf2_ros
import transforms3d.euler as euler
from geometry_msgs.msg import PoseArray, Twist
from rclpy.node import Node
from std_srvs.srv import Trigger

from plannedcntrl.trajectory import plan_trajectory, sample_trajectory

OFFSET_L = 0.10
GOAL_TOL = 0.05
MAX_LINEAR = 0.22
MAX_ANGULAR = 2.0
CONTROL_PERIOD = 0.05

# Plan once: the cones leave the camera's view as soon as the robot turns.
SNAPSHOT_FRAMES = 10
CLUSTER_RADIUS = 0.25
MIN_SIGHTING_FRACTION = 0.5


class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')

        self.declare_parameter('path_mode', 'bezier')   # 'straight' or 'bezier'
        self.declare_parameter('k1', 1.0)
        self.declare_parameter('k2', 1.0)
        self.declare_parameter('offset_l', OFFSET_L)

        self.path_mode = self.get_parameter('path_mode').value
        self.k1 = self.get_parameter('k1').value        # TUNE THESE
        self.k2 = self.get_parameter('k2').value
        self.offset_l = self.get_parameter('offset_l').value

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(PoseArray, '/cones', self.planning_callback, 10)
        self.create_service(Trigger, 'replan', self.replan_callback)
        self.timer = self.create_timer(CONTROL_PERIOD, self.control_loop)

        self.trajectory = None
        self.t_start = None
        self.planned = False
        self.snapshot = []

        self.get_logger().info(
            f'TurtleBot controller initialized: mode={self.path_mode}, '
            f'k=[{self.k1}, {self.k2}], l={self.offset_l}'
        )

    def robot_pose(self):
        """(x, y, theta) in odom, or None if tf is not up yet."""
        try:
            trans = self.tf_buffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time()
            )
        except tf2_ros.TransformException:
            return None
        q = trans.transform.rotation
        return (
            trans.transform.translation.x,
            trans.transform.translation.y,
            euler.quat2euler([q.w, q.x, q.y, q.z])[2],
        )

    def control_loop(self):
        if not self.trajectory:
            return

        pose = self.robot_pose()
        if pose is None:
            return
        x, y, theta = pose

        elapsed = (self.get_clock().now() - self.t_start).nanoseconds * 1e-9
        xd, yd, xd_dot, yd_dot, finished = sample_trajectory(
            self.trajectory, elapsed
        )

        # TODO: Find the controlled point. It sits self.offset_l in front of
        # the robot's center -- that offset is what makes the robot's motion
        # invertible, see the point offset derivation in the lab doc.
        a, b = 0., 0.

        # TODO: Error between where the reference should be and where the
        # controlled point actually is.
        ex, ey = 0., 0.

        if finished and math.hypot(ex, ey) < GOAL_TOL:
            self.trajectory = None
            self.pub.publish(Twist())
            self.get_logger().info(
                'Goal reached. Call the replan service to go again: '
                'ros2 service call /turtlebot_controller/replan '
                'std_srvs/srv/Trigger'
            )
            return

        # TODO: Desired velocity of the controlled point: the trajectory's own
        # velocity (the feedforward terms) plus a proportional correction on
        # the error, with gains self.k1 and self.k2.
        ux, uy = 0., 0.

        # TODO: Map that back to the robot's inputs through the inverse of the
        # decoupling matrix. Watch the 1/l on the angular row.
        control_cmd = Twist()

        control_cmd.linear.x = float(
            np.clip(control_cmd.linear.x, -MAX_LINEAR, MAX_LINEAR)
        )
        control_cmd.angular.z = float(
            np.clip(control_cmd.angular.z, -MAX_ANGULAR, MAX_ANGULAR)
        )
        self.pub.publish(control_cmd)

    def planning_callback(self, msg: PoseArray):
        if self.planned:
            return

        self.snapshot.append([(p.position.x, p.position.y) for p in msg.poses])
        if len(self.snapshot) < SNAPSHOT_FRAMES:
            return

        pose = self.robot_pose()
        if pose is None:
            self.get_logger().warn('TF not ready; retrying')
            return
        x, y, theta = pose

        # Plan from the controlled point, not the robot's center. Otherwise
        # t=0 opens with an offset_l error pulling backwards and the robot
        # crawls off the line.
        start = (x + self.offset_l * math.cos(theta),
                 y + self.offset_l * math.sin(theta),
                 theta)

        obstacles = self._consolidate(self.snapshot)
        self.planned = True
        self.get_logger().info(
            f'Planning around {len(obstacles)} cone(s) from '
            f'{SNAPSHOT_FRAMES} frames, mode={self.path_mode}'
        )
        self.trajectory = plan_trajectory(obstacles, start, mode=self.path_mode)
        self.t_start = self.get_clock().now()
        self.get_logger().info(
            f'Trajectory: {len(self.trajectory)} points, '
            f'{self.trajectory[-1][0]:.1f}s'
        )

    def replan_callback(self, request, response):
        """Plan again from wherever the robot is standing now."""
        self.pub.publish(Twist())
        self.trajectory = None
        self.t_start = None
        self.snapshot = []
        self.planned = False
        self.get_logger().info('Re-arming: collecting cone frames again.')
        response.success = True
        response.message = f'collecting {SNAPSHOT_FRAMES} frames, mode={self.path_mode}'
        return response

    @staticmethod
    def _consolidate(frames):
        """Median cone positions across frames, dropping anything that
        appeared in fewer than half of them."""
        clusters = []
        for frame in frames:
            for point in frame:
                for cluster in clusters:
                    centre = np.mean(cluster, axis=0)
                    if np.linalg.norm(np.array(point) - centre) < CLUSTER_RADIUS:
                        cluster.append(point)
                        break
                else:
                    clusters.append([point])

        needed = max(1, int(len(frames) * MIN_SIGHTING_FRACTION))
        return [
            tuple(np.median(np.array(cluster), axis=0))
            for cluster in clusters if len(cluster) >= needed
        ]


def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
