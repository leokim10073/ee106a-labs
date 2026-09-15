import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

SIDE =    # meters
V =     # m/s 
W =       # rad/s 


def quaternion_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))

class SquareDrive(Node):

    def __init__(self):
        super().__init__('square_drive')


        # odom is watched, never used to steer. Do not touch just observe
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.first_odom = None
        self.last_odom = None

    def spin_for(self, seconds):
        # keep the node spinning. Why do we need to do this?
        end = self.get_clock().now() + Duration(seconds=seconds)
        while rclpy.ok() and self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def drive(self):
        # TODO: Drive around the square. 
        raise NotImplementedError

# -----------------------------------------#
    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.last_odom = (p.x, p.y, quaternion_to_yaw(msg.pose.pose.orientation))
        if self.first_odom is None:
            self.first_odom = self.last_odom

    def report(self):
        if self.first_odom is None or self.last_odom is None:
            return

        x0, y0, yaw0 = self.first_odom
        x1, y1, yaw1 = self.last_odom

        self.get_logger().info(
            f'odom start ({x0:.3f}, {y0:.3f}, {math.degrees(yaw0):6.1f} deg)')
        self.get_logger().info(
            f'odom end   ({x1:.3f}, {y1:.3f}, {math.degrees(yaw1):6.1f} deg)')

        # we commanded a closed loop. 
        # odom says we traveled a certain amount. 
        # Doth our eyes believe odom (smth smth insert 1984)
        self.get_logger().info(
            f'odom thinks it closed the loop to within '
            f'{math.hypot(x1 - x0, y1 - y0):.3f} m')


def main():
    rclpy.init()
    node = SquareDrive()

    node.spin_for(1.0)   # let the first odom message land before we move

    try:
        # TODO
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())

    node.report()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
