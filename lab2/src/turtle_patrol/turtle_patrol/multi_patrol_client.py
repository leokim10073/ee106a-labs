#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from turtle_patrol_interface.srv import Patrol

class MultiPatrolClient(Node):
    def __init__(self):
        super().__init__('multi_patrol_client')
        self.cli = self.create_client(Patrol, '/turtle_patrol')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /turtle_patrol service...')

    def send_request(self, name, x, y, theta, vel, omega):
        req = Patrol.Request()
        req.turtle_name = name
        req.x = float(x)
        req.y = float(y)
        req.theta = float(theta)
        req.vel = float(vel)
        req.omega = float(omega)
        return self.cli.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 7:
        print("Usage: ros2 run turtle_patrol multi_patrol_client <turtle_name> <x> <y> <theta> <velocity> <omega>")
        sys.exit(1)

    client = MultiPatrolClient()
    future = client.send_request(
        sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5], sys.argv[6]
    )
    rclpy.spin_until_future_complete(client, future)

    response = future.result()
    if response:
        client.get_logger().info(f'Response: success={response.success}, msg="{response.message}"')
    else:
        client.get_logger().error('Service call failed.')

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()