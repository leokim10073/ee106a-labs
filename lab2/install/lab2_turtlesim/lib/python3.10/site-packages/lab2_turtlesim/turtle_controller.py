#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleController(Node):
    def __init__(self, turtle_name: str):
        super().__init__('turtle_controller')
        self.turtle_name = turtle_name
        self.topic_name = f'/{turtle_name}/cmd_vel'
        self.publisher_ = self.create_publisher(Twist, self.topic_name, 10)
        self.get_logger().info(f'Controller initialized for {turtle_name}. Publishing to {self.topic_name}')

    def run(self):
        print(f"\n--- Controlling {self.turtle_name} ---")
        print("Controls: [w] Forward | [s] Backward | [a] Turn Left | [d] Turn Right | [q] Quit")
        
        while rclpy.ok():
            try:
                cmd = input("Enter command: ").strip().lower()
            except (KeyboardInterrupt, EOFError):
                break

            msg = Twist()
            if cmd == 'w':
                msg.linear.x = 2.0
            elif cmd == 's':
                msg.linear.x = -2.0
            elif cmd == 'a':
                msg.angular.z = 1.5
            elif cmd == 'd':
                msg.angular.z = -1.5
            elif cmd == 'q':
                break
            else:
                print("Invalid key! Use w/a/s/d/q.")
                continue

            self.publisher_.publish(msg)
            self.get_logger().info(f'Published -> linear.x: {msg.linear.x}, angular.z: {msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("Usage: ros2 run lab2_turtlesim turtle_controller <name_of_turtle>")
        sys.exit(1)

    turtle_name = sys.argv[1]
    node = TurtleController(turtle_name)

    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()