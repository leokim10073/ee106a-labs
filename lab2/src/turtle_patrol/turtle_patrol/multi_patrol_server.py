#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute
from turtle_patrol_interface.srv import Patrol

class MultiPatrolServer(Node):
    def __init__(self):
        super().__init__('multi_patrol_server')
        self.srv = self.create_service(Patrol, '/turtle_patrol', self.patrol_callback)
        # turtles 字典結構: {turtle_name: {'pub': pub, 'teleport_cli': cli, 'twist': Twist}}
        self.turtles = {}
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Multi Patrol Server ready on /turtle_patrol')

    def patrol_callback(self, request, response):
        name = request.turtle_name
        self.get_logger().info(f'Received request for {name}: Teleport to ({request.x}, {request.y}, {request.theta}) with vel={request.vel}, omega={request.omega}')

        # 1. 若該烏龜第一次被控制，建立對應的 Publisher 與 Teleport Client
        if name not in self.turtles:
            pub = self.create_publisher(Twist, f'/{name}/cmd_vel', 10)
            teleport_cli = self.create_client(TeleportAbsolute, f'/{name}/teleport_absolute')
            self.turtles[name] = {
                'pub': pub,
                'teleport_cli': teleport_cli,
                'twist': Twist()
            }

        turtle_data = self.turtles[name]

        # 2. 非阻塞呼叫瞬移（直接 call_async，不要 spin 自己）
        teleport_req = TeleportAbsolute.Request()
        teleport_req.x = float(request.x)
        teleport_req.y = float(request.y)
        teleport_req.theta = float(request.theta)
        turtle_data['teleport_cli'].call_async(teleport_req)

        # 3. 更新目標速度（Timer 會在背景持續以 10Hz 發布）
        cmd = Twist()
        cmd.linear.x = float(request.vel)
        cmd.angular.z = float(request.omega)
        turtle_data['twist'] = cmd

        response.success = True
        response.message = f'{name} teleported and patrolling.'
        return response

    def timer_callback(self):
        for name, data in self.turtles.items():
            data['pub'].publish(data['twist'])

def main(args=None):
    rclpy.init(args=args)
    node = MultiPatrolServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()