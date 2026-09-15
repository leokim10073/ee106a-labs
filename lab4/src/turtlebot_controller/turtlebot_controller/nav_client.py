import math

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_to_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class NavClient(Node):

    def __init__(self):
        super().__init__('nav_client')

        self.declare_parameter('marker_id', 7)
        self.declare_parameter('standoff', 0.5)
        self.declare_parameter('goal_frame', 'map')
        self.declare_parameter('max_tf_age', 5.0)
        self.declare_parameter('set_initial_pose', True)
        self.declare_parameter('x', float('nan'))
        self.declare_parameter('y', float('nan'))
        self.declare_parameter('yaw', 0.0)

        self.marker_id = self.get_parameter('marker_id').value
        self.standoff = self.get_parameter('standoff').value
        self.goal_frame = self.get_parameter('goal_frame').value
        self.max_tf_age = self.get_parameter('max_tf_age').value

        self.done = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10)

        # TODO 1: build the action client. which action type, and what is the
        # server called? (ros2 action list)
        self.client = None

    def spin_for(self, seconds):
        end = self.get_clock().now() + Duration(seconds=seconds)
        while rclpy.ok() and self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.goal_frame
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        _, _, qz, qw = yaw_to_quaternion(0.0)
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.068

        for _ in range(3):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.initial_pose_pub.publish(msg)
            self.spin_for(0.5)

        self.get_logger().info(f'seeded AMCL at the origin of {self.goal_frame}')

    def robot_xy(self):
        tf = self.tf_buffer.lookup_transform(self.goal_frame, 'base_footprint', Time())
        return tf.transform.translation.x, tf.transform.translation.y

    def tag_transform(self):
        frame = f'ar_marker_{self.marker_id}'
        # TODO 2: look the tag up in self.goal_frame. one call -- tf composes the
        # whole chain from the map down to the marker for you.
        tf = None

        # tf2 keeps handing back the last transform it saw. refuse a stale one.
        age = (self.get_clock().now() - Time.from_msg(tf.header.stamp)).nanoseconds / 1e9
        if age > self.max_tf_age:
            raise RuntimeError(
                f'{frame} was last seen {age:.1f} s ago -- point the camera at the tag')

        return tf

    def tag_goal(self):
        tf = self.tag_transform()
        tag_x = tf.transform.translation.x
        tag_y = tf.transform.translation.y
        robot_x, robot_y = self.robot_xy()

        # TODO 3: return (goal_x, goal_y, goal_yaw) self.standoff metres short of the
        # tag, facing it. 
        raise NotImplementedError

    def make_goal(self):
        x = self.get_parameter('x').value
        y = self.get_parameter('y').value

        if math.isnan(x) or math.isnan(y):
            x, y, yaw = self.tag_goal()
        else:
            yaw = self.get_parameter('yaw').value

        # TODO 4: build and return the PoseStamped. it needs a frame_id, a stamp, a
        # position and an orientation. yaw_to_quat is given
        raise NotImplementedError

    def send(self):
        goal = self.make_goal()
        self.get_logger().info(
            f'goal ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f}) '
            f'in frame {goal.header.frame_id}')

        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('no navigate_to_pose action server')
            return False

        # TODO 5: build the goal message, send it with on_feedback as the feedback
        # callback, and hand the future it returns to on_goal_response. read
        # on_goal_response below first -- it does the same trick a second time.
        raise NotImplementedError

    def on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('goal rejected')
            self.done = True
            return

        self.get_logger().info('goal accepted')
        handle.get_result_async().add_done_callback(self.on_result)

    def on_feedback(self, msg):
        # TODO 6: print how far Nav2 thinks it still has to go. 
        raise NotImplementedError

    def on_result(self, future):
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('arrived')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn('aborted')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('canceled')

        self.done = True


def main():
    rclpy.init()
    node = NavClient()

    if node.get_parameter('set_initial_pose').value:
        node.publish_initial_pose()

    # nothing has spun yet, so the tf buffer is empty. fill it before looking anything up.
    node.spin_for(3.0)

    try:
        if node.send():
            while rclpy.ok() and not node.done:
                rclpy.spin_once(node, timeout_sec=0.1)
    except (TransformException, RuntimeError) as exc:
        node.get_logger().error(f'could not build a goal: {exc}')
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
