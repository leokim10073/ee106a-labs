
# here are a number of imports you may find helpful
import time
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import Pose, TransformStamped
from moveit_msgs.srv import GetCartesianPath
from tf2_ros import Buffer, TransformException, TransformListener

from straight_line_interface.action import MoveStraight

class StraightLineServer(Node):
    def __init__(self):
        super().__init__('straight_line_server')
        self._cb_group = ReentrantCallbackGroup()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._switch_cli = self.create_client(
            SwitchController,
            '/controller_manager/switch_controller',
            callback_group=self._cb_group,
        )
        self._cart_cli = self.create_client(
            GetCartesianPath,
            '/compute_cartesian_path',
            callback_group=self._cb_group,
        )
        self._exec_ac = ActionClient(
            self,
            FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory',
            callback_group=self._cb_group,
        )

        self._action_server = ActionServer(
            self,
            MoveStraight
            'move_straight',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._cb_group,
        )
        self.get_logger().info('StraightLineServer ready on /move_straight')

    def goal_callback(self, goal_request):
        self.get_logger().info('Received MoveStraight goal request')
        return ...

    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info('Received cancel')
        return ...

    def _lookup_tool0_pose(self) -> Pose:
        """
        get the transform between `base_link` and the tool frame (`tool0`).

        this can fail!
        """
        # hint: self._tf_buffer.lookup_transform
        return ...

    def _ensure_controller(self) -> bool:
        """
        use self._switch_cli to switch the controller to `scaled_joint_trajectory_controller`.
        Deactivate any of the following controllers:
          - `freedrive_mode_controller`
          - `forward_position_controller`
          - `forward_velocity_controller`
        see https://docs.ros.org/en/humble/p/controller_manager_msgs/srv/SwitchController.html

        return a bool indicating success. it should return false if any of these happen:
          - _switch_cli wait_for_service takes too long
          - _switch_cli call takes too long
          - _switch_cli call fails (result.ok is false)
        """
        if not self._switch_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('switch_controller service unavailable')
            return False

        req = SwitchController.Request()
        # TODO: fill in `req`
        
        future = self._switch_cli.call_async(req)
        if not self._wait_future(future, timeout_sec=10.0):
            self.get_logger().error('Controller switch timed out')
            return False
        try:
            res = future.result()
        except Exception as exc:
            self.get_logger().error(f'Controller switch failed: {exc}')
            return False

        if not res.ok:
            self.get_logger().error('Controller switch returned not ok')
            return False
        return True

        
    def _plan_cartesian(self, target_pose: Pose, max_step: float):
        """get the path.
        call _cart_cli (using the GetCartesianPath service type)
        if this fails, it will log an error to the logger.

        return type is (trajectory, fraction_planned, error). the
        `error` value is just the error_code that moveit's planner
        returns, if we get there.  If something fails unrecoverably
        before then, we will just return (None, 0.0, -1).

        """
        if not self._cart_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                '/compute_cartesian_path unavailable (is MoveIt running?)'
            )
            return None, 0.0, -1

        req = GetCartesianPath.Request()
        req.header.frame_id = 'base_link'
        req.header.stamp = self.get_clock().now().to_msg()
        req.start_state.is_diff = True
        
        req.group_name = 'ur_manipulator'

        # TODO: fill these out!
        # check the docs: https://docs.ros.org/en/humble/p/moveit_msgs/srv/GetCartesianPath.html
        req.link_name = ...
        req.waypoints = ...
        req.max_step = ...
        
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        
        fut = self._cart_cli.call_async(req)
        
        if not self._wait_future(future, timeout_sec=30.0):
            self.get_logger().error('Cartesian planning timed out')
            return None, 0.0, -1

        try:
            res = fut.result()
        except Exception as exc:
            self.get_logger().error(f'Cartesian planning call failed: {exc}')
            return None, 0.0, -1

        planned_fraction = float(res.fraction)
        err = int(res.error_code.val)
        jt = res.solution.joint_trajectory
        if err != 1 or not jt.points:
            self.get_logger().error(
                f'Cartesian planning failed (error_code={err}, '
                f'planned_fraction={planned_fraction})'
            )
            return None, planned_fraction, err

        self.get_logger().info(
            f'Cartesian path planned: planned_fraction={planned_fraction:.3f}, '
            f'waypoints={len(jt.points)}'
        )
        return jt, planned_fraction, err
        
    def _execute_trajectory(self, traj, goal_handle: ServerGoalHandle):
        """
        use the follow_joint_trajectory action server to run the movement!
        returns: (succeeded: bool, message: string)
        """
        if not self._exec_ac.wait_for_server(timeout_sec=5.0):
            return False, 'FollowJointTrajectory action server unavailable'

        # TODO: construct the correct goal message for the FollowJointTrajectory action.
        goal = ...
        
        send_future = self._exec_ac.send_goal_async(goal)
        if not self._wait_future(send_future, timeout_sec=10.0):
            return False, 'Timed out waiting for trajectory goal acceptance'

        exec_handle = send_future.result()
        if exec_handle is None or not exec_handle.accepted:
            return False, 'Trajectory goal rejected by controller'

        result_future = exec_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            # TODO: monitor `goal_handle.is_cancel_requested`.
            # if a cancellation was requested, use exec_handle to cancel the goal.
            ...
        
        try:
            result_future.result()
        except Exception as exc:
            return False, f'Trajectory execution failed: {exc}'

        return True, 'ok'

    def _wait_future(self, future, timeout_sec: float) -> bool:
        deadline = time.time() + timeout_sec
        while rclpy.ok() and not future.done():
            if time.time() > deadline:
                return False
            time.sleep(0.01)
        return future.done()

    def execute_callback(self, goal_handle: ServerGoalHandle):
        goal = goal_handle.request
        result = MoveStraight.Result() # TODO: construct empty Result message for this action type
        feedback = MoveStraight.Feedback() # TODO: construct empty Feedback message for this action type
        result.planned_fraction = 0.0
        result.dist_to_go = 0.0

        ########## STEP 1 ##########
        # switch the controller to `scaled_joint_trajectory_controller`
        
        feedback.state = 'switching_controller'
        feedback.planned_fraction = 0.0
        goal_handle.publish_feedback(feedback)

        if not self._ensure_controller():
            result.success = False
            result.message = 'Failed to activate scaled_joint_trajectory_controller'
            goal_handle.abort()
            return result


        ########## STEP 2 ##########
        # get the end effector pose
        
        feedback.state = 'looking_up_ee_pose'
        goal_handle.publish_feedback(feedback)

        try:
            start_pose = self._lookup_tool0_pose()
        except TransformException as exc:
            # TODO: handle this issue! what should we do if this
            # lookup fails?  hint: look at how we dealt with a failure
            # of _ensure_controller() above.
            ...

        target_pose = Pose()
        target_pose.position = goal.target.position
        target_pose.orientation = start_pose.orientation
        # TODO: update dist_to_go based on the above information!
        result.dist_to_go = ...


        ########## STEP 3 ##########
        # compute a jointspace plan to get to the desired pose!

        # we should check for cancellation before planning, because
        # planning is an expensive thing to do. if cancellation has
        # been requested, we don't have to do all that work!
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.success = False
            result.message = 'Canceled before planning'
            return result
        
        feedback.state = 'planning'
        goal_handle.publish_feedback(feedback)

        self.get_logger().info(
            'Planning straight line '
            f'from ({start_pose.position.x:.3f}, {start_pose.position.y:.3f}, '
            f'{start_pose.position.z:.3f}) to '
            f'({target_pose.position.x:.3f}, {target_pose.position.y:.3f}, '
            f'{target_pose.position.z:.3f})'
        )
        
        traj, planned_fraction, err = self._plan_cartesian(target_pose, goal.max_step)
        result.planned_fraction = planned_fraction
        feedback.planned_fraction = planned_fraction

        if traj is None:
            ... # TODO: handle total planning failure

        if planned_fraction < 0.999:
            ... # TODO: handle partial plannnig failure (it got stuck halfway ig)

        ########## STEP 4 ##########
        # execute the plan!
        feedback.state = 'executing'
        goal_handle.publish_feedback(feedback)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.success = False
            result.message = 'Canceled before execution'
            return result

        ok, exec_msg = self._execute_trajectory(traj, goal_handle)

        # get end pose so we can return how far we got
        current_pose = self._lookup_tool0_pose()
        result.dist_to_go = ... # TODO: get distance to target
  
        if not ok:
            result.success = False
            result.message = exec_msg
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return result

        feedback.state = 'done'
        goal_handle.publish_feedback(feedback)

        result.success = True
        result.message = 'Moved along 100% of requested path'
        goal_handle.succeed()
        return result



def main(args=None):
    rclpy.init(args=args)
    node = StraightLineServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
