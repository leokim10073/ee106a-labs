# ROS Libraries
from std_srvs.srv import Trigger
import sys
import copy
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation as R
import numpy as np

from planning.ik import IKPlanner

TAPE_SETTLE_SEC = 1.0  # wait this long after getting back to the start pose before trusting /tape_pose

# How far below wrist_3_link the fingertips are. Measure this on your gripper!
# The IK targets are for wrist_3_link, so add this to where you want the fingertips to be.
GRIPPER_LENGTH = 0.05
PRE_GRASP_CLEARANCE = 0.05  # how far above the cube the fingertips start
TABLE_CLEARANCE = 0.01  # never bring the fingertips (or the held cube) closer than this to the table

class UR7e_CubeGrasp(Node):
    def __init__(self):
        super().__init__('cube_grasp')

        self.cube_pub = self.create_subscription(PointStamped, '/cube_pose', self.cube_callback, 1)
        self.cube_height_sub = self.create_subscription(Float32, '/cube_height', self.cube_height_callback, 1)
        self.tape_sub = self.create_subscription(PointStamped, '/tape_pose', self.tape_callback, 1)
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 1)

        self.exec_ac = ActionClient(
            self, FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )

        self.gripper_cli = self.create_client(Trigger, '/toggle_gripper')
        self.switch_cli = self.create_client(SwitchController, '/controller_manager/switch_controller')

        self.cube_pose = None
        self.cube_height = None
        self.grasp_height = None  # how far above the bottom of the cube we grabbed it
        self.held_cube_height = None
        self.current_plan = None
        self.joint_state = None
        self.start_joints = None  # where you free drove the robot to, recorded on startup

        self.waiting_for_tape = False
        self.tape_wait_until = None
        self.create_timer(3.0, self._check_tape_wait)

        self.ik_planner = IKPlanner()

        self.job_queue = [] # Entries should be of type either JointState, String('toggle_grip') or String('find_tape')

        # after free driving, the trajectory controller might not be the active one anymore
        self._switch_to_trajectory_controller()

    def joint_state_callback(self, msg: JointState):
        self.joint_state = msg
        if self.start_joints is None:
            self.start_joints = copy.deepcopy(msg)
            self.get_logger().info('Recorded the start pose, we will come back here to look for the tape')

    def cube_height_callback(self, msg: Float32):
        self.cube_height = msg.data

    def cube_callback(self, cube_pose):
        if self.cube_pose is not None:
            return

        if self.joint_state is None or self.cube_height is None:
            self.get_logger().info("No joint state or cube height yet, cannot proceed")
            return

        self.cube_pose = cube_pose
        # /cube_pose is the middle of the cube, and /cube_height is how tall it is
        x, y, z = cube_pose.point.x, cube_pose.point.y, cube_pose.point.z
        h = self.cube_height
        self.get_logger().info(f'Cube at ({x:.3f}, {y:.3f}, {z:.3f}), {h * 100:.1f} cm tall')

        # -----------------------------------------------------------
        # TODO: In the following section you will add joint angles to the job queue.
        # Entries of the job queue should be of type either JointState, String('toggle_grip') or String('find_tape')
        # Think about you will leverage the IK planner to get joint configurations for the cube grasping task.
        # To understand how the queue works, refer to the execute_jobs() function below.
        # -----------------------------------------------------------

        cube_top = z + h / 2
        table_z = z - h / 2
        self.held_cube_height = h

        # 1) Move to Pre-Grasp Position (gripper above the cube)
        '''
        The fingertips should start PRE_GRASP_CLEARANCE above the top of the cube.
        Remember the IK target is wrist_3_link, which is GRIPPER_LENGTH above the fingertips.
        '''
        pre_grasp_tip_z = cube_top + PRE_GRASP_CLEARANCE
        pre_grasp_js = self.ik_planner.compute_ik(
            self.joint_state, x, y, pre_grasp_tip_z + GRIPPER_LENGTH)
        if pre_grasp_js is None:
            self.get_logger().error('No IK for pre-grasp, will try again with the next cube pose')
            self.cube_pose = None
            return
        self.job_queue.append(pre_grasp_js)

        # 2) Move to Grasp Position (lower the gripper to the cube)
        '''
        Put the fingertips at the middle of the cube (half its height), but never closer
        than TABLE_CLEARANCE to the table. Save how far above the bottom of the cube the
        fingertips are in self.grasp_height, you'll need it to put the cube down.
        '''
        grasp_tip_z = max(z, table_z + TABLE_CLEARANCE)
        self.grasp_height = grasp_tip_z - table_z
        grasp_js = self.ik_planner.compute_ik(
            pre_grasp_js, x, y, grasp_tip_z + GRIPPER_LENGTH)
        if grasp_js is None:
            self.get_logger().error('No IK for grasp, will try again with the next cube pose')
            self.job_queue.clear()
            self.cube_pose = None
            return
        self.job_queue.append(grasp_js)

        # 3) Close the gripper. See job_queue entries defined in init above for how to add this action.
        self.job_queue.append('toggle_grip')

        # 4) Move back to Pre-Grasp Position
        self.job_queue.append(pre_grasp_js)

        # 5) Go back to the start pose (self.start_joints) so the camera can see the table again
        self.job_queue.append(self.start_joints)

        # 6) Look for the tape. plan_place() gets called once we have a fresh /tape_pose
        self.job_queue.append('find_tape')

        self.execute_jobs()

    def tape_callback(self, tape_pose):
        if not self.waiting_for_tape:
            return
        # ignore anything from before the arm settled at the start pose
        if Time.from_msg(tape_pose.header.stamp) < self.tape_wait_until:
            return
        self.waiting_for_tape = False
        self.plan_place(tape_pose)

    def plan_place(self, tape_pose):
        x, y = tape_pose.point.x, tape_pose.point.y
        self.get_logger().info(f'Tape at ({x:.3f}, {y:.3f}, {tape_pose.point.z:.3f})')

        # -----------------------------------------------------------
        # TODO: Add the jobs to put the cube down on the tape and come back.
        # The tape is flat on the table, so tape_pose.point.z is the height of the table
        # there. The fingertips are self.grasp_height above the bottom of the cube, so to
        # set the cube down, put them that far above the tape, plus TABLE_CLEARANCE so the
        # cube doesn't get pushed into the table.
        # -----------------------------------------------------------

        table_z = tape_pose.point.z
        h = self.held_cube_height

        # 1) Move above the tape (so the bottom of the cube clears the table)
        above_tip_z = table_z + h + PRE_GRASP_CLEARANCE
        above_js = self.ik_planner.compute_ik(
            self.joint_state, x, y, above_tip_z + GRIPPER_LENGTH)
        if above_js is None:
            self.get_logger().error('No IK above the tape')
            return
        self.job_queue.append(above_js)

        # 2) Lower the cube down onto the tape
        place_tip_z = table_z + self.grasp_height + TABLE_CLEARANCE
        place_js = self.ik_planner.compute_ik(
            above_js, x, y, place_tip_z + GRIPPER_LENGTH)
        if place_js is None:
            self.get_logger().error('No IK to lower onto the tape')
            self.job_queue.clear()
            return
        self.job_queue.append(place_js)

        # 3) Open the gripper
        self.job_queue.append('toggle_grip')

        # 4) Move back up
        self.job_queue.append(above_js)

        # 5) Go back to the start pose
        self.job_queue.append(self.start_joints)

        self.execute_jobs()

    def execute_jobs(self):
        if not self.job_queue:
            self.get_logger().info("All jobs completed.")
            rclpy.shutdown()
            return

        self.get_logger().info(f"Executing job queue, {len(self.job_queue)} jobs remaining.")
        next_job = self.job_queue.pop(0)

        if isinstance(next_job, JointState):

            traj = self.ik_planner.plan_to_joints(next_job)
            if traj is None:
                self.get_logger().error("Failed to plan to position")
                return

            self.get_logger().info("Planned to position")

            self._execute_joint_trajectory(traj.joint_trajectory)
        elif next_job == 'toggle_grip':
            self.get_logger().info("Toggling gripper")
            self._toggle_gripper()
        elif next_job == 'find_tape':
            self.get_logger().info("Looking for the tape...")
            self.tape_wait_until = self.get_clock().now() + Duration(seconds=TAPE_SETTLE_SEC)
            self.waiting_for_tape = True
            # tape_callback takes it from here
        else:
            self.get_logger().error("Unknown job type.")
            self.execute_jobs()  # Proceed to next job

    def _check_tape_wait(self):
        if self.waiting_for_tape and self.get_clock().now() - self.tape_wait_until > Duration(seconds=4.0):
            self.get_logger().warn('Still no tape. Check /tape_points in RViz and your HSV thresholds')

    def _switch_to_trajectory_controller(self):
        if not self.switch_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('No controller_manager, assuming the trajectory controller is already active')
            return
        req = SwitchController.Request()
        req.activate_controllers = ['scaled_joint_trajectory_controller']
        req.deactivate_controllers = ['freedrive_mode_controller', 'forward_position_controller',
                                      'forward_velocity_controller']
        req.strictness = SwitchController.Request.BEST_EFFORT
        req.activate_asap = True
        future = self.switch_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is None or not future.result().ok:
            self.get_logger().warn('Could not switch to scaled_joint_trajectory_controller')

    def _toggle_gripper(self):
        if not self.gripper_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Gripper service not available')
            rclpy.shutdown()
            return

        req = Trigger.Request()
        future = self.gripper_cli.call_async(req)
        # wait for 2 seconds
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        self.get_logger().info('Gripper toggled.')
        self.execute_jobs()  # Proceed to next job


    def _execute_joint_trajectory(self, joint_traj):
        self.get_logger().info('Waiting for controller action server...')
        self.exec_ac.wait_for_server()

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = joint_traj

        self.get_logger().info('Sending trajectory to controller...')
        send_future = self.exec_ac.send_goal_async(goal)
        print(send_future)
        send_future.add_done_callback(self._on_goal_sent)

    def _on_goal_sent(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('bonk')
            rclpy.shutdown()
            return

        self.get_logger().info('Executing...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_exec_done)

    def _on_exec_done(self, future):
        try:
            result = future.result().result
            self.get_logger().info('Execution complete.')
            self.execute_jobs()  # Proceed to next job
        except Exception as e:
            self.get_logger().error(f'Execution failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = UR7e_CubeGrasp()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()
