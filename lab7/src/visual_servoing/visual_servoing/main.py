#!/usr/bin/env python3
"""
Main script to run visual servoing
Author: Daniel Municio, Spring 2026
"""

import rclpy
from rclpy.node import Node
import numpy as np
from tf2_ros import TransformListener, Buffer
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK
from nav_msgs.msg import Path
from builtin_interfaces.msg import Duration
import visual_servoing.jacobian_func_skeleton as jfs
import visual_servoing.kin_func_skeleton as kfs
from rclpy.signals import SignalHandlerOptions
import argparse
import sys
import select
import time
import subprocess
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

from .trajectories import LinearTrajectory, CircularTrajectory
from .controller import UR7eTrajectoryController, PIDJointVelocityController

UR7E_TWISTS = np.array(
    [
        [0.0, -0.1625,-0.1625, -0.1625, -0.1333, -0.0628],
        [0.0, 0.0,    0.0,    0.0,   0.8172, 0.0],
        [0.0, 0.0,    0.4250, 0.8172, 0.0,    0.8172],
        [0.0, 0.0,    0.0,    0.0,    0.0,    0.0],
        [0.0, 1.0,    1.0,   1.0,    0.0,   1.0],
        [1.0, 0.0,    0.0,    0.0,   -1.0,    0.0],
    ]
)

class VisualServo(Node):
    def __init__(self, args):
        super().__init__('visual_servo_node')

        self.args = args
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.twists = UR7E_TWISTS
        self.current_joint_state = None

        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 1)
        self._velocity_pub = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)

        self.controller_type = args.controller
        self.ar_marker = args.ar_marker


        self.nominal_dt = 0.1 #we spin the node every 0.1 sec

        #TODO tune these as needed

        self.Kp = 0
        self.Kd = 0 * self.nominal_dt
        self.Ki = 0 / self.nominal_dt

        self.alpha = 0.8

        self.viz_timer = None
        self.ar_tag_detected = False
        self.ar_tag_position = None

    def joint_state_callback(self, msg):
        """Store current joint state"""
        self.current_joint_state = msg

    def quaternion_to_rotation_matrix(self, x, y, z, w):
        """Convert quaternion (x, y, z, w) to a 3x3 rotation matrix."""
        norm = np.sqrt(x * x + y * y + z * z + w * w)
        if norm > 0:
            x, y, z, w = x / norm, y / norm, z / norm, w / norm
        return np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ]
        )
    
    def lookup_ar_tag(self, marker_id):
        """Looks for ar tag during visual servoing and returns the transformation in the spatial frame"""
        target_frame = 'base_link'
        source_frame = f'ar_marker_{marker_id}'

        try:
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            
            transform_time = rclpy.time.Time.from_msg(
                transform.header.stamp
            )

            now = self.get_clock().now()
            age = (now - transform_time).nanoseconds / 1e9

            if age > 1.0:  # 200 ms
                self.get_logger().info("Transform is stale")
                self.send_zero()
                return False
            
            return True


        except Exception as e:
            self.send_zero()
            self.get_logger().info(f"Waiting for AR tag {marker_id}")
            return False


    def find_g(self, source_frame):
        """Finds the desired frame in the spatial frame"""
        target_frame = 'base_link'

        transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())

        pos = transform.transform.translation
        rot = transform.transform.rotation

        # Convert quaternion to rotation matrix
        rotation_matrix = self.quaternion_to_rotation_matrix(
            rot.x, rot.y, rot.z, rot.w
        )

        G = np.eye(4)
        G[:3,:3]= rotation_matrix
        G[:3,3]= np.array([pos.x,pos.y,pos.z])
        return G

    def find_error(self, gd, g):
        """Converts the configuration error from SE(3) to a 6x1 twist and returns it"""
        #TODO: Hint: look at the forward kinematics helper functions
        return ...
    
    def send_velocity(self, joint_velocities):
        """Caps the joint speeds at 1 rad/s and publishes the joint speeds to /forward_velocity_controller/commands """
        max_speed = 1
        if any(abs(i) > max_speed for i in joint_velocities):
            joint_velocities = joint_velocities * (max_speed/max(abs(joint_velocities)))

        print(joint_velocities)

        vel_msg = Float64MultiArray()
        vel_msg.data = joint_velocities.tolist()
        self._velocity_pub.publish(vel_msg)

    def send_zero(self):
        """Sets all joint velocities to 0"""
        vel_msg = Float64MultiArray()
        vel_msg.data = [0.0]*6
        self._velocity_pub.publish(vel_msg)


    def get_joint_state(self):
        """Sorts by name and extracts joint positions and velocities from /joint_states"""
        current_joint_dict = {}
        for i, name in enumerate(self.current_joint_state.name):
            if i < 6:
                current_joint_dict[name] = (self.current_joint_state.position[i],
                                            self.current_joint_state.velocity[i])

        current_position = np.array([
            current_joint_dict['shoulder_pan_joint'][0],
            current_joint_dict['shoulder_lift_joint'][0],
            current_joint_dict['elbow_joint'][0],
            current_joint_dict['wrist_1_joint'][0],
            current_joint_dict['wrist_2_joint'][0],
            current_joint_dict['wrist_3_joint'][0]
        ])
        current_velocity = np.array([
            current_joint_dict['shoulder_pan_joint'][1],
            current_joint_dict['shoulder_lift_joint'][1],
            current_joint_dict['elbow_joint'][1],
            current_joint_dict['wrist_1_joint'][1],
            current_joint_dict['wrist_2_joint'][1],
            current_joint_dict['wrist_3_joint'][1]
        ])

        return current_position, current_velocity

    def get_spatial_jacobian(self):
        """Calculates the spatial jacobian based on the current joint state"""
        current_position, _ = self.get_joint_state()
        #TODO hint: look at the jacobian helper functions
        return ...
    
    #Circle Stuff

    def wait_for_state(self):
        """Block until joint states and the wrist TF are both available."""
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_joint_state is None:
                continue
            try:
                self.find_g('wrist_3_link')
                return
            except Exception:
                pass

    def pose_matrix_from_target(self, traj, t):
        """Trajectory [x,y,z,qx,qy,qz,qw] -> 4x4 desired pose g_d(t)."""
        pose = traj.target_pose(t)
        G = np.eye(4)
        G[:3, :3] = self.quaternion_to_rotation_matrix(*pose[3:7])
        G[:3, 3] = pose[:3]
        return G

    def make_pose(self):
        """Creates the goal pose for a point"""
        pose = self.find_g('center_pose')
        return pose

    def make_line(self):
        # LinearTrajectory starts at point p0. Place the target at the center_pose position
        #TODO

        return LinearTrajectory(p0, target, self.args.total_time)

    def make_circle(self):
        # CircularTrajectory starts at center + [r, 0, 0], where the center is the center_pose
        #TODO

        return CircularTrajectory(center, r, self.args.total_time)

    def control_step(self, goal_pose, Vff, eta, nu, dt):
        """Runs the workspace controller and sends the velocities"""

        Vs, eta, nu = self.workspace_controller(goal_pose, Vff, eta, nu, dt)
        joint_speeds = np.linalg.pinv(self.get_spatial_jacobian()) @ Vs
        self.send_velocity(joint_speeds)
        return eta, nu

    def run_point(self, goal_pose):
        """Run workspace controller to a pose"""
        switch_controllers(self.controller_type)
        min_dt = 0.01  # cap the controller at ~100 Hz

        # ---- Phase 1: converge to the point (Vff = 0) ----
        goal = goal_pose
        eta = self.find_error(goal, self.find_g('wrist_3_link'))  # avoids derivative kick
        nu = np.zeros(6)
        t_begin = t_prev = self.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=self.nominal_dt)
            now = self.get_clock().now()
            dt = (now - t_prev).nanoseconds * 1e-9
            if dt < min_dt:
                continue
            t_prev = now

            eta, nu = self.control_step(goal, np.zeros(6), eta, nu, dt)
            if np.linalg.norm(eta) < 0.005:      # ~2 cm / ~0.02 rad
                break
            if (now - t_begin).nanoseconds * 1e-9 > 20.0:
                self.get_logger().error("Could not reach trajectory start")
                self.send_zero()
                return

        self.send_zero()
        self.get_logger().info("Point Reached")

    def run_trajectory(self, traj):
        """Run workspace controller along a trajectory"""
        switch_controllers(self.controller_type)
        min_dt = 0.01  # cap the controller at ~100 Hz

        # ---- Phase 1: converge to the trajectory's t=0 pose (Vff = 0) ----
        goal = self.pose_matrix_from_target(traj, 0.0)
        eta = self.find_error(goal, self.find_g('wrist_3_link'))  # avoids derivative kick
        nu = np.zeros(6)
        t_begin = t_prev = self.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=self.nominal_dt)
            now = self.get_clock().now()
            dt = (now - t_prev).nanoseconds * 1e-9
            if dt < min_dt:
                continue
            t_prev = now

            eta, nu = self.control_step(goal, np.zeros(6), eta, nu, dt)
            if np.linalg.norm(eta) < 0.02:      # ~2 cm / ~0.02 rad
                break
            if (now - t_begin).nanoseconds * 1e-9 > 20.0:
                self.get_logger().error("Could not reach trajectory start")
                self.send_zero()
                return

        # ---- Phase 2: track the trajectory ----
        nu = np.zeros(6)  # drop integrator windup from the approach
        t_start = t_prev = self.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=self.nominal_dt)
            now = self.get_clock().now()
            dt = (now - t_prev).nanoseconds * 1e-9
            if dt < min_dt:
                continue
            t_prev = now

            t = (now - t_start).nanoseconds * 1e-9
            if t > traj.total_time:
                break

            goal = self.pose_matrix_from_target(traj, t)
            #TODO Change Vff to 0 for 3.4.2, but then change it back
            Vff = traj.target_velocity(t)       # [vx, vy, vz, 0, 0, 0]
            eta, nu = self.control_step(goal, Vff, eta, nu, dt)

        self.send_zero()
        self.get_logger().info("Trajectory finished")



    def workspace_controller(self, goal_pose, Vff, eta_prev, nu, dt):
        """Calculates the spatial velocity based on the configuration error and returns the spatial velocity, configuration error, and integral config. error"""

        current_pose = self.find_g('wrist_3_link')

        #TODO: Implement the workspace controller


        return Vs, eta, nu

        

    def run_visual_servoing(self):
        """Main run loop for visual servoing"""
        
        eta = np.zeros(6)
        nu = np.zeros(6)
        time = 0
        switch_controllers(self.controller_type)

        while rclpy.ok():

            if self.lookup_ar_tag(self.ar_marker):
                #TODO: Use the workspace controller you already made for visual servoing
            
                goal_pose = ...
                Vff = ...

                time_prev = ...
                time = ...
                dt = ...

                Vs, eta, nu = ...

                if self.current_joint_state is not None:
                    joint_speeds = ...
                else:
                    self.get_logger().info("Joint states not found")
                
                self.send_velocity(joint_speeds)
            
            rclpy.spin_once(self, timeout_sec=self.nominal_dt)


def switch_controllers(controller_type):
    """
    Switch ROS2 controllers based on the selected controller type.

    Parameters
    ----------
    controller_type : str
        Either 'default' or 'pid'
    """
    if controller_type == 'default':
        cmd = [
            'ros2', 'control', 'switch_controllers',
            '--deactivate', 'forward_velocity_controller',
            '--activate', 'scaled_joint_trajectory_controller'
        ]
    else:  # pid
        cmd = [
            'ros2', 'control', 'switch_controllers',
            '--deactivate', 'scaled_joint_trajectory_controller',
            '--activate', 'forward_velocity_controller'
        ]

    logger.info(f"Switching controllers for '{controller_type}' mode...")
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            logger.info("Switching controllers successful")
        else:
            logger.warning(f"Controller switch warning: {result.stderr}")
    except subprocess.TimeoutExpired:
        logger.error("Controller switch timed out")
    except Exception as e:
        logger.error(f"Controller switch failed: {e}")


def main(args=None):
    parser = argparse.ArgumentParser(description='Bonk')
    parser.add_argument('--task', '-t', type=str, default='',
                       choices=['pose', 'line', 'circle'],
                       help='Type of trajectory: line or circle')
    parser.add_argument('--ar_marker', type=int, default=0,
                       help='AR marker ID to track')
    parser.add_argument('--total_time', type=float, default=10.0,
                       help='Total time for trajectory execution (seconds)')
    parser.add_argument('--circle_radius', type=float, default=0.1,
                       help='Radius for circular trajectory (meters)')
    parser.add_argument('--controller', '-c', type=str, default='default',
                       choices=['default', 'pid'],
                       help='Controller type: trajectory (default), pid')

    parsed_args = parser.parse_args()

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = VisualServo(parsed_args)

    
    try:
        if parsed_args.task == 'circle':
            node.wait_for_state()
            node.run_trajectory(node.make_circle())
        elif parsed_args.task == 'line':
            node.wait_for_state()
            node.run_trajectory(node.make_line())
        elif parsed_args.task == 'pose':
            node.wait_for_state()
            node.run_point(node.make_pose())
        else:
            node.run_visual_servoing()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    finally:
        logger.info("Stopping robot and switching back to default controller...")
        try:
            for _ in range(5):          # repeat in case a message gets dropped
                node.send_zero()
                time.sleep(0.02)
        except Exception as e:
            logger.error(f"Failed to send zero velocity: {e}")
        switch_controllers('default')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
