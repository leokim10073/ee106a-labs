import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    my_pkg = get_package_share_directory('turtlebot_controller')
    cartographer_pkg = get_package_share_directory('turtlebot3_cartographer')

    # TODO: declare the 'rviz' launch argument for the RViz node below 
    # DeclareLaunchArgument takes a name, a default_value
    # (a string, even for a boolean) and a description.
    rviz_arg = None

    cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-configuration_directory', os.path.join(cartographer_pkg, 'config'),
            '-configuration_basename', 'turtlebot3_lds_2d.lua',
        ],
    )

    occupancy_grid = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
    )

    # TODO: Think about how we used ros2 launch...
    # TODO: For the conditions, look around this file for a hint. 
    # For the argument, normally we use config. For this lab, our configs are somewhere in the lab.
    rviz = Node(
        package='TODO',
        executable='TODO',
        name='TODO',
        arguments=['-d', os.path.join(my_pkg, 'TODO', 'TODO')],
        output='screen',
        condition=TODO:
    )

    return LaunchDescription([
        rviz_arg,
        cartographer,
        occupancy_grid,
        rviz,
    ])
