import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    aruco_pkg = get_package_share_directory('ros2_aruco')

    # base_link -> camera. The TurtleBot description has no camera link, so without
    # this every ar_marker_N frame is an orphan with no relationship to the map..
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_camera',
        output='screen',
        arguments=[
            '--x', '0.115', '--y', '0.0', '--z', '0.0',
            '--roll', '-1.5708', '--pitch', '0.0', '--yaw', '-1.5708',
            '--frame-id', 'base_link', '--child-frame-id', 'camera',
        ],
    )

    aruco = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(aruco_pkg, 'launch', 'aruco_recognition.launch.py')
        ),
    )

    return LaunchDescription([
        camera_tf,
        aruco,
    ])
