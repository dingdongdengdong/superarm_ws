# ROS2 launch file for Gazebo simulation
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_arm_hand_urdf_description')
    xacro_path = os.path.join(pkg_share, 'urdf', 'robot_arm_hand_urdf.xacro')
    robot_description = Command(['xacro ', xacro_path])

    gazebo_share = get_package_share_directory('gazebo_ros')
    empty_world_launch = os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(empty_world_launch),
        launch_arguments={'paused': 'true', 'use_sim_time': 'true', 'gui': 'true'}.items()
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'robot_arm_hand_urdf'],
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
    ])
