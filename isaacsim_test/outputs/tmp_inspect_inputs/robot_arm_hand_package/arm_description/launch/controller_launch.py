# ROS2 launch file for ros2_control / gazebo_ros2_control
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', 'position_controller'],
            output='screen',
        ),
    ])
