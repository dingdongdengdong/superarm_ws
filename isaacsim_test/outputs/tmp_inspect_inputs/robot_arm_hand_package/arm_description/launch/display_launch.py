# ROS2 launch file for displaying URDF in RViz2
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_arm_hand_urdf_description')
    default_model = os.path.join(pkg_share, 'urdf', 'robot_arm_hand_urdf.xacro')
    default_rviz = os.path.join(pkg_share, 'launch', 'urdf.rviz')

    model_arg = DeclareLaunchArgument(name='model', default_value=default_model, description='Path to robot xacro')
    rvizconfig_arg = DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz, description='Path to rviz config')

    robot_description = Command(['xacro ', LaunchConfiguration('model')])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')]
    )

    return LaunchDescription([
        model_arg,
        rvizconfig_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
    ])
