#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_model', default_value='ur3e'),
        DeclareLaunchArgument('robot_prefix', default_value=''),
        DeclareLaunchArgument('tcp_frame', default_value='tool0'),
        DeclareLaunchArgument('is_robot_real', default_value='false'),

        Node(
            package='manymove_color_signal',
            executable='bt_client_ur_color_signal',
            output='screen',
            parameters=[{
                'robot_model': LaunchConfiguration('robot_model'),
                'robot_prefix': LaunchConfiguration('robot_prefix'),
                'tcp_frame': LaunchConfiguration('tcp_frame'),
                'is_robot_real': LaunchConfiguration('is_robot_real'),
            }],
        ),
    ])

