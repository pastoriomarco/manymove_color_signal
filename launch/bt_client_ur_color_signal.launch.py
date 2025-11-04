#!/usr/bin/env python3

from launch import LaunchDescription
from launch import logging
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot_model = LaunchConfiguration('robot_model').perform(context)
    prefix_raw = LaunchConfiguration('robot_prefix').perform(context)
    tcp_frame = LaunchConfiguration('tcp_frame').perform(context)
    is_robot_real_value = LaunchConfiguration('is_robot_real').perform(context)
    gripper_action_value = LaunchConfiguration('gripper_action_server').perform(context)

    prefix_clean = prefix_raw.strip('"')

    logger = logging.get_logger('bt_client_ur_color_signal.launch')

    def normalize_gripper_action(raw_value: str) -> str:
        stripped = raw_value.lstrip('/')
        if stripped.endswith('gripper_command'):
            converted = stripped[: -len('gripper_command')] + 'gripper_cmd'
            logger.warning(
                "Parameter 'gripper_action_server' value '%s' uses deprecated suffix "
                "'gripper_command'; using '%s' instead.",
                raw_value,
                converted,
            )
            return converted
        return stripped

    default_gripper_action = 'robotiq_gripper_controller/gripper_cmd'
    gripper_action_stripped = normalize_gripper_action(gripper_action_value)
    if not gripper_action_stripped:
        resolved_gripper_action = ''
    elif gripper_action_stripped == default_gripper_action:
        resolved_action = (
            f'{prefix_clean}{gripper_action_stripped}' if prefix_clean else gripper_action_stripped
        )
        resolved_gripper_action = f'/{resolved_action}'
    else:
        resolved_gripper_action = (
            gripper_action_value
            if gripper_action_value.startswith('/')
            else f'/{gripper_action_stripped}'
        )

    is_robot_real_flag = is_robot_real_value.lower() in ('true', '1', 't', 'yes', 'on')

    bt_node = Node(
        package='manymove_color_signal',
        executable='bt_client_ur_color_signal',
        output='screen',
        parameters=[{
            'robot_model': robot_model,
            'robot_prefix': prefix_clean,
            'tcp_frame': tcp_frame,
            'is_robot_real': is_robot_real_flag,
            'gripper_action_server': resolved_gripper_action,
        }],
    )

    return [bt_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_model', default_value='ur3e'),
        DeclareLaunchArgument('robot_prefix', default_value=''),
        DeclareLaunchArgument('tcp_frame', default_value='tool0'),
        DeclareLaunchArgument('is_robot_real', default_value='false'),
        DeclareLaunchArgument(
            'gripper_action_server',
            default_value='robotiq_gripper_controller/gripper_cmd',
        ),
        OpaqueFunction(function=launch_setup),
    ])
