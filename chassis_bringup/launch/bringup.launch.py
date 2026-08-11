"""Bring up the chassis driver node together with robot_state_publisher.

The chassis model is selected by name from chassis_description/config/models.yaml:

    ros2 launch chassis_bringup bringup.launch.py model:=DD-M-HH

xacro_file and vehicle_param_file stay available to override either half of the
registry entry, e.g. to test a work-in-progress parameter set.

localization_mode picks how the chassis pose reaches the TF tree:

    standalone         chassis broadcasts odom -> base_frame itself (default)
    external_takeover  chassis publishes /odom only, an external localiser owns
                       odom -> base_frame
    rep105_bridge      chassis broadcasts odom -> base_frame and map_odom_bridge
                       turns an external absolute pose into map -> odom
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.join(get_package_share_directory('chassis_description'), 'launch'))

from model_registry import default_model, model_entry  # noqa: E402

# mode -> (chassis broadcasts odom -> base_frame, start map_odom_bridge)
LOCALIZATION_MODES = {
    'standalone': (True, False),
    'external_takeover': (False, False),
    'rep105_bridge': (True, True),
}


def _launch_setup(context, *args, **kwargs):
    model = LaunchConfiguration('model').perform(context)
    xacro_file = LaunchConfiguration('xacro_file').perform(context)
    vehicle_param_file = LaunchConfiguration('vehicle_param_file').perform(context)
    system_service = LaunchConfiguration('system_service').perform(context)
    system_param_file = LaunchConfiguration('system_param_file').perform(context)
    localization_mode = LaunchConfiguration('localization_mode').perform(context)
    external_odom_topic = LaunchConfiguration('external_odom_topic').perform(context)

    if localization_mode not in LOCALIZATION_MODES:
        raise RuntimeError(
            f"Unknown localization_mode '{localization_mode}': "
            f'expected one of {sorted(LOCALIZATION_MODES)}'
        )
    publish_tf, start_bridge = LOCALIZATION_MODES[localization_mode]

    entry = model_entry(model)
    xacro_file = xacro_file or entry['xacro']
    vehicle_param_file = vehicle_param_file or entry['vehicle_param']

    xacro_path = os.path.join(
        get_package_share_directory('chassis_description'), 'urdf', xacro_file
    )
    vehicle_param_path = os.path.join(
        get_package_share_directory('chassis_bringup'), 'config', vehicle_param_file
    )

    for path in (xacro_path, vehicle_param_path):
        if not os.path.exists(path):
            raise RuntimeError(f"Model '{model}' refers to a missing file: {path}")

    robot_description = Command(['xacro ', xacro_path])

    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='chassis_driver',
            executable='chassis_driver_node',
            output='screen',
            parameters=[vehicle_param_path, {'publish_tf': publish_tf}],
        ),
    ]

    if start_bridge:
        nodes.append(
            Node(
                package='chassis_driver',
                executable='map_odom_bridge',
                output='screen',
                parameters=[vehicle_param_path],
                remappings=[('external_odom', external_odom_topic)],
            )
        )

    if system_service.lower() in ('true', '1'):
        system_param_path = os.path.join(
            get_package_share_directory('chassis_bringup'), 'config', system_param_file
        )
        if not os.path.exists(system_param_path):
            raise RuntimeError(f'Missing system parameter file: {system_param_path}')
        nodes.append(
            Node(
                package='chassis_system',
                executable='system_service_node',
                output='screen',
                parameters=[system_param_path],
            )
        )

    return nodes


def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=default_model(),
        description='Chassis model name declared in chassis_description/config/models.yaml',
    )
    xacro_file_arg = DeclareLaunchArgument(
        'xacro_file',
        default_value='',
        description='Override the xacro file name under chassis_description/urdf/',
    )
    vehicle_param_file_arg = DeclareLaunchArgument(
        'vehicle_param_file',
        default_value='',
        description='Override the parameter yaml file name under chassis_bringup/config/',
    )

    system_service_arg = DeclareLaunchArgument(
        'system_service',
        default_value='true',
        description='Start chassis_system/system_service_node (onboard computer shutdown service)',
    )
    system_param_file_arg = DeclareLaunchArgument(
        'system_param_file',
        default_value='system_param.yaml',
        description='Parameter yaml for system_service_node, under chassis_bringup/config/',
    )

    localization_mode_arg = DeclareLaunchArgument(
        'localization_mode',
        default_value='standalone',
        description=(
            'How the chassis pose reaches the TF tree: '
            f'{" | ".join(sorted(LOCALIZATION_MODES))}'
        ),
    )
    external_odom_topic_arg = DeclareLaunchArgument(
        'external_odom_topic',
        default_value='external_odom',
        description='nav_msgs/Odometry topic map_odom_bridge subscribes to',
    )

    return LaunchDescription([
        model_arg,
        xacro_file_arg,
        vehicle_param_file_arg,
        system_service_arg,
        system_param_file_arg,
        localization_mode_arg,
        external_odom_topic_arg,
        OpaqueFunction(function=_launch_setup),
    ])
