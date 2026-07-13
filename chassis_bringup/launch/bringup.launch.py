"""Bring up the chassis driver node together with robot_state_publisher."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xacro_file_arg = DeclareLaunchArgument(
            "xacro_file",
            default_value="chassis_DD-S.xacro",
            description="Xacro file name under chassis_description/urdf/",
        )
    vehicle_param_file_arg = DeclareLaunchArgument(
        "vehicle_param_file",
        default_value="vehicle_param_DD-S.yaml",
        description="Parameter yaml file name under chassis_bringup/config/",
    )

    xacro_path = PathJoinSubstitution([
        FindPackageShare("chassis_description"),
        "urdf",
        LaunchConfiguration("xacro_file"),
    ])
    vehicle_param_path = PathJoinSubstitution([
        FindPackageShare("chassis_bringup"),
        "config",
        LaunchConfiguration("vehicle_param_file"),
    ])

    robot_description = Command(["xacro ", xacro_path])

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    chassis_driver_node = Node(
        package="chassis_driver",
        executable="chassis_driver_node",
        output="screen",
        parameters=[vehicle_param_path],
    )

    return LaunchDescription([
        xacro_file_arg,
        vehicle_param_file_arg,
        robot_state_publisher_node,
        chassis_driver_node,
    ])