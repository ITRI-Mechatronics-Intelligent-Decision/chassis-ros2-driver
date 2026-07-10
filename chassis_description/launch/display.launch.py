import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

rviz_config = PathJoinSubstitution([
    FindPackageShare("chassis_description"),
    "rviz",
    "static_view.rviz"
])


def generate_launch_description():
    xacro_path = PathJoinSubstitution([
        FindPackageShare('chassis_description'), 'urdf', 'chassis.urdf.xacro'
    ])

    robot_description = Command([FindExecutable(name='xacro'), ' ', xacro_path])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='static_view',
            arguments=["-d", rviz_config],
        ),
    ])
