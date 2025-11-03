#!/usr/bin/env python3

import launch
import os

from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, IfElseSubstitution, PythonExpression, PathJoinSubstitution, EnvironmentVariable

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = launch.LaunchDescription()

    # Package Directories
    pkg_mrs_uav_gazebo_simulator = get_package_share_directory('mrs_uav_gazebo_simulator')

    # Launch arguments declaration
    ld.add_action(DeclareLaunchArgument(
        'custom_config',
        default_value = "",
        description="Path to the custom configuration file. The path can be absolute, starting with '/' or relative to the current working directory",
    ))

    ld.add_action(DeclareLaunchArgument(
        'default_spawner_config',
        default_value = PathJoinSubstitution([
            pkg_mrs_uav_gazebo_simulator, 'config', 'spawner_params.yaml'
        ]),
        description='Path to the default spawner configuration file. The path can be absolute, starting with "/" or relative to the current working directory',
    ))


    ld.add_action(DeclareLaunchArgument(
        'debug', default_value = 'false',
        description='Run spawner with debug log level'
    ))

    # Conditionally set the log level using PythonExpression
    log_level = PythonExpression([
        "'debug' if '", LaunchConfiguration('debug'), "' == 'true' else 'info'"
    ])

    log_config_args = LogInfo(msg=[
        '\n[mrs_drone_spawner.launch.py] Custom config file: ', LaunchConfiguration('custom_config'),
        '\n[mrs_drone_spawner.launch.py] Default config file: ', LaunchConfiguration('default_spawner_config')
    ])
    ld.add_action(log_config_args)

    ld.add_action(
            Node(
                name='mrs_drone_spawner',
                package='mrs_uav_gazebo_simulator',
                executable='mrs_drone_spawner',
                output="screen",
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[
                    LaunchConfiguration('default_spawner_config'),
                    LaunchConfiguration('custom_config')
                    ],

                remappings=[
                    ('spawn', '~/spawn'),
                    ('diagnostics', '~/diagnostics'),
                    ('create_entity', '/ros_gz_bridge/create_entity'),
                    ('delete_entity', '/ros_gz_bridge/delete_entity'),
                ],
                )
            )

    return ld