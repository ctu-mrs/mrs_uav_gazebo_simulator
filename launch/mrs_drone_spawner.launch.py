#!/usr/bin/env python3

import launch
import os

from launch_ros.actions import Node 
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, IfElseSubstitution, PythonExpression, PathJoinSubstitution, EnvironmentVariable

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = launch.LaunchDescription()

    # Package Directories
    pkg_mrs_uav_gazebo_simulation = get_package_share_directory('mrs_uav_gazebo_simulation')

    # Launch arguments declaration
    ld.add_action(DeclareLaunchArgument(
        'custom_config',
        default_value="",
        description="Path to the custom configuration file. The path can be absolute, starting with '/' or relative to the current working directory",
        ))

    ld.add_action(DeclareLaunchArgument(
        'spawner_params',
        default_value = PathJoinSubstitution([
            pkg_mrs_uav_gazebo_simulation, 'config', 'spawner_params.yaml'
        ]),
        description='Path to the default spawner configuration file. The path can be absolute, starting with "/" or relative to the current working directory',
    ))

    ld.add_action(DeclareLaunchArgument(
        'debug', default_value = 'false',
        description='Run spawner with debug log level'
    ))

    # This logic correctly creates a substitution that resolves to an absolute path.
    custom_config_path = IfElseSubstitution(
            condition=PythonExpression(['"', LaunchConfiguration('custom_config'), '" != "" and ', 'not "', LaunchConfiguration('custom_config'), '".startswith("/")']),
            if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), LaunchConfiguration('custom_config')]),
            else_value=LaunchConfiguration('custom_config')
            )

    
    # Conditionally set the log level using PythonExpression
    log_level = PythonExpression([
        "'debug' if '", LaunchConfiguration('debug'), "' == 'true' else 'info'"
    ])

    ld.add_action(
            Node(
                name='mrs_drone_spawner',
                package='mrs_uav_gazebo_simulation',
                executable='mrs_drone_spawner',
                output="screen",
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[
                    # This passes the custom_config path as a parameter named 'custom_config'
                    {'custom_config': custom_config_path},
                    LaunchConfiguration('spawner_params'),
                    ],

                remappings=[
                    ('spawn', '~/spawn'),
                    ('create_entity', '/ros_gz_bridge/create_entity'),
                    ('delete_entity', '/ros_gz_bridge/delete_entity'),
                ],
                )
            )

    return ld
