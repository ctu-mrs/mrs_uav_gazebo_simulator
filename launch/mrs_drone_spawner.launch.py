#!/usr/bin/env python3

import launch
import os

from launch_ros.actions import Node 
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
        LaunchConfiguration,
        IfElseSubstitution,
        PythonExpression,
        PathJoinSubstitution,
        EnvironmentVariable,
        )

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "mrs_uav_gazebo_simulation"

    this_pkg_path = get_package_share_directory(pkg_name)
    node_name='mrs_drone_spawner'

    # #{ custom_config

    custom_config_arg = LaunchConfiguration('custom_config')

    ld.add_action(DeclareLaunchArgument(
        'custom_config',
        default_value="",
        description="Path to the custom configuration file. The path can be absolute, starting with '/' or relative to the current working directory",
        ))

    # This logic correctly creates a substitution that resolves to an absolute path.
    custom_config_path = IfElseSubstitution(
            condition=PythonExpression(['"', custom_config_arg, '" != "" and ', 'not "', custom_config_arg, '".startswith("/")']),
            if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), custom_config_arg]),
            else_value=custom_config_arg
            )

    # #} end of custom_config

    # #{ log_level

    ld.add_action(DeclareLaunchArgument(name='log_level', default_value='info'))

    # #} end of log_level

    ld.add_action(DeclareLaunchArgument(
        'spawner_params',
        default_value=os.path.join(this_pkg_path, 'config', 'spawner_params.yaml'),
        description='Path to the default spawner configuration file. The path can be absolute, starting with "/" or relative to the current working directory',
    ))
    

    ld.add_action(
            Node(
                name=node_name,
                package=pkg_name,
                executable='mrs_drone_spawner',
                output="screen",
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                parameters=[
                    # This passes the custom_config path as a parameter named 'custom_config'
                    {'custom_config': custom_config_path},
                    LaunchConfiguration('spawner_params'),
                    ],

                remappings=[
                    ('spawn', '~/spawn'),
                ],
                )
            )

    return ld
