"""Launch gzsim + ros_gz_bridge + mrs_spawner in a component container."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # Package Directories
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_mrs_common_gazebo_resources = get_package_share_directory('mrs_gazebo_common_resources')
    pkg_mrs_uav_gazebo_simulator = get_package_share_directory('mrs_uav_gazebo_simulator')


    # Launch arguments declaration
    declare_world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value = PathJoinSubstitution([
                pkg_mrs_common_gazebo_resources, 'worlds', 'grass_plane.sdf'
            ]),
        description='Path to the SDF world file'
    )

    declare_spawner_config_arg = DeclareLaunchArgument(
        'spawner_config',
        default_value = "",
        description='Configuration file for the custom spawner.'
    )

    declare_spawner_debug_arg = DeclareLaunchArgument(
        'spawner_debug', default_value = 'false',
        description='Run spawner with debug log level'
    )

    declare_bridge_debug_arg = DeclareLaunchArgument(
        'bridge_debug', default_value = 'false',
        description='Run ros_gz_bridge with debug log level'
    )

    declare_gz_headless_arg = DeclareLaunchArgument(
        'gz_headless', default_value = 'false',
        description='Run gz in headless mode'
    )

    declare_gz_verbose_arg = DeclareLaunchArgument(
        'gz_verbose', default_value = 'false',
        description='Run gz in verbose mode'
    )

    declare_gz_sim_server_config_path_arg =  DeclareLaunchArgument(
        'GZ_SIM_SERVER_CONFIG_PATH',
        default_value=PathJoinSubstitution([
            pkg_mrs_uav_gazebo_simulator, 'config/gazebo_server.config'
        ]),
        description='Custom Gazebo server configuration file'
    )

    # Environment variables setting
    set_gz_config_env_var = SetEnvironmentVariable(
        name='GZ_SIM_SERVER_CONFIG_PATH',
        value=LaunchConfiguration('GZ_SIM_SERVER_CONFIG_PATH')
    )

    # Prepare the arguments for the Gazebo executable
    gz_args = [LaunchConfiguration('world_file'), ' -r']

    # Check if we should run gz in headless mode
    gz_args.append(PythonExpression([
        "'' if '", LaunchConfiguration('gz_headless'), "' == 'false' else ' -s'",
    ]))

    # Check if we should run gz in verbose mode
    gz_args.append(PythonExpression([
        "'' if '", LaunchConfiguration('gz_verbose'), "' == 'false' else ' -v'",
    ]))

    ## | ----------------------- Gazebo sim  ---------------------- |
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': gz_args
        }.items(),
    )

    ## | --------------------- GZ - ROS bridge -------------------- |

    # Conditionally set the log level using PythonExpression
    bridge_log_level = PythonExpression([
        "'debug' if '", LaunchConfiguration('bridge_debug'), "' == 'true' else 'info'"
    ])

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            # Clock (IGN -> ROS2)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # SpawnEntity (ROS2 -> IGN)
            '/world/default/create@ros_gz_interfaces/srv/SpawnEntity',
            # DeleteEntity (ROS2 -> IGN)
            '/world/default/remove@ros_gz_interfaces/srv/DeleteEntity',
            '--ros-args', '--log-level', bridge_log_level
        ],
        remappings=[
            ('/world/default/create', '~/create_entity'),
            ('/world/default/remove', '~/delete_entity'),
        ],
        output='screen'
    )

    ## | ---------------------- Drone Spawner --------------------- |

    drone_spawner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_mrs_uav_gazebo_simulator, 'launch', 'mrs_drone_spawner.launch.py'
            ])
        ),
        launch_arguments={
            'custom_config': LaunchConfiguration('spawner_config'),
            'debug': LaunchConfiguration('spawner_debug'),
        }.items()
    )

    return LaunchDescription(
        [
            # Launch arguments
            declare_world_file_arg,
            declare_spawner_config_arg,
            declare_spawner_debug_arg,
            declare_bridge_debug_arg,
            declare_gz_sim_server_config_path_arg,
            declare_gz_headless_arg,
            declare_gz_verbose_arg,
            # Environment variables
            set_gz_config_env_var,
            # Nodes and Launches
            gazebo,
            bridge,
            drone_spawner,
        ]
    )