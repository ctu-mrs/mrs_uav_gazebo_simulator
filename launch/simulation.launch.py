"""Launch gzsim + ros_gz_bridge + mrs_spawner in a component container."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # Package Directories
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_mrs_common_gazebo_resources = get_package_share_directory('mrs_gazebo_common_resources')
    pkg_mrs_uav_gazebo_simulation = get_package_share_directory('mrs_uav_gazebo_simulation')


    # Launch arguments declaration
    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value = PathJoinSubstitution([
                pkg_mrs_common_gazebo_resources, 'worlds', 'grass_plane.world'
            ]),
        description='Path to the SDF world file'
    )

    declare_spawner_config_cmd = DeclareLaunchArgument(
        'spawner_config', 
        default_value = PathJoinSubstitution([
                pkg_mrs_uav_gazebo_simulation, 'config', 'spawner_params.yaml'
            ]),
        description='Configuration file for the custom spawner.'
    )

    declare_spawner_debug_cmd = DeclareLaunchArgument(
        'spawner_debug', default_value = 'false',
        description='Run spawner with debug log level'
    )

    declare_bridge_debug_cmd = DeclareLaunchArgument(
        'bridge_debug', default_value = 'false',
        description='Run ros_gz_bridge with debug log level'
    )

    ## | ----------------------- Gazebo sim  ---------------------- |
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': LaunchConfiguration('world_file'),
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
                pkg_mrs_uav_gazebo_simulation, 'launch', 'mrs_drone_spawner.launch.py'
            ])
        ),
        launch_arguments={
            'spawner_config': LaunchConfiguration('spawner_config'),
            'debug': LaunchConfiguration('spawner_debug'),
        }.items()
    )

    return LaunchDescription(
        [
            # Launch arguments
            declare_world_file_cmd,
            declare_spawner_config_cmd,
            declare_spawner_debug_cmd,
            declare_bridge_debug_cmd,
            # Nodes and Launches
            gazebo,
            bridge,
            drone_spawner,
        ]
    )
