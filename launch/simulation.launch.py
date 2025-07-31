"""Launch gzsim + ros_gz_bridge + mrs_spawner in a component container."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from ros_gz_bridge.actions import RosGzBridge
from ros_gz_sim.actions import GzServer
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    pkg_mrs_uav_gazebo_simulation = get_package_share_directory('mrs_uav_gazebo_simulation')

    declare_bridge_name_cmd = DeclareLaunchArgument(
        'bridge_name', 
        default_value='ros_gz_bridge',
        description='Name of the bridge'
    )

    declare_bridge_config_file_cmd = DeclareLaunchArgument(
        'bridge_config_file', 
        default_value = PathJoinSubstitution([
                pkg_mrs_uav_gazebo_simulation, 'config', 'ros_gz_bridge.yaml'
            ]),
        description='YAML config file for ros_gz_bridge:'
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name',
        default_value='ros_gz_container',
        description='Name of container that nodes will load in if use composition',
    )

    declare_create_own_container_cmd = DeclareLaunchArgument(
        'create_own_container',
        default_value='False',
        description='Whether we should start a ROS container when using composition.',
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='False', description='Use composed bringup if True'
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.',
    )

    declare_bridge_log_level_cmd = DeclareLaunchArgument(
        'bridge_log_level', default_value='info', description='Bridge log level'
    )

    declare_bridge_params_cmd = DeclareLaunchArgument(
        'bridge_params', default_value='', description='Extra parameters to pass to the bridge.'
    )

    declare_world_sdf_file_cmd = DeclareLaunchArgument(
        'world_sdf_file', default_value='empty.sdf',
        description='Path to the SDF world file'
    )

    declare_world_sdf_string_cmd = DeclareLaunchArgument(
        'world_sdf_string', default_value=TextSubstitution(text=''),
        description='SDF world string'
    )
    
    declare_spawner_config_cmd = DeclareLaunchArgument(
        'spawner_config', 
        default_value = PathJoinSubstitution([
                pkg_mrs_uav_gazebo_simulation, 'config', 'spawner_params.yaml'
            ]),
        description='Configuration file for the custom spawner.'
    )

    gz_server_action = GzServer(
        world_sdf_file=LaunchConfiguration('world_sdf_file'),
        world_sdf_string=LaunchConfiguration('world_sdf_string'),
        container_name=LaunchConfiguration('container_name'),
        create_own_container=LaunchConfiguration('create_own_container'),
        use_composition=LaunchConfiguration('use_composition'),
    )

    ros_gz_bridge_action = RosGzBridge(
        bridge_name=LaunchConfiguration('bridge_name'),
        config_file=LaunchConfiguration('bridge_config_file'),
        container_name=LaunchConfiguration('container_name'),
        create_own_container=str(False),
        namespace=LaunchConfiguration('namespace'),
        use_composition=LaunchConfiguration('use_composition'),
        use_respawn=LaunchConfiguration('use_respawn'),
        log_level=LaunchConfiguration('bridge_log_level'),
        bridge_params=LaunchConfiguration('bridge_params'),
        extra_bridge_params=[dict()], # Fix for broken Jazzy release
    )

    # Include your custom spawner launch file
    drone_spawner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_mrs_uav_gazebo_simulation, 'launch', 'mrs_drone_spawner.launch.py'
            ])
        ),
        launch_arguments={
            'spawner_config': LaunchConfiguration('spawner_config')
        }.items()
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_bridge_name_cmd)
    ld.add_action(declare_bridge_config_file_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_create_own_container_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_bridge_log_level_cmd)
    ld.add_action(declare_bridge_params_cmd)
    ld.add_action(declare_world_sdf_file_cmd)
    ld.add_action(declare_world_sdf_string_cmd)
    ld.add_action(declare_spawner_config_cmd)
    # Add the actions to launch all of the bridge + gz_server nodes
    ld.add_action(gz_server_action)
    ld.add_action(ros_gz_bridge_action)

    return ld
