import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    """
    Launches Gazebo, a ROS-Gazebo bridge, and a custom spawner.
    """

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_mrs_uav_gazebo_simulation = get_package_share_directory('mrs_uav_gazebo_simulation')

    declared_arguments = [
        DeclareLaunchArgument(
            'world_file',
            default_value='',
            description='Full path to the world file (.sdf) to be loaded.'
        ),
        DeclareLaunchArgument(
            'paused',
            default_value='false',
            description='If true, start Gazebo paused.'
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='If true, run Gazebo in server-only (headless) mode.'
        ),
        DeclareLaunchArgument(
            'verbose',
            default_value='false',
            description='If true, run Gazebo with verbose output.'
        ),
        DeclareLaunchArgument(
            'gz_args',
            default_value='',
            description='Extra arguments to pass to Gazebo.'
        ),
        DeclareLaunchArgument(
            'spawner_config',
            default_value='',
            description='Configuration file for the custom spawner.'
        ),
    ]


    # Set 'use_sim_time' parameter for all nodes
    set_use_sim_time = SetParameter(name='use_sim_time', value=True)

    # Launch Gazebo Simulator
    # An OpaqueFunction is used to correctly build the 'gz_args' string at launch time.
    launch_gazebo = OpaqueFunction(
        function=lambda context, *args, **kwargs: [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
                ),
                launch_arguments={
                    'gz_args': ' '.join([
                        LaunchConfiguration('world_file').perform(context),
                        '-p' if LaunchConfiguration('paused').perform(context) == 'true' else '',
                        '-r' if LaunchConfiguration('paused').perform(context) == 'false' else '',
                        '-s' if LaunchConfiguration('headless').perform(context) == 'true' else '',
                        '--verbose' if LaunchConfiguration('verbose').perform(context) == 'true' else '',
                        LaunchConfiguration('gz_args').perform(context),
                    ])
                }.items()
            )
        ]
    )

    # Launch the ROS-Gazebo Bridge
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
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

    return LaunchDescription(
        declared_arguments +
        [
            set_use_sim_time,
            launch_gazebo,
            ros_gz_bridge,
            drone_spawner,
        ]
    )
