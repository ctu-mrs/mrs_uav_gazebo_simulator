from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    GroupAction,
    ExecuteProcess
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare, FindPackagePrefix

def generate_launch_description():
    """
    Generates the launch description for running PX4 SITL firmware.
    """

    declared_args = [
        DeclareLaunchArgument('ID', description='The unique ID for the UAV.'),
        DeclareLaunchArgument('PX4_SIM_MODEL', description='The PX4 simulation model (e.g., iris).'),
        DeclareLaunchArgument('PX4_ESTIMATOR', default_value='ekf2', description='The PX4 estimator to use.'),
        DeclareLaunchArgument('CONNECT_TO_QGC', default_value='0', description='Create a mavlink stream for QGroundControl.'),
        DeclareLaunchArgument(
            'ROMFS_PATH',
            default_value=PathJoinSubstitution([
                FindPackageShare('mrs_uav_gazebo_simulator'), 'ROMFS'
            ]),
            description='Path to the PX4 ROMFS directory.'
        ),
        DeclareLaunchArgument('interactive', default_value='true', description='Run PX4 in interactive mode.')
    ]

    px4_interactive_mode = PythonExpression([
        "'' if '", LaunchConfiguration('interactive'), "' == 'true' else '-d'"
    ])

    # This correctly launches PX4 as an external command without injecting ROS arguments.
    px4_sitl_process = ExecuteProcess(
        cmd=[
            PathJoinSubstitution([FindPackagePrefix('px4'), 'lib/px4/px4']),
            PathJoinSubstitution([LaunchConfiguration('ROMFS_PATH'), 'px4fmu_common']),
            '-s',
            'etc/init.d-posix/rcS',
            px4_interactive_mode,
            '-i',
            LaunchConfiguration('ID'),
            '-w',
            [TextSubstitution(text='/tmp/sitl_uav_'), LaunchConfiguration('ID')]
        ],
        name=['sitl_uav_', LaunchConfiguration('ID')],
        output='screen'
    )

    namespaced_group = GroupAction(
        actions=[
            PushRosNamespace(namespace=['uav', LaunchConfiguration('ID')]),

            # Set environment variables for the PX4 process
            SetEnvironmentVariable('PX4_SIM_MODEL', LaunchConfiguration('PX4_SIM_MODEL')),
            SetEnvironmentVariable('PX4_ESTIMATOR', LaunchConfiguration('PX4_ESTIMATOR')),
            SetEnvironmentVariable('PX4_GZ_MODEL_NAME', [TextSubstitution(text='uav'), LaunchConfiguration('ID')]),
            SetEnvironmentVariable('CONNECT_TO_QGC', LaunchConfiguration('CONNECT_TO_QGC')),

            # The process to run
            px4_sitl_process
        ]
    )

    return LaunchDescription(declared_args + [namespaced_group])
