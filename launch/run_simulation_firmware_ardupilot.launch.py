from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Generates the launch description for running ArduPilot SITL firmware (JSON backend).

    The process talks to the ArduPilotPlugin in Gazebo over the JSON FDM interface
    (ports 9002/9003 + 10*instance) and exposes MAVLink over UDP for MAVROS
    (serial1) and, optionally, QGroundControl (serial2).
    """

    declared_args = [
        DeclareLaunchArgument('ID', description='SITL instance index (identifies the UAV)'),
        DeclareLaunchArgument('ARDUPILOT_BINARY', description='Path to the arducopter SITL binary'),
        DeclareLaunchArgument('PARAMS_FILE', description='Full path to the SITL parameter defaults file'),
        DeclareLaunchArgument('SYSID', default_value='1', description='MAVLink system ID of this SITL instance'),
        DeclareLaunchArgument('HOME', default_value='',
                              description='Simulated world home "lat,lon,alt_amsl,heading_deg"; empty keeps SITL default'),
        DeclareLaunchArgument('WORKING_DIR', description='Working directory of the SITL process (eeprom/parameters storage)'),
        DeclareLaunchArgument('SIM_ADDRESS', default_value='127.0.0.1',
                              description='Address of the machine running Gazebo (JSON FDM endpoint)'),
        DeclareLaunchArgument('MAVROS_OUT_ARG', default_value='',
                              description='Serial option exposing MAVLink to MAVROS, e.g. --serial0=udpclient:127.0.0.1:14550'),
        DeclareLaunchArgument('QGC_OUT_ARG', default_value='',
                              description='Serial option exposing MAVLink to QGroundControl, e.g. --serial1=udpclient:127.0.0.1:14560'),
    ]

    def launch_setup(context, *args, **kwargs):
        id = LaunchConfiguration('ID').perform(context)
        home = LaunchConfiguration('HOME').perform(context)
        mavros_out_arg = LaunchConfiguration('MAVROS_OUT_ARG').perform(context)
        qgc_out_arg = LaunchConfiguration('QGC_OUT_ARG').perform(context)

        cmd = [
            LaunchConfiguration('ARDUPILOT_BINARY').perform(context),
            '--model', 'JSON',
            '--sim-address', LaunchConfiguration('SIM_ADDRESS').perform(context),
            '-I', id,
            '--defaults', LaunchConfiguration('PARAMS_FILE').perform(context),
            '--sysid', LaunchConfiguration('SYSID').perform(context),
        ]

        if home != '':
            cmd += ['--home', home]

        if mavros_out_arg != '':
            cmd.append(mavros_out_arg)

        if qgc_out_arg != '':
            cmd.append(qgc_out_arg)

        sitl_process = ExecuteProcess(
            cmd=cmd,
            name=f'sitl_ardu_uav_{id}',
            cwd=LaunchConfiguration('WORKING_DIR').perform(context),
            output='screen',
            emulate_tty=True,
        )

        return [sitl_process]

    return LaunchDescription(declared_args + [OpaqueFunction(function=launch_setup)])
