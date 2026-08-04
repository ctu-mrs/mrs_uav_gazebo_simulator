import os
import multiprocessing
import time

from mrs_uav_gazebo_simulator.utils.spawner_exceptions import *
from mrs_uav_gazebo_simulator.utils.spawner_types import ArduPilotSitlConfig

from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandLong

# MAVLink message intervals requested after the FCU comes online.
# (message id, interval in microseconds)
# ArduPilot does not stream telemetry until a GCS explicitly asks for it and
# MAVROS does not request streams on its own, so the spawner has to.
STREAM_REQUESTS = [
    (105, 20000),  # HIGHRES_IMU            50 Hz - imu data
    (30, 100000),  # ATTITUDE               10 Hz - imu orientation
    (33, 100000),  # GLOBAL_POSITION_INT    10 Hz - gnss position
    (24, 100000),  # GPS_RAW_INT            10 Hz - gps status
    (32, 20000),   # LOCAL_POSITION_NED     50 Hz - local odometry
    (27, 20000),   # RAW_IMU                50 Hz - magnetic field
    (1, 500000),   # SYS_STATUS              2 Hz - battery voltage
    (65, 500000),  # RC_CHANNELS             2 Hz - rc channels
    (74, 500000),  # VFR_HUD                 2 Hz - compass heading
    (132, 100000), # DISTANCE_SENSOR        10 Hz - rangefinders (if present)
    (147, 1000000),# BATTERY_STATUS          1 Hz - battery state
]

MAV_CMD_SET_MESSAGE_INTERVAL = 511


class ArduPilotSitlManager():
    '''Launches per-robot ArduPilot SITL instances and MAVROS.

    Port layout follows the ArduPilot multi-instance convention (base + 10*instance):
      - fdm_base_port + 10*ID:   JSON FDM backend (servo packets SITL -> ArduPilotPlugin)
      - 5760 + 10*ID:            SITL GCS TCP server (SERIAL0 default), consumed by MAVROS
      - qgc_base_port + 10*ID:   optional UDP GCS link (SERIAL1) for QGroundControl

    MAVLink sysid of the SITL instance is set with --sysid ID+1, matching the
    tgt_system passed to MAVROS.
    '''

    # The keyword that enables this backend in the spawn service input string
    SPAWNER_KEYWORD = 'use-ardupilot'

    # #{ __init__(self, ros_node, gazebo_simulator_path, ardupilot_config, jinja_templates)
    def __init__(self, ros_node: Node, gazebo_simulator_path: str, ardupilot_config: ArduPilotSitlConfig,
                 jinja_templates: dict):
        self._ros_node = ros_node

        ardupilot_api_path = get_package_share_directory('mrs_uav_ardupilot_api')
        self._mavros_launch_path = os.path.join(ardupilot_api_path, 'launch', 'mavros.launch')
        self._mavros_pluginlists_path = os.path.join(ardupilot_api_path, 'config', 'mavros_apm_pluginlists.yaml')
        self._mavros_config_path = os.path.join(ardupilot_api_path, 'config', 'mavros_apm_config.yaml')

        self._sitl_launch_path = os.path.join(gazebo_simulator_path, 'launch',
                                              'run_simulation_firmware_ardupilot.launch.py')

        self._config = ardupilot_config

        self._jinja_templates = jinja_templates

        self._resolve_paths()

    # #}

    # #{ _resolve_paths(self)
    def _resolve_paths(self):
        '''Fills in empty paths from the environment / package defaults and checks their existence'''

        if self._config.binary_path == '':
            ardupilot_home = os.environ.get('ARDUPILOT_HOME', os.path.expanduser('~/ardupilot'))
            self._config.binary_path = os.path.join(ardupilot_home, 'build', 'sitl', 'bin', 'arducopter')

        if self._config.defaults_file == '':
            self._config.defaults_file = os.path.join(self._sitl_launch_path, '..', '..', 'config', 'ardupilot',
                                                      'mrs_copter_defaults.parm')
            self._config.defaults_file = os.path.normpath(self._config.defaults_file)

    # #}

    # #{ get_ardupilot_config_for_robot(self, ID)
    def get_ardupilot_config_for_robot(self, ID):
        '''Creates the per-robot ardupilot configuration, injected into spawner_args for jinja rendering

        The fdm_port_in value has to match the port the SITL JSON backend sends
        servo packets to (Default: 9002 + 10*instance).
        '''
        config = {}
        config['fdm_port_in'] = self._config.fdm_base_port + 10 * ID
        config['gcs_tcp_port'] = 5760 + 10 * ID
        config['qgc_udp_port'] = self._config.qgc_base_port + 10 * ID
        config['sysid'] = ID + 1
        config['home'] = self._config.home
        # TCP client link to the SITL GCS server, the standard way to connect
        # MAVROS to an ArduPilot SITL instance
        config['fcu_url'] = f'tcp://127.0.0.1:{config["gcs_tcp_port"]}@'
        return config

    # #}

    # #{ launch_mavros(self, robot_params)
    def launch_mavros(self, robot_params):
        name = robot_params['name']
        launch_arguments = {
            'fcu_url': str(robot_params['ardupilot_config']['fcu_url']),
            'gcs_url': '',  # do not connect to QGC using mavros, SITL exposes a dedicated stream instead
            'tgt_system': str(robot_params['ardupilot_config']['sysid']),
            'tgt_component': '1',
            'fcu_protocol': 'v2.0',
            'pluginlists_yaml': self._mavros_pluginlists_path,
            'config_yaml': self._mavros_config_path,
            'namespace': 'mavros',
            'use_sim_time': 'true',
        }

        launch_description = LaunchDescription([
            GroupAction(actions=[
                PushRosNamespace(name),
                IncludeLaunchDescription(
                    AnyLaunchDescriptionSource(self._mavros_launch_path),
                    launch_arguments=launch_arguments.items(),
                ),
            ]),
        ])

        self._ros_node.get_logger().info(f'launch_arguments: {launch_arguments}')
        launch_service = LaunchService(debug=False)
        launch_service.include_launch_description(launch_description)
        mavros_process = multiprocessing.Process(target=launch_service.run)

        try:
            mavros_process.start()
        except Exception as e:
            self._ros_node.get_logger().error(f'Could not start mavros for {name}. Node failed to launch: {e}')
            raise CouldNotLaunch('Mavros failed to launch')

        self._ros_node.get_logger().info(f'Mavros for {name} launched')
        self._register_stream_initialization(name)
        return mavros_process

    # #}

    # #{ launch_sitl(self, robot_params)
    def launch_sitl(self, robot_params):
        if self._config.firmware_launch_delay > 0:
            self._ros_node.get_logger().info(
                f'Waiting for {self._config.firmware_launch_delay} s before launching ArduPilot SITL')
            time.sleep(self._config.firmware_launch_delay)

        name = robot_params['name']
        ID = robot_params['ID']
        ardupilot_config = robot_params['ardupilot_config']

        if not os.path.isfile(self._config.binary_path):
            self._ros_node.get_logger().error(
                f'Could not start ArduPilot SITL for {name}. Binary not found at "{self._config.binary_path}". '
                'Build ArduPilot SITL (see README) or set the "ardupilot_config/binary_path" spawner parameter.')
            raise CouldNotLaunch('ArduPilot SITL binary not found')

        if not os.path.isfile(self._config.defaults_file):
            self._ros_node.get_logger().error(
                f'Could not start ArduPilot SITL for {name}. Defaults file not found at "{self._config.defaults_file}"'
            )
            raise CouldNotLaunch('ArduPilot defaults file not found')

        working_dir = f'/tmp/sitl_ardu_uav_{ID}'
        os.makedirs(working_dir, exist_ok=True)

        self._ros_node.get_logger().info(f'Launching ArduPilot SITL for {name} (instance {ID})')

        launch_arguments = {
            'ID': str(ID),
            'ARDUPILOT_BINARY': str(self._config.binary_path),
            'PARAMS_FILE': str(self._config.defaults_file),
            'SYSID': str(ardupilot_config['sysid']),
            'HOME': str(ardupilot_config['home']),
            'WORKING_DIR': str(working_dir),
            # serial0 keeps the SITL default: a GCS TCP server on 5760+10*ID.
            # MAVROS connects to it as a TCP client, which is also what makes
            # the SITL start streaming telemetry (MAVProxy-style handshake).
            'MAVROS_OUT_ARG': '',
            'QGC_OUT_ARG':
            f'--serial1=udpclient:127.0.0.1:{ardupilot_config["qgc_udp_port"]}' if self._config.stream_for_qgc else '',
        }

        launch_description = LaunchDescription([
            IncludeLaunchDescription(AnyLaunchDescriptionSource(self._sitl_launch_path),
                                     launch_arguments=launch_arguments.items())
        ])

        launch_service = LaunchService(debug=False)
        launch_service.include_launch_description(launch_description)
        sitl_process = multiprocessing.Process(target=launch_service.run)

        try:
            sitl_process.start()
        except Exception as e:
            self._ros_node.get_logger().error(
                f'Could not start ArduPilot SITL for {name}. Process failed to launch: {e}')
            raise CouldNotLaunch('ArduPilot SITL failed to launch')

        self._ros_node.get_logger().info(f'ArduPilot SITL for {name} launched')
        if self._config.stream_for_qgc:
            self._ros_node.get_logger().info(
                f'QGC connection for {name} created at localhost UDP port {ardupilot_config["qgc_udp_port"]}')
        return sitl_process

    # #}

    # #{ stream initialization (MAVLink message intervals)

    def _register_stream_initialization(self, name):
        '''Starts periodically checking mavros connection of the given UAV and
        requests telemetry message intervals from ArduPilot once it is online.'''
        if not hasattr(self, '_stream_init_entries'):
            self._stream_init_entries = {}
            # lazy one-shot timer at 1 Hz, drives all pending robots
            self._stream_init_timer = self._ros_node.create_timer(1.0, self._stream_init_callback)

        entry = {
            'state': State(),
            'futures': [],
            'done': False,
        }
        entry['state_sub'] = self._ros_node.create_subscription(
            State, f'/{name}/mavros/state', lambda msg, n=name: self._state_callback(n, msg), 10)
        entry['client'] = self._ros_node.create_client(CommandLong, f'/{name}/mavros/cmd/command')
        self._stream_init_entries[name] = entry
        self._ros_node.get_logger().info(f'Registered stream initialization for {name}')

    def _state_callback(self, name, msg):
        self._stream_init_entries[name]['state'] = msg

    def _stream_init_callback(self):
        for name, entry in self._stream_init_entries.items():
            if entry['done']:
                continue

            if not entry['state'].connected:
                continue

            client = entry['client']
            if not client.service_is_ready():
                continue

            # wait for the previous batch before sending the next one
            if entry['futures']:
                if not all(f.done() for f in entry['futures']):
                    continue
                results = [f.result() for f in entry['futures']]
                if all(r is not None and r.success and r.result == 0 for r in results):
                    entry['done'] = True
                    self._ros_node.get_logger().info(f'Stream rates initialized for {name}')
                    continue
                self._ros_node.get_logger().warn(
                    f'Stream rate requests for {name} not fully accepted, retrying: '
                    f'{[(r.success, r.result) if r is not None else None for r in results]}')

            entry['futures'] = []
            for msg_id, interval_us in STREAM_REQUESTS:
                request = CommandLong.Request()
                request.command = MAV_CMD_SET_MESSAGE_INTERVAL
                request.param1 = float(msg_id)
                request.param2 = float(interval_us)
                entry['futures'].append(client.call_async(request))

    # #}
