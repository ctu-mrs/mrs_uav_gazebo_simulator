import os
import multiprocessing
import time
import jinja2

from mrs_uav_gazebo_simulator.utils.spawner_exceptions import *
from mrs_uav_gazebo_simulator.utils.spawner_types import Px4MavlinkConfig

from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource


class Px4MavlinkManager():

    # #{ __init__(self, ros_node, gazebo_simulator_path, px4_mavlink_config, tempfile_folder, jinja_templates)
    def __init__(self, ros_node: Node, gazebo_simulator_path: str, px4_mavlink_config: Px4MavlinkConfig,
                 tempfile_folder: str, jinja_templates: dict):
        self._ros_node = ros_node

        px4_api_path = get_package_share_directory('mrs_uav_px4_api')
        self._mavros_launch_path = os.path.join(px4_api_path, 'launch', 'mavros.launch')
        self._mavros_px4_config_path = os.path.join(px4_api_path, 'config')
        self._mavros_px4_config_template_name = 'mavros_px4_config.jinja.yaml'
        self._px4_fimrware_launch_path = os.path.join(gazebo_simulator_path, 'launch',
                                                      'run_simulation_firmware.launch.py')
        self._mavros_plugin_list = os.path.join(self._mavros_px4_config_path, 'mavros_plugins.yaml')

        self._px4_mavlink_config = px4_mavlink_config

        self._tempfile_folder = tempfile_folder
        self._jinja_templates = jinja_templates

    # #}

    # #{ launch_mavros(self, robot_params)
    def launch_mavros(self, robot_params):
        name = robot_params['name']
        self._ros_node.get_logger().info(f'Launching mavros for {name}')

        launch_arguments = {
            'fcu_url': str(robot_params['mavlink_config']['fcu_url']),
            'gcs_url': '',  # do not connect to QGC using mavros, we create a dedicated mavlink stream instead
            'tgt_system': str(robot_params['ID'] + 1),
            'tgt_component': str(1),
            'pluginlists_yaml': self._mavros_plugin_list,
            'config_yaml': str(robot_params['mavros_px4_config']),
            'namespace': name + '/mavros',
            'use_sim_time': 'true',
            'base_link_frame_id': name + '/base_link',
            'odom_frame_id': name + '/odom',
            'map_frame_id': name + '/map'
        }

        ld = LaunchDescription([
            IncludeLaunchDescription(
                XMLLaunchDescriptionSource(self._mavros_launch_path),
                launch_arguments=launch_arguments.items(),
            )
        ])

        self._ros_node.get_logger().info(f'launch_arguments: {launch_arguments}')
        launch_service = LaunchService(debug=False)
        launch_service.include_launch_description(ld)
        mavros_process = multiprocessing.Process(target=launch_service.run)

        try:
            mavros_process.start()
        except Exception as e:
            self._ros_node.get_logger().error(f'Could not start mavros for {name}. Node failed to launch: {e}')
            raise CouldNotLaunch('Mavros failed to launch')

        self._ros_node.get_logger().info(f'Mavros for {name} launched')
        return mavros_process

    # #}

    # #{ launch_px4_firmware(self, robot_params)
    def launch_px4_firmware(self, robot_params):
        if self._px4_mavlink_config.firmware_launch_delay > 0:
            self._ros_node.get_logger().info(
                f'Waiting for {self._px4_mavlink_config.firmware_launch_delay} s before launching firmware')
            time.sleep(self._px4_mavlink_config.firmware_launch_delay)

        name = robot_params['name']
        self._ros_node.get_logger().info(f'Launching PX4 firmware for {name}')

        package_name = self._jinja_templates[robot_params['model']].package_name
        package_path = get_package_share_directory(package_name)

        romfs_path = os.path.join(str(package_path), 'ROMFS')

        if not os.path.exists(romfs_path) or not os.path.isdir(romfs_path):
            self._ros_node.get_logger().error(f'Could not start PX4 firmware for {name}. ROMFS folder not found')
            raise CouldNotLaunch('ROMFS folder not found')

        launch_arguments = {
            'ID': str(robot_params['ID']),
            'PX4_SIM_MODEL': str(robot_params['model']),
            'ROMFS_PATH': str(romfs_path),
            'CONNECT_TO_QGC': str(self._px4_mavlink_config.stream_for_qgc)
        }

        ld = LaunchDescription([
            IncludeLaunchDescription(PythonLaunchDescriptionSource(self._px4_fimrware_launch_path),
                                     launch_arguments=launch_arguments.items())
        ])

        launch_service = LaunchService(debug=False)
        launch_service.include_launch_description(ld)
        firmware_process = multiprocessing.Process(target=launch_service.run)

        try:
            firmware_process.start()
        except Exception as e:
            self._ros_node.get_logger().error(f'Could not start PX4 firmware for {name}. Node failed to launch: {e}')
            raise CouldNotLaunch('PX4 failed to launch')

        self._ros_node.get_logger().info(f'PX4 firmware for {name} launched')
        if self._px4_mavlink_config.stream_for_qgc:
            qgc_port = robot_params['mavlink_config']['udp_qgc_port_remote']
            self._ros_node.get_logger().info(f'QGC connection for {name} created at localhost UDP port {qgc_port}')
        return firmware_process

    # #}

    # #{ get_mavlink_config_for_robot(self, ID)
    def get_mavlink_config_for_robot(self, ID):
        '''Creates a mavlink port configuration based on default values offset by ID

        NOTE: The offsets have to match values assigned in px4-rc.* files located in package_root/ROMFS/px4fmu_common/init.d-posix!!

        '''
        mavlink_config = {}
        udp_offboard_port_local = self._px4_mavlink_config.vehicle_base_port + (4 * ID)
        udp_offboard_port_remote = self._px4_mavlink_config.vehicle_base_port + (4 * ID) + 1
        udp_qgc_port_local = self._px4_mavlink_config.vehicle_base_port + (4 * ID) + 2
        udp_qgc_port_remote = self._px4_mavlink_config.vehicle_base_port + (4 * ID) + 3
        mavlink_config['udp_offboard_port_remote'] = udp_offboard_port_remote
        mavlink_config['udp_offboard_port_local'] = udp_offboard_port_local
        mavlink_config['udp_qgc_port_remote'] = udp_qgc_port_remote
        mavlink_config['udp_qgc_port_local'] = udp_qgc_port_local
        mavlink_config['fcu_url'] = f'udp://127.0.0.1:{udp_offboard_port_remote}@127.0.0.1:{udp_offboard_port_local}'

        return mavlink_config

    # #}

    # #{ generate_mavros_px4_config(self, uav_name)
    def generate_mavros_px4_config(self, uav_name):

        jinja_env = jinja2.Environment(loader=jinja2.FileSystemLoader(self._mavros_px4_config_path), autoescape=False)

        template = jinja_env.get_template(self._mavros_px4_config_template_name)

        rendered_template = template.render(uav_name=uav_name)

        filename = f'mavros_px4_config_{uav_name}.yaml'
        filepath = os.path.join(self._tempfile_folder, filename)

        with open(filepath, 'w', encoding='utf-8') as f:
            f.write(rendered_template)
            self._ros_node.get_logger().info(f'Mavros PX4 config for {uav_name} written to {filepath}')

        return filepath

    # #}
