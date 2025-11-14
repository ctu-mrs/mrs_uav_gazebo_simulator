#!/usr/bin/python3
import ast
import atexit
import copy
import csv
import datetime
import jinja2
import math
import os
import random
import re
import sys
import rclpy
from rclpy.node import Node
import rclpy.exceptions
import multiprocessing
import xml.dom.minidom
import time
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from mrs_uav_gazebo_simulator.utils.sdf_to_tf_publisher import SdfTfPublisherSingleton
from mrs_uav_gazebo_simulator.utils.spawner_enums import RosGzBridgeCategory, Px4MavlinkConfig
from mrs_uav_gazebo_simulator.utils.spawner_exceptions import *
from mrs_uav_gazebo_simulator.core.jinja_template_manager import JinjaTemplateManager
from mrs_uav_gazebo_simulator.core.ros_gz_bridge_manager import RosGzBridgeManager
from mrs_uav_gazebo_simulator.core.px4_mavlink_manager import Px4MavlinkManager

# ROS 2 Imports
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from ros_gz_interfaces.srv import SpawnEntity, DeleteEntity
from mrs_msgs.srv import String as StringSrv
from mrs_msgs.msg import GazeboSpawnerDiagnostics

glob_running_processes = []


# #{ dummy_function()
def dummy_function():
    '''Empty function to temporarily replace ros signal handlers'''
    pass


# #}


# #{ exit_handler()
def exit_handler():
    '''
    Kill all subprocesses started by the spawner to prevent orphaned processes (mainly px4 and mavros)
    '''
    print('[INFO] [MrsDroneSpawner]: Exit requested')

    if len(glob_running_processes) > 0:
        print(f'[INFO] [MrsDroneSpawner]: Shutting down {len(glob_running_processes)} subprocesses')

        num_zombies = 0
        for p in glob_running_processes:
            try:
                if p.is_alive():
                    p.terminate()
                    p.join()
                    print(f'[INFO] [MrsDroneSpawner]: Process {p.pid} terminated')
                else:
                    print(f'[INFO] [MrsDroneSpawner]: Process {p.pid} finished cleanly')
            except:
                num_zombies += 1

        if num_zombies > 0:
            print(f'\033[91m[ERROR] [MrsDroneSpawner]: Could not stop {num_zombies} subprocesses\033[91m')
            exit(1)

    if rclpy.ok():
        rclpy.shutdown()
        print('[INFO] [MrsDroneSpawner]: rclpy shut down.')

    print('[INFO] [MrsDroneSpawner]: Exited gracefully')


# #}


class MrsDroneSpawner(Node):

    def __init__(self):
        super().__init__('mrs_drone_spawner')

        resource_paths = self._handle_rosparams()

        self._template_manager = JinjaTemplateManager(self, resource_paths, self.template_suffix)
        self.jinja_templates = self._template_manager.get_jinja_templates()

        self.tempfile_folder = self._create_tempfile_folder()

        # Find launch files
        gazebo_simulator_path = get_package_share_directory('mrs_uav_gazebo_simulator')
        self.ros_gz_manager = RosGzBridgeManager(self, gazebo_simulator_path, self.tempfile_folder)
        self._px4_mavlink_manager = Px4MavlinkManager(self, gazebo_simulator_path, self._px4_mavlink_config,
                                                      self.tempfile_folder)

        # Setup ROS 2 communications
        self.spawn_server = self.create_service(StringSrv, 'spawn', self.callback_spawn)
        self.diagnostics_pub = self.create_publisher(GazeboSpawnerDiagnostics, 'diagnostics', 1)
        self.diagnostics_timer = self.create_timer(0.1, self.callback_diagnostics_timer)
        self.action_timer = self.create_timer(0.1, self.callback_action_timer)

        self.gazebo_spawn_proxy = self.create_client(SpawnEntity, 'create_entity')
        self.gazebo_delete_proxy = self.create_client(DeleteEntity, 'delete_entity')

        # Setup system variables
        self.spawn_called = False
        self.processing = False
        self.vehicle_queue = []
        self.queue_mutex = multiprocessing.Lock()
        self.active_vehicles = []
        self.assigned_ids = set()
        self.gazebo_spawn_future = None
        self.gazebo_delete_future = None
        self.gazebo_spawn_request_start_time = None

        # SdfToTf Publisher
        self.sdf_to_tf_publisher = SdfTfPublisherSingleton(self, self.tf_base_frame, self.tf_ignored_sensor_frames)
        self.sdf_files = []

        self.is_initialized = True
        self.get_logger().info('Initialized')

    # #{ handle_rosparams(self)
    def _handle_rosparams(self) -> list[str]:
        # Declare all parameters with default values. The type is inferred.
        self.declare_parameter('mavlink_config.vehicle_base_port', 14000)
        self.declare_parameter('mavlink_config.stream_for_qgc', True)

        self.declare_parameter('gazebo_models.default_robot_name', 'uav')
        self.declare_parameter('gazebo_models.spacing', 5.0)

        self.declare_parameter('jinja_templates.suffix', '.sdf.jinja')

        self.declare_parameter('firmware_launch_delay', 0.0)

        self.declare_parameter('extra_resource_paths', [""])

        self.declare_parameter('tf_static_publisher.base_frame', "fcu")
        self.declare_parameter('tf_static_publisher.ignored_sensor_frames',
                               ["air_pressure_sensor", "magnetometer_sensor", "navsat_sensor", "imu_sensor"])

        # Get all parameters
        try:
            self._px4_mavlink_config = Px4MavlinkConfig()
            self._px4_mavlink_config.vehicle_base_port = self.get_parameter('mavlink_config.vehicle_base_port').value
            self._px4_mavlink_config.stream_for_qgc = int(self.get_parameter('mavlink_config.stream_for_qgc').value)
            self._px4_mavlink_config.firmware_launch_delay = float(self.get_parameter('firmware_launch_delay').value)

            self.default_robot_name = self.get_parameter('gazebo_models.default_robot_name').value
            self.model_spacing = self.get_parameter('gazebo_models.spacing').value

            self.template_suffix = self.get_parameter('jinja_templates.suffix').value

            self.tf_base_frame = self.get_parameter('tf_static_publisher.base_frame').value
            self.tf_ignored_sensor_frames = self.get_parameter('tf_static_publisher.ignored_sensor_frames').value

        except rclpy.exceptions.ParameterNotDeclaredException as e:
            self.get_logger().error(f'Could not load required param. {e}')
            raise RuntimeError(f'Could not load required param. {e}')

        # Configure resources and Jinja environment
        resource_paths = [os.path.join(get_package_share_directory('mrs_uav_gazebo_simulator'), 'models')]

        try:
            extra_resource_paths = self.get_parameter('extra_resource_paths').value
        except:
            # no extra resources
            extra_resource_paths = []
            pass

        if extra_resource_paths is not None:
            for elem in extra_resource_paths:
                rpath = get_package_share_directory(elem) if not os.path.exists(elem) else elem
                self.get_logger().info(f'Adding extra resources from {rpath}')
                resource_paths.append(rpath)

        return resource_paths

    # #}

    # #{_create_tempfile_folder(self)
    def _create_tempfile_folder(self) -> str:
        time_str = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
        temp_folder = f'mrs_gazebo_simulator_{time_str}'
        tempfile_folder = os.path.join(tempfile.gettempdir(), temp_folder)

        try:
            os.makedirs(tempfile_folder, exist_ok=False)
            return tempfile_folder
        except Exception as e:
            raise RuntimeError(f"Error creating directory {tempfile_folder}: {e}")

    # #}

    # #{ spawn_gazebo_model(self, robot_params)
    def spawn_gazebo_model(self, robot_params):
        name = robot_params['name']
        sdf_content = self._template_manager.render_sdf(robot_params, self.jinja_templates)

        if sdf_content is None:
            self.get_logger().error('Template did not render, spawn failed.')
            return

        self.sdf_to_tf_publisher.generate_sensor_tfs(sdf_content)

        filename = f'mrs_drone_spawner_{name}.sdf'
        filepath = os.path.join(self.tempfile_folder, filename)

        with open(filepath, 'w') as output_file:
            output_file.write(sdf_content)
            self.get_logger().info(f'Model for {name} written to {filepath}')
            robot_params['sdf_filepath'] = filepath

        request = SpawnEntity.Request()
        request.entity_factory.name = name
        request.entity_factory.sdf = sdf_content
        request.entity_factory.pose.position.x = robot_params['spawn_pose']['x']
        request.entity_factory.pose.position.y = robot_params['spawn_pose']['y']
        request.entity_factory.pose.position.z = robot_params['spawn_pose']['z']

        q_w = math.cos(robot_params['spawn_pose']['heading'] / 2.0)
        q_z = math.sin(robot_params['spawn_pose']['heading'] / 2.0)
        request.entity_factory.pose.orientation.w = q_w
        request.entity_factory.pose.orientation.z = q_z

        self.get_logger().info(f'Requesting spawn for model {name}')
        self.gazebo_spawn_future = self.gazebo_spawn_proxy.call_async(request)

        self.gazebo_spawn_future.add_done_callback(
            lambda future: self.service_response_callback_spawn_gazebo_model(future, robot_params))

    # #}

    # #{ service_response_callback_spawn_gazebo_model(self, future, robot_params)
    def service_response_callback_spawn_gazebo_model(self, future, robot_params):
        # This function is called automatically when the service response arrives.
        try:
            response = future.result()
            if not response.success:
                raise CouldNotSpawn(f'Call failed')

            ros_gz_bridge_process = None
            firmware_process = None
            mavros_process = None

            ros_gz_bridge_config, sensor_topics = self.ros_gz_manager.generate_uav_ros_gz_config(robot_params)

            try:
                if ros_gz_bridge_config != "":
                    ros_gz_bridge_process = self.ros_gz_manager.launch_uav_ros_gz_bridge(
                        robot_params['name'], ros_gz_bridge_config, sensor_topics)
                mavros_process = self._px4_mavlink_manager.launch_mavros(robot_params)
                firmware_process = self._px4_mavlink_manager.launch_px4_firmware(self.jinja_templates, robot_params)

            except Exception as e:
                self.get_logger().error(f'Failed during spawn sequence for {robot_params["name"]}: {e}')
                self.delete_gazebo_model(robot_params['name'])
                if firmware_process and firmware_process.is_alive():
                    firmware_process.terminate()
                if mavros_process and mavros_process.is_alive():
                    mavros_process.terminate()
                if ros_gz_bridge_process and ros_gz_bridge_process.is_alive():
                    ros_gz_bridge_process.terminate()
                self.assigned_ids.remove(robot_params['ID'])
                self.gazebo_spawn_future = None
                return

            glob_running_processes.append(firmware_process)
            glob_running_processes.append(mavros_process)
            if ros_gz_bridge_process is not None:
                glob_running_processes.append(ros_gz_bridge_process)

            self.get_logger().info(f'Vehicle {robot_params["name"]} successfully spawned')
            self.active_vehicles.append(robot_params['name'])
            self.gazebo_spawn_future = None

        except Exception as e:
            self.get_logger().error(
                f'Spawning failed for {robot_params["name"]} with error: {e}, aborting launch sequence.')
            self.assigned_ids.remove(robot_params['ID'])
            self.gazebo_spawn_future = None
            return

    # #}

    # #{ delete_gazebo_model(self, name)
    def delete_gazebo_model(self, name):
        self.get_logger().info(f'Requesting delete for model {name}')
        request = DeleteEntity.Request()
        request.entity.name = name

        self.gazebo_delete_future = self.gazebo_delete_proxy.call_async(request)
        self.gazebo_delete_future.add_done_callback(
            lambda future: self.service_response_callback_delete_gazebo_model(future, name))

    # #}

    # #{ service_response_callback_spawn_gazebo_model(self, future, name)
    def service_response_callback_delete_gazebo_model(self, future, name):
        # This function is called automatically when the service response arrives.
        try:
            response = future.result()
            if response is not None and response.success:
                self.get_logger().info(f'Model {name} deleted successfully.')
            else:
                self.get_logger().error(f'Failed to delete model {name}. Error: {future.exception()}')

            self.gazebo_delete_future = None

        except Exception as e:
            self.get_logger().error(f'Failed to delete model {name}. Error: {e}')
            self.gazebo_spawn_future = None

    # #}

    # #{ callback_spawn(self, request, response)
    def callback_spawn(self, request, response):
        if not self.gazebo_spawn_proxy.wait_for_service(timeout_sec=5.0):
            service_name = self.gazebo_spawn_proxy.service_name
            self.get_logger().error(f'Gazebo spawn service "{service_name}" not available.')
            response.success = False
            response.message = f'Gazebo spawn service "{service_name}" not available.'
            return response

        self.spawn_called = True
        self.get_logger().info(f'Spawn called with args "{request.value}"')
        response.success = False

        params_dict = None
        already_assigned_ids = copy.deepcopy(self.assigned_ids)
        try:
            params_dict = self.parse_user_input(request.value)
        except Exception as e:
            self.get_logger().warn(f'While parsing user input: {e}')
            response.message = str(e.args[0])
            self.assigned_ids = already_assigned_ids
            return response

        help_text = self.get_help_text(params_dict)
        if help_text is not None:
            self.get_logger().info(help_text)
            response.message = help_text.replace('\n', ' ').replace('\t', ' ')
            response.success = True
            return response

        if not self.check_user_request(params_dict):
            self.get_logger().warn("User request contains invalid arguments.")
            response.message = ("The request contains invalid arguments. "
                                "Use the --help option to see the supported arguments.")
            response.success = False
            self.assigned_ids = already_assigned_ids
            return response

        self.get_logger().info(f'Spawner params assigned "{params_dict}"')

        self.get_logger().info('Adding vehicles to a spawn queue')
        self.processing = True
        with self.queue_mutex:
            for i, ID in enumerate(params_dict['ids']):
                robot_params = self.get_jinja_params_for_one_robot(params_dict, i, ID)
                self.vehicle_queue.append(robot_params)
                self.sdf_files.append(robot_params)

        response.success = True
        response.message = f'Launch sequence queued for {len(params_dict["ids"])} robots'
        return response

    # #}

    # #{ check_user_request(self, params_dict)
    def check_user_request(self, params_dict) -> bool:
        core_keys = ["help", "model", "ids", "names", "spawn_poses"]
        user_cmds = []
        for key, _ in params_dict.items():
            if key in core_keys:
                continue
            user_cmds.append(key)

        valid_cmds = []
        try:
            template_wrapper = self.jinja_templates[params_dict["model"]]
            for name, component in template_wrapper.components.items():
                valid_cmds.append(component.keyword)
        except ValueError:
            return False

        for cmd in user_cmds:
            if cmd not in valid_cmds:
                return False

        return True

    # #}

    # #{ callback_action_timer(self)
    def callback_action_timer(self):
        # Check for an ongoing request and if it has timed out
        if self.gazebo_spawn_future is not None and not self.gazebo_spawn_future.done(
        ) and self.gazebo_spawn_request_start_time is not None:
            if time.time() - self.gazebo_spawn_request_start_time > 5.0:
                self.get_logger().error('Service call timed out!')
                self.gazebo_spawn_future = None  # Reset state to allow a new request
            else:
                self.get_logger().warn('Previous gazebo_spawn service call is pending. Skipping this cycle.')
            return
        with self.queue_mutex:
            if not self.vehicle_queue:
                self.processing = False
                return
            robot_params = self.vehicle_queue.pop(0)

        self.spawn_gazebo_model(robot_params)

        if len(self.vehicle_queue) == 0:
            self.sdf_to_tf_publisher.publish_sensor_tfs()

    # #}

    # #{ callback_diagnostics_timer(self)
    def callback_diagnostics_timer(self):
        diagnostics = GazeboSpawnerDiagnostics()
        diagnostics.spawn_called = self.spawn_called
        diagnostics.processing = self.processing
        diagnostics.active_vehicles = self.active_vehicles
        self.queue_mutex.acquire()
        diagnostics.queued_vehicles = [params['name'] for params in self.vehicle_queue]
        diagnostics.queued_processes = len(self.vehicle_queue)
        self.queue_mutex.release()
        self.diagnostics_pub.publish(diagnostics)

    # #}

    # --------------------------------------------------------------
    # |                     user input parsing                     |
    # --------------------------------------------------------------

    # #{ parse_user_input(self, input_str)
    def parse_user_input(self, input_str):
        '''
        Extract params from an input string, create spawner args
        expected input:
            device ids (integers separated by spaces)
            keywords (specified in jinja components starting with '--')
            component args following a keyword (values separated by spaces)
        :param input_str: string containing all args in the format specified above
        :return: a dict in format {keyword: component_args}, always contains keys "help", "model", "ids", "names", "spawn_poses"
        NOTE: arguments of a component/keyword will always be parsed as a list/dict, even for a single value

        Raises:
        AssertionError in case of unexpected data in mandatory values under keys "model", "ids", "names", "spawn_poses"
        '''

        input_dict = {'help': False, 'model': None, 'ids': [], 'names': [], 'spawn_poses': {}}

        # parse out the keywords starting with '--'
        pattern = re.compile(r'(--\S*)')
        substrings = [m.strip() for m in re.split(pattern, input_str) if len(m.strip()) > 0]

        if len(substrings) < 1:
            input_dict['help'] = True
            return input_dict

        # before the first keyword, there should only be device IDs
        first_keyword_index = 0
        if '--' not in substrings[0]:
            input_dict['ids'] = self.parse_string_to_objects(substrings[0])
            first_keyword_index = 1
        else:
            input_dict['ids'].append(self.assign_free_id())

        # pair up keywords with args
        for i in range(first_keyword_index, len(substrings)):

            if substrings[i].startswith('--'):
                input_dict[substrings[i][2:]] = None
                continue
            else:
                input_keys = [*input_dict.keys()]
                if len(input_keys) > 1:
                    input_dict[input_keys[-1]] = self.parse_string_to_objects(substrings[i])

        # attempt to match model to available templates
        for k in input_dict.keys():
            if k in self.jinja_templates.keys():
                input_dict['model'] = str(k)
                del input_dict[k]
                break

        valid_ids = []

        for ID in input_dict['ids']:
            if not isinstance(ID, int):
                if ID in self.jinja_templates.keys() and input_dict['model'] is None:
                    self.get_logger().info(f'Using {ID} as model template')
                    input_dict['model'] = ID
                else:
                    self.get_logger().warn(f'Ignored ID {ID}: Not an integer')
                continue
            if ID < 0 or ID > 255:
                self.get_logger().warn(f'Ignored ID {ID}: Must be in range(0, 256)')
                continue
            if ID in self.assigned_ids:
                self.get_logger().warn(f'Ignored ID {ID}: Already assigned')
                continue
            valid_ids.append(ID)

        input_dict['ids'].clear()

        if '--help' in substrings:
            input_dict['help'] = True
            return input_dict

        if len(valid_ids) > 0:
            self.get_logger().info(f'Valid robot IDs: {valid_ids}')
            input_dict['ids'] = valid_ids
            self.assigned_ids.update(input_dict['ids'])
        else:
            raise NoValidIDGiven('No valid ID given. Check your input')

        if 'pos' in input_dict.keys():
            try:
                input_dict['spawn_poses'] = self.get_spawn_poses_from_args(input_dict['pos'], input_dict['ids'])
            except (WrongNumberOfArguments, ValueError) as err:
                self.get_logger().error(f'While parsing args for "--pos": {err}')
                self.get_logger().warn(f'Assigning random spawn poses instead')
                input_dict['spawn_poses'] = self.get_randomized_spawn_poses(input_dict['ids'])
            finally:
                del input_dict['pos']

        elif 'pos-file' in input_dict.keys():
            try:
                input_dict['spawn_poses'] = self.get_spawn_poses_from_file(input_dict['pos-file'][0], input_dict['ids'])
            except (FileNotFoundError, SuffixError, FormattingError, WrongNumberOfArguments, ValueError) as err:
                self.get_logger().error(f'While parsing args for "--pos-file": {err}')
                self.get_logger().warn(f'Assigning random spawn poses instead')
                input_dict['spawn_poses'] = self.get_randomized_spawn_poses(input_dict['ids'])
            finally:
                del input_dict['pos-file']

        else:
            input_dict['spawn_poses'] = self.get_randomized_spawn_poses(input_dict['ids'])

        if 'name' in input_dict.keys():
            for ID in input_dict['ids']:
                input_dict['names'].append(str(input_dict['name'][0]) + str(ID))
            del input_dict['name']
        else:
            for ID in input_dict['ids']:
                input_dict['names'].append(str(self.default_robot_name) + str(ID))

        assert isinstance(input_dict['ids'], list) and len(input_dict['ids']) > 0, 'No vehicle ID assigned'
        assert input_dict['model'] is not None, 'Model not specified'
        assert isinstance(input_dict['names'], list) and len(input_dict['names']) == len(
            input_dict['ids']), f'Invalid vehicle names {input_dict["names"]}'
        assert isinstance(input_dict['spawn_poses'], dict) and len(input_dict['spawn_poses'].keys()) == len(
            input_dict['ids']), f'Invalid spawn poses {input_dict["spawn_poses"]}'

        return input_dict

    # #}

    # #{ parse_string_to_objects(self, input_str)
    def parse_string_to_objects(self, input_str):
        '''
        Attempt to convert input_str into a dictionary or a list
        Convert numerals into number datatypes whenever possible
        Returns None if the input cannot be interpreted as dict or list
        '''
        input_str = input_str.strip()

        params = []
        for s in input_str.split():
            if len(s) > 0:
                try:
                    # try to convert input_str to numbers
                    params.append(ast.literal_eval(s))
                except (SyntaxError, ValueError):
                    # leave non-numbers as string
                    params.append(s)

        params_dict = {}
        if isinstance(params, list):
            # try to convert named args into a dict
            for p in params:
                try:
                    if ':=' in p:
                        kw, arg = p.split(':=')
                        try:
                            # try to convert arg to number
                            params_dict[kw] = ast.literal_eval(arg)
                        except (SyntaxError, ValueError):
                            # leave non-numbers as string
                            params_dict[kw] = arg
                except TypeError:
                    pass

        if len(params_dict.keys()) > 0 and len(params_dict.keys()) == len(params):
            # whole input converted to a dict
            return params_dict
        else:
            return params

        return None

    # #}

    # #{ get_help_text(self, input_dict):
    def get_help_text(self, input_dict):
        '''
        Used to construct the help text (string) for a given dict of input args
        Returns:
            generic spawner help
            or
            help for a specific model
            or
            None (if the input does not contain "help")
        '''
        if not input_dict['help']:
            return None

        if input_dict['model'] is None:
            display_text = self.get_spawner_help_text()
        else:
            display_text = self.get_model_help_text(input_dict['model'])

        return display_text

    # #}

    # #{ get_model_help_text(self, model_name)
    def get_model_help_text(self, model_name):
        '''
        Create a help string by loading all callable components from a given template in the following format
        Component name
            Description:
            Default args:
        '''
        self.get_logger().info(f'Getting help for model {model_name}')
        try:
            template_wrapper = self.jinja_templates[model_name]
            response = f'Components used in template "{template_wrapper.jinja_template.filename}":\n'
        except ValueError:
            return f'Template for model {model_name} not found'

        for name, component in template_wrapper.components.items():
            response += f'{component.keyword}\n\tDescription: {component.description}\n\tDefault args: {component.default_args}\n\n'

        return response

    # #}

    # #{ get_spawner_help_text(self)
    def get_spawner_help_text(self):
        '''Create a generic help string for the spawner basic use'''

        self.get_logger().info(f'Getting generic spawner help')
        response = 'The spawn service expects the following input (as a string):\n'
        response += '\tdevice ids (integers separated by spaces, auto-assigned if no ID is specified),\n'
        response += '\tmodel (use \'--\' with a model name to select a specific model),\n'
        response += '\tkeywords (specified inside jinja macros as "spawner_keyword". Add \'--\' before each keyword when calling spawn),\n'
        response += '\tcomponent args following a keyword (values separated by spaces or a python dict, overrides "spawner_default_args" in jinja macros),\n'
        response += '\n'
        response += '\tModels available: '

        for model_name in sorted(self.jinja_templates.keys()):
            response += f'{model_name}, '

        return response

    # #}

    # --------------------------------------------------------------
    # |                        Spawner utils                       |
    # --------------------------------------------------------------

    # #{ assign_free_id(self)
    def assign_free_id(self):
        '''
        Assign an unused ID in range <0, 255>
        :return: unused ID for a robot (int)
        :raise NoFreeIDAvailable: if max vehicle count has been reached
        '''
        for i in range(0, 256):  # 255 is a hard limit of px4 sitl
            if i not in self.assigned_ids:
                self.get_logger().info(f'Assigned free ID "{i}" to a robot')
                return i
        raise NoFreeIDAvailable('Cannot assign a free ID')

    # #}

    # #{ get_spawn_poses_from_file(self, filename, ids)
    def get_spawn_poses_from_file(self, filename, ids):
        '''
        Parses an input file and extracts spawn poses for vehicles. The file must be either ".csv" or ".yaml"

        CSV files have to include one line per robot, formatting: X, Y, Z, HEADING
        YAML files have to include one block per robot, formatting:
        block_header: # not used
            id: int
            x: float
            y: float
            z: float
            heading: float


        The file must contain spawn poses for all vehicles
        :param fileame: full path to a file
        :param ids: a list of ints containing unique vehicle IDs
        :return: a dict in format {id: {'x': pos_x, 'y', pos_y, 'z': pos_z, 'heading': pos_heading}}

        Raises:
        FileNotFoundError - if filename does not exist
        FormattingError - if the csv or yaml file does not match the expected structure
        SuffixError - filename has other suffix than ".csv" or ".yaml"
        WrongNumberOfArguments - number of poses defined in the file does not match the number of ids
        ValueError - spawn poses are not numbers
        '''

        self.get_logger().info(f'Loading spawn poses from file "{filename}"')
        if not os.path.isfile(filename):
            raise FileNotFoundError(f'File "{filename}" does not exist!')

        spawn_poses = {}

        # #{ csv
        if filename.endswith('.csv'):
            array_string = list(csv.reader(open(filename)))
            for row in array_string:
                if (len(row) != 5):
                    raise FormattingError(
                        f'Incorrect data in file "{filename}"! Data in ".csv" file type should be in format [id, x, y, z, heading] (types: int, float, float, float, float)'
                    )
                if int(row[0]) in ids:
                    spawn_poses[int(row[0])] = {
                        'x': float(row[1]),
                        'y': float(row[2]),
                        'z': float(row[3]),
                        'heading': float(row[4])
                    }
        # #}

        # #{ yaml
        elif filename.endswith('.yaml'):
            dict_vehicle_info = yaml.safe_load(open(filename, 'r'))
            for item, data in dict_vehicle_info.items():
                if (len(data.keys()) != 5):
                    raise FormattingError(
                        f'Incorrect data in file "{filename}"! Data  in ".yaml" file type should be in format \n uav_name: \n\t id: (int) \n\t x: (float) \n\t y: (float) \n\t z: (float) \n\t heading: (float)'
                    )

                if int(data['id']) in ids:
                    spawn_poses[data['id']] = {
                        'x': float(data['x']),
                        'y': float(data['y']),
                        'z': float(data['z']),
                        'heading': float(data['heading'])
                    }
        # #}

        else:
            raise SuffixError(f'Incorrect file type! Suffix must be either ".csv" or ".yaml"')

        if len(spawn_poses.keys()) != len(ids) or set(spawn_poses.keys()) != set(ids):
            raise WrongNumberOfArguments(f'File "{filename}" does not specify poses for all robots!')

        self.get_logger().info(f'Spawn poses returned: {spawn_poses}')
        return spawn_poses

    # #}

    # #{ get_spawn_poses_from_args(self, pos_args, ids)
    def get_spawn_poses_from_args(self, pos_args, ids):
        '''
        Parses the input args extracts spawn poses for vehicles.
        If more vehicles are spawned at the same time, the given pose is used for the first vehicle.
        Additional vehicles are spawned with an offset of {config param: gazebo_models/spacing} meters in X

        :param pos_args: a list of 4 numbers [x,y,z,heading]
        :param ids: a list of ints containing unique vehicle IDs
        :return: a dict in format {id: {'x': pos_x, 'y', pos_y, 'z': pos_z, 'heading': pos_heading}}

        Raises:
        WrongNumberOfArguments - pos_args does not contain exactly 4 values
        ValueError - input cannot be converted into numbers
        '''
        spawn_poses = {}
        if len(pos_args) != 4:
            raise WrongNumberOfArguments(f'Expected exactly 4 args after keyword "--pos", got {len(pos_args)}')

        x = float(pos_args[0])
        y = float(pos_args[1])
        z = float(pos_args[2])
        heading = float(pos_args[3])

        spawn_poses[ids[0]] = {'x': x, 'y': y, 'z': z, 'heading': heading}

        if len(ids) > 1:
            self.get_logger().warn(
                f'Spawning more than one vehicle with "--pos". Each additional vehicle will be offset by {self.model_spacing} meters in X'
            )
            for i in range(len(ids)):
                x += self.model_spacing
                spawn_poses[ids[i]] = {'x': x, 'y': y, 'z': z, 'heading': heading}

        self.get_logger().info(f'Spawn poses returned: {spawn_poses}')
        return spawn_poses

    # #}

    # #{ get_randomized_spawn_poses(self, ids)
    def get_randomized_spawn_poses(self, ids):
        '''
        Creates randomized spawn poses for all vehicles.
        The poses are generated with spacing defined by config param: gazebo_models/spacing
        Height is always set to 0.3

        :param ids: a list of ints containing unique vehicle IDs
        :return: a dict in format {id: {'x': pos_x, 'y', pos_y, 'z': pos_z, 'heading': pos_heading}}
        '''
        spawn_poses = {}

        circle_diameter = 0.0
        total_positions_in_current_circle = 0
        angle_increment = 0
        remaining_positions_in_current_circle = 1
        circle_perimeter = math.pi * circle_diameter
        random_angle_offset = 0
        random_x_offset = round(random.uniform(-self.model_spacing, self.model_spacing), 2)
        random_y_offset = round(random.uniform(-self.model_spacing, self.model_spacing), 2)

        for ID in ids:
            if remaining_positions_in_current_circle == 0:
                circle_diameter = circle_diameter + self.model_spacing
                circle_perimeter = math.pi * circle_diameter
                total_positions_in_current_circle = math.floor(circle_perimeter / self.model_spacing)
                remaining_positions_in_current_circle = total_positions_in_current_circle
                angle_increment = (math.pi * 2) / total_positions_in_current_circle
                random_angle_offset = round(random.uniform(-math.pi, math.pi), 2)

            x = round(
                math.sin(angle_increment * remaining_positions_in_current_circle + random_angle_offset) *
                circle_diameter, 2) + random_x_offset
            y = round(
                math.cos(angle_increment * remaining_positions_in_current_circle + random_angle_offset) *
                circle_diameter, 2) + random_y_offset
            z = 0.3
            heading = round(random.uniform(-math.pi, math.pi), 2)
            remaining_positions_in_current_circle = remaining_positions_in_current_circle - 1
            spawn_poses[ID] = {'x': x, 'y': y, 'z': z, 'heading': heading}

        self.get_logger().info(f'Spawn poses returned: {spawn_poses}')
        return spawn_poses

    # #}

    # #{ get_jinja_params_for_one_robot(self, params_dict, index, ID)
    def get_jinja_params_for_one_robot(self, params_dict, index, ID):
        '''Makes a deep copy of params dict, removes entries of other robots, assigns mavlink ports
        :param index: index of the robot in the input sequence
        :param ID: ID of the robot, should match the value in params_dict['ids'][index]
        :return: a dict of params to be used in rendering the jinja template
        '''

        robot_params = copy.deepcopy(params_dict)
        robot_params['ID'] = ID
        robot_params['name'] = params_dict['names'][index]
        robot_params['spawn_pose'] = params_dict['spawn_poses'][ID]

        del robot_params['names']
        del robot_params['help']
        del robot_params['ids']
        del robot_params['spawn_poses']

        robot_params['mavlink_config'] = self._px4_mavlink_manager.get_mavlink_config_for_robot(ID)
        robot_params['mavros_px4_config'] = self._px4_mavlink_manager.generate_mavros_px4_config(robot_params['name'])

        return robot_params

    # #}


def main(args=None):
    rclpy.init(args=args)
    atexit.register(exit_handler)
    spawner_node = MrsDroneSpawner()
    try:
        rclpy.spin(spawner_node)
    except KeyboardInterrupt:
        pass
    finally:
        spawner_node.destroy_node()


if __name__ == '__main__':
    main()
