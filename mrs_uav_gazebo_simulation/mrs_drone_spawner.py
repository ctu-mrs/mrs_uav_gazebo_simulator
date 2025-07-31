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
import rclpy
from rclpy.node import Node
import rclpy.exceptions
import multiprocessing
import xml.dom.minidom

from ament_index_python.packages import get_package_share_directory
from mrs_uav_gazebo_simulation.utils.component_wrapper import ComponentWrapper
from mrs_uav_gazebo_simulation.utils.template_wrapper import TemplateWrapper

# ROS 2 Imports
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ros_gz_interfaces.srv import SpawnEntity, DeleteEntity
from mrs_msgs.srv import String as StringSrv
from mrs_msgs.msg import GazeboSpawnerDiagnostics

glob_running_processes = []

# #{ Exceptions and Errors
#  exceptions that can be raised by the spawner

class NoFreeIDAvailable(RuntimeError):
    """Raised when an ID could not be automatically assigned."""
    pass

class NoValidIDGiven(RuntimeError):
    """Raised when a user-provided ID is already in use."""
    pass

class CouldNotLaunch(RuntimeError):
    """Raised when a subprocess (like PX4 or MAVROS) fails to start."""
    pass

class FormattingError(ValueError):
    """Raised when spawn arguments are in an unrecognizable format."""
    pass

class WrongNumberOfArguments(ValueError):
    """Raised when a command-line flag is missing its required value."""
    pass

class SuffixError(NameError):
    """Raised for issues related to file suffixes, like for Jinja templates."""
    pass

# #} end of Exceptions and Errors

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

        # Declare all parameters with default values. The type is inferred.
        self.declare_parameter('mavlink_config.vehicle_base_port', 14000)
        self.declare_parameter('mavlink_config.mavlink_tcp_base_port', 4560)
        self.declare_parameter('mavlink_config.mavlink_udp_base_port', 14560)
        self.declare_parameter('mavlink_config.mavlink_gcs_udp_base_port_local', 18000)
        self.declare_parameter('mavlink_config.mavlink_gcs_udp_base_port_remote', 18100)
        self.declare_parameter('mavlink_config.qgc_udp_port', 14550)
        self.declare_parameter('mavlink_config.sdk_udp_port', 14540)
        self.declare_parameter('mavlink_config.send_vision_estimation', False)
        self.declare_parameter('mavlink_config.send_odometry', True)
        self.declare_parameter('mavlink_config.enable_lockstep', True)
        self.declare_parameter('mavlink_config.use_tcp', True)

        self.declare_parameter('gazebo_models.default_robot_name', 'uav')
        self.declare_parameter('gazebo_models.spacing', 5.0)

        self.declare_parameter('jinja_templates.suffix', '.sdf.jinja')
        self.declare_parameter('jinja_templates.save_rendered_sdf', True)

        self.declare_parameter('extra_resource_paths', [])
        self.declare_parameter('world_name', "default")

        # Get all parameters
        try:
            self.vehicle_base_port = self.get_parameter('mavlink_config.vehicle_base_port').value
            self.mavlink_tcp_base_port = self.get_parameter('mavlink_config.mavlink_tcp_base_port').value
            self.mavlink_udp_base_port = self.get_parameter('mavlink_config.mavlink_udp_base_port').value
            self.mavlink_gcs_udp_base_port_local = self.get_parameter('mavlink_config.mavlink_gcs_udp_base_port_local').value
            self.mavlink_gcs_udp_base_port_remote = self.get_parameter('mavlink_config.mavlink_gcs_udp_base_port_remote').value
            self.qgc_udp_port = self.get_parameter('mavlink_config.qgc_udp_port').value
            self.sdk_udp_port = self.get_parameter('mavlink_config.sdk_udp_port').value
            self.send_vision_estimation = self.get_parameter('mavlink_config.send_vision_estimation').value
            self.send_odometry = self.get_parameter('mavlink_config.send_odometry').value
            self.enable_lockstep = self.get_parameter('mavlink_config.enable_lockstep').value
            self.use_tcp = self.get_parameter('mavlink_config.use_tcp').value

            self.default_robot_name = self.get_parameter('gazebo_models.default_robot_name').value
            self.model_spacing = self.get_parameter('gazebo_models.spacing').value

            self.template_suffix = self.get_parameter('jinja_templates.suffix').value
            self.save_sdf_files = self.get_parameter('jinja_templates.save_rendered_sdf').value
            self.world_name = self.get_parameter('world_name').value

        except rclpy.exceptions.ParameterNotDeclaredException as e:
            self.get_logger().error(f'Could not load required param. {e}')
            raise RuntimeError(f'Could not load required param. {e}')

        # Configure resources and Jinja environment
        resource_paths = [os.path.join(get_package_share_directory('mrs_uav_gazebo_simulation'), 'models')]
        
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

        self.jinja_env = self.configure_jinja2_environment(resource_paths)

        # Find launch files
        gazebo_simulation_path = get_package_share_directory('mrs_uav_gazebo_simulation')
        px4_api_path = get_package_share_directory('mrs_uav_px4_api')
        self.mavros_launch_path = os.path.join(px4_api_path, 'launch', 'mavros_gazebo_simulation.py')
        self.px4_fimrware_launch_path = os.path.join(gazebo_simulation_path, 'launch', 'run_simulation_firmware.launch.py')

        try:
            self.jinja_templates = self.build_template_database()
        except RecursionError as err:
            self.get_logger().error(f'{err}')
            raise RuntimeError(f'{err}')

        self.get_logger().info('Jinja templates loaded.')
        
        # Setup ROS 2 communications
        self.spawn_server = self.create_service(StringSrv, 'spawn', self.callback_spawn)
        self.diagnostics_pub = self.create_publisher(GazeboSpawnerDiagnostics, 'diagnostics', 1)
        self.diagnostics_timer = self.create_timer(0.1, self.callback_diagnostics_timer)
        self.action_timer = self.create_timer(0.1, self.callback_action_timer)

        self.gazebo_spawn_proxy = self.create_client(SpawnEntity, f'/world/{self.world_name}/create')
        self.gazebo_delete_proxy = self.create_client(DeleteEntity, f'/world/{self.world_name}/remove')

        # Setup system variables
        self.spawn_called = False
        self.processing = False
        self.vehicle_queue = []
        self.queue_mutex = multiprocessing.Lock()
        self.active_vehicles = []
        self.assigned_ids = set()

        self.is_initialized = True
        self.get_logger().info('Initialized')

    # #{ launch_px4_firmware 
    def launch_px4_firmware(self, robot_params):
        name = robot_params['name']
        self.get_logger().info(f'Launching PX4 firmware for {name}')

        package_name = self.jinja_templates[robot_params['model']].package_name
        package_path = get_package_share_directory(package_name)
        romfs_path = os.path.join(str(package_path), 'ROMFS')
        if not os.path.exists(romfs_path) or not os.path.isdir(romfs_path):
            self.get_logger().error(f'Could not start PX4 firmware for {name}. ROMFS folder not found')
            raise CouldNotLaunch('ROMFS folder not found')

        launch_arguments = {
            'ID': str(robot_params['ID']),
            'PX4_SIM_MODEL': str(robot_params['model']),
            'ROMFS_PATH': str(romfs_path)
        }

        if 'mavlink_gcs_udp_port_local' in robot_params and 'mavlink_gcs_udp_port_remote' in robot_params:
            launch_arguments['MAVLINK_GCS_UDP_PORT_LOCAL'] = str(robot_params['mavlink_gcs_udp_port_local'])
            launch_arguments['MAVLINK_GCS_UDP_PORT_REMOTE'] = str(robot_params['mavlink_gcs_udp_port_remote'])

        ld = LaunchDescription([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(self.px4_fimrware_launch_path),
                launch_arguments=launch_arguments.items()
            )
        ])

        launch_service = LaunchService(debug=False)
        launch_service.include_launch_description(ld)
        firmware_process = multiprocessing.Process(target=launch_service.run)

        try:
            firmware_process.start()
        except Exception as e:
            self.get_logger().error(f'Could not start PX4 firmware for {name}. Node failed to launch: {e}')
            raise CouldNotLaunch('PX4 failed to launch')

        self.get_logger().info(f'PX4 firmware for {name} launched')
        return firmware_process
    # #}

    # #{ launch_mavros
    def launch_mavros(self, robot_params):
        name = robot_params['name']
        self.get_logger().info(f'Launching mavros for {name}')

        ld = LaunchDescription([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(self.mavros_launch_path),
                launch_arguments={
                    'ID': str(robot_params['ID']),
                    'fcu_url': str(robot_params['mavlink_config']['fcu_url']),
                    'vehicle': str(robot_params['model'])
                }.items(),
            )
        ])

        launch_service = LaunchService(debug=False)
        launch_service.include_launch_description(ld)
        mavros_process = multiprocessing.Process(target=launch_service.run)

        try:
            mavros_process.start()
        except Exception as e:
            self.get_logger().error(f'Could not start mavros for {name}. Node failed to launch: {e}')
            raise CouldNotLaunch('Mavros failed to launch')

        self.get_logger().info(f'Mavros for {name} launched')
        return mavros_process
    # #}

    # #{ spawn_gazebo_model 
    def spawn_gazebo_model(self, robot_params):
        name = robot_params['name']
        sdf_content = self.render(robot_params)

        if sdf_content is None:
            self.get_logger().error('Template did not render, spawn failed.')
            return False

        if self.save_sdf_files:
            time_str = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
            
            filename = f"mrs_drone_spawner_{name}_{time_str}.sdf"
            filepath = os.path.join('/tmp', filename)
            
            with open(filepath, 'w') as output_file:
                output_file.write(sdf_content)
                self.get_logger().info(f'Model for {name} written to {filepath}')

        request = SpawnEntity.Request()
        request.entity_factory.name = name
        request.entity_factory.sdf = sdf_content
        request.initial_pose.position.x = robot_params['spawn_pose']['x']
        request.initial_pose.position.y = robot_params['spawn_pose']['y']
        request.initial_pose.position.z = robot_params['spawn_pose']['z']
        
        q_w = math.cos(robot_params['spawn_pose']['heading'] / 2.0)
        q_z = math.sin(robot_params['spawn_pose']['heading'] / 2.0)
        request.initial_pose.orientation.w = q_w
        request.initial_pose.orientation.z = q_z

        self.get_logger().info(f'Requesting spawn for model {name}')
        future = self.gazebo_spawn_proxy.call_async(request)
        
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is not None and future.result().success:
            self.get_logger().info(f'Model {name} spawned successfully.')
            return True
        else:
            self.get_logger().error(f'Failed to spawn model {name}. Error: {future.exception()}')
            return False
    # #}

    # #{ delete_gazebo_model
    def delete_gazebo_model(self, name):
        self.get_logger().info(f'Requesting delete for model {name}')
        request = DeleteEntity.Request()
        request.entity.name = name
        request.entity.type = request.entity.MODEL

        future = self.gazebo_delete_proxy.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None and future.result().success:
            self.get_logger().info(f'Model {name} deleted successfully.')
        else:
            self.get_logger().error(f'Failed to delete model {name}. Error: {future.exception()}')
    # #}
    
    # #{ callback_spawn
    def callback_spawn(self, req, res):
        if not self.gazebo_spawn_proxy.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Gazebo spawn service not available.')
            res.success = False
            res.message = 'Gazebo spawn service not available.'
            return res

        self.spawn_called = True
        self.get_logger().info(f'Spawn called with args "{req.data}"')
        res.success = False

        params_dict = None
        already_assigned_ids = copy.deepcopy(self.assigned_ids)
        try:
            params_dict = self.parse_user_input(req.data)
        except Exception as e:
            self.get_logger().warn(f'While parsing user input: {e}')
            res.message = str(e.args[0])
            self.assigned_ids = already_assigned_ids
            return res

        help_text = self.get_help_text(params_dict)
        if help_text is not None:
            self.get_logger().info(help_text)
            res.message = help_text.replace('\n', ' ').replace('\t', ' ')
            res.success = True
            return res

        self.get_logger().info(f'Spawner params assigned "{params_dict}"')

        self.get_logger().info('Adding vehicles to a spawn queue')
        self.processing = True
        with self.queue_mutex:
            for i, ID in enumerate(params_dict['ids']):
                robot_params = self.get_jinja_params_for_one_robot(params_dict, i, ID)
                self.vehicle_queue.append(robot_params)

        res.success = True
        res.message = f'Launch sequence queued for {len(params_dict["ids"])} robots'
        return res
    # #}

    # #{ callback_action_timer
    def callback_action_timer(self):
        with self.queue_mutex:
            if not self.vehicle_queue:
                self.processing = False
                return
            robot_params = self.vehicle_queue.pop(0)

        model_spawned = False
        firmware_process = None
        mavros_process = None

        try:
            model_spawned = self.spawn_gazebo_model(robot_params)

            if not model_spawned:
                self.get_logger().error(f"Spawning failed for {robot_params['name']}, aborting launch sequence.")
                self.assigned_ids.remove(robot_params['ID'])
                return

            firmware_process = self.launch_px4_firmware(robot_params)
            mavros_process = self.launch_mavros(robot_params)

        except Exception as e:
            self.get_logger().error(f"Failed during spawn sequence for {robot_params['name']}: {e}")
            if model_spawned:
                self.delete_gazebo_model(robot_params['name'])
            if firmware_process and firmware_process.is_alive():
                firmware_process.terminate()
            if mavros_process and mavros_process.is_alive():
                mavros_process.terminate()
            self.assigned_ids.remove(robot_params['ID'])
            return

        glob_running_processes.append(firmware_process)
        glob_running_processes.append(mavros_process)

        self.get_logger().info(f'Vehicle {robot_params["name"]} successfully spawned')
        self.active_vehicles.append(robot_params['name'])
    # #}

    # #{ callback_diagnostics_timer
    def callback_diagnostics_timer(self):
        msg = GazeboSpawnerDiagnostics()
        msg.spawn_called = self.spawn_called
        msg.processing = self.processing
        msg.queued_vehicles= self.vehicle_queue
        msg.active_vehicles = self.active_vehicles

        self.diagnostics_pub.publish(msg)
    # #}

    # -----------------------------------------------------------------------------------
    # | --------------------------- All Helper Functions -------------------------------- |
    # -----------------------------------------------------------------------------------

    # #{ get_ros_package_name
    def get_ros_package_name(self, model_path):
        # This function is ported to use ROS 2's ament_index_python
        # It finds the package a file belongs to by looking for a package.xml file
        # in parent directories.
        current_path = os.path.dirname(model_path)
        while current_path != os.path.dirname(current_path): # Stop at root
            if 'package.xml' in os.listdir(current_path):
                with open(os.path.join(current_path, 'package.xml'), 'r') as f:
                    doc = xml.dom.minidom.parse(f)
                    package_name = doc.getElementsByTagName('name')[0].firstChild.nodeValue
                    return package_name
            current_path = os.path.dirname(current_path)
        return None
    # #}

    # #{ get_all_templates
    def get_all_templates(self, path):
        # returns all .sdf.jinja templates in a folder
        templates = []
        for f in os.listdir(path):
            if f.endswith(self.template_suffix):
                templates.append(TemplateWrapper(os.path.join(path, f), self.get_ros_package_name(os.path.join(path, f)), self))
        return templates
    # #}

    # #{ get_template_imports
    def get_template_imports(self, template, templates):
        """
        Recursively returns all templates imported by a template
        """
        # remove suffix to get model name
        my_name = os.path.basename(template.path).replace(self.template_suffix, '')
        components = self.get_spawner_components_from_template(template)
        imports = []
        if 'imports' in components:
            for imp in components['imports']:
                if imp in [os.path.basename(t.path).replace(self.template_suffix, '') for t in templates]:
                    # get the template object from the list of all templates
                    imported_template = [t for t in templates if os.path.basename(t.path).replace(self.template_suffix, '') == imp][0]
                    # recursively get all imports from the imported template
                    imports.extend(self.get_template_imports(imported_template, templates))
                    imports.append(imp)
                else:
                    self.get_logger().warn(f'Could not find imported model "{imp}" in template "{my_name}"')
        return imports
    # #}

    # #{ get_spawner_components_from_template
    def get_spawner_components_from_template(self, template):
        """
        Returns a dictionary of all spawner components from a template
        """
        # load jinja file
        with open(template.path, 'r') as f:
            template_string = f.read()

        # find all spawner comments
        spawner_components_re = re.findall(r'\{\#\#(.+?)\#\#\}', template_string, re.DOTALL)
        spawner_components = {}

        # parse all spawner comments to dictionary
        for component in spawner_components_re:
            try:
                # remove all whitespace
                component = ''.join(component.split())
                # parse to dictionary
                spawner_components.update(ast.literal_eval(component))
            except:
                self.get_logger().warn(f'Could not parse spawner component "{component}" in template "{template.path}"')

        return spawner_components
    # #}

    # #{ get_accessible_components
    def get_accessible_components(self, path):
        # check if component file is a valid csv file
        components = []
        try:
            with open(path, 'r') as f:
                reader = csv.reader(f)
                for row in reader:
                    components.append(ComponentWrapper(row[0], row[1], row[2], self))
            return components
        except:
            return None
    # #}

    # #{ get_callable_components
    def get_callable_components(self, components):
        # check if component file is a valid csv file
        callable_comps = []
        for comp in components:
            if comp.callable:
                callable_comps.append(comp)

        return callable_comps
    # #}

    # #{ build_template_database
    def build_template_database(self):
        # find all templates in all resource paths
        templates = []
        for path in self.jinja_env.loader.searchpath:
            templates.extend(self.get_all_templates(path))

        # remove duplicates
        templates = list(dict.fromkeys(templates))

        # build template database
        template_database = {}
        for template in templates:
            # remove suffix to get model name
            my_name = os.path.basename(template.path).replace(self.template_suffix, '')
            template_database[my_name] = template

        # check for circular dependencies
        for name, template in template_database.items():
            imports = self.get_template_imports(template, templates)
            if name in imports:
                raise RecursionError(f'Circular dependency detected in template "{name}"')

        return template_database
    # #}

    # #{ configure_jinja2_environment
    def configure_jinja2_environment(self, resource_paths):
        # create jinja2 environment
        return jinja2.Environment(
            loader=jinja2.FileSystemLoader(resource_paths),
            autoescape=jinja2.select_autoescape(['html', 'xml'])
        )
    # #}

    # #{ render
    def render(self, robot_params):
        try:
            template = self.jinja_templates[robot_params['model']]
            return template.render(robot_params)
        except jinja2.exceptions.TemplateNotFound:
            self.get_logger().error(f'Could not find template for model "{robot_params["model"]}"')
            return None
        except Exception as e:
            self.get_logger().error(f'Error rendering template for model "{robot_params["model"]}": {e}')
            return None
    # #}

    # #{ parse_user_input
    def parse_user_input(self, data):
        data = self.parse_string_to_objects(data)
        return self.get_spawn_poses_from_args(data)
    # #}

    # #{ parse_string_to_objects
    def parse_string_to_objects(self, data):
        # parse the string to a list of objects
        data = data.split(' ')
        # remove empty strings
        data = [x for x in data if x]
        return data
    # #}

    # #{ get_help_text
    def get_help_text(self, params):
        if 'help' in params:
            if params['help'] is None:
                return self.get_spawner_help_text(params)
            else:
                return self.get_model_help_text(params['help'], self.jinja_templates)
        return None
    # #}

    # #{ get_model_help_text
    def get_model_help_text(self, model_name, templates):
        if model_name not in templates:
            return f'Model "{model_name}" not found'

        template = templates[model_name]
        return template.get_help_text()
    # #}

    # #{ get_spawner_help_text
    def get_spawner_help_text(self, spawner_params):
        # TODO this
        return 'TODO'
    # #}

    # #{ get_jinja_params_for_one_robot
    def get_jinja_params_for_one_robot(self, spawner_params, it, ID):
        params = copy.deepcopy(spawner_params)
        params.update(self.get_mavlink_config_for_robot(it, ID))
        params['ID'] = ID
        return params
    # #}

    # #{ get_mavlink_config_for_robot
    def get_mavlink_config_for_robot(self, it, ID):
        mavlink_config = {}
        mavlink_config['mavlink_config'] = {}
        mavlink_config['mavlink_config']['vehicle_port'] = self.vehicle_base_port + it
        mavlink_config['mavlink_config']['qgc_udp_port'] = self.qgc_udp_port + it
        mavlink_config['mavlink_config']['sdk_udp_port'] = self.sdk_udp_port + it
        mavlink_config['mavlink_config']['mavlink_tcp_port'] = self.mavlink_tcp_base_port + it
        mavlink_config['mavlink_config']['mavlink_udp_port'] = self.mavlink_udp_base_port + it
        mavlink_config['mavlink_config']['fcu_url'] = f'udp://127.0.0.1:{self.vehicle_base_port + it}@127.0.0.1:{self.mavlink_udp_base_port + it}'
        mavlink_config['mavlink_config']['gcs_url'] = f'udp://@127.0.0.1:{self.mavlink_gcs_udp_base_port_local + it}'
        mavlink_config['mavlink_config']['send_vision_estimation'] = self.send_vision_estimation
        mavlink_config['mavlink_config']['send_odometry'] = self.send_odometry
        mavlink_config['mavlink_config']['enable_lockstep'] = self.enable_lockstep
        mavlink_config['mavlink_config']['use_tcp'] = self.use_tcp
        return mavlink_config
    # #}

    # #{ assign_free_id
    def assign_free_id(self, uav_id):

        if uav_id is None:
            # find the first free ID
            for i in range(1, 100):
                if i not in self.assigned_ids:
                    self.assigned_ids.add(i)
                    return i
            raise NoFreeIDAvailable('Could not find a free ID')
        else:
            if uav_id in self.assigned_ids:
                raise NoValidIDGiven(f'ID "{uav_id}" is already assigned')
            else:
                self.assigned_ids.add(uav_id)
                return uav_id
    # #}

    # #{ get_spawn_poses_from_file
    def get_spawn_poses_from_file(self, filename):
        vehicles = {}
        with open(filename, 'r') as f:
            for line in f:
                line = line.strip()
                if line.startswith('#'):
                    continue
                if len(line) == 0:
                    continue
                parts = line.split(',')
                uav_id = self.assign_free_id(int(parts[0]))
                vehicles[uav_id] = {
                    'x': float(parts[1]),
                    'y': float(parts[2]),
                    'z': float(parts[3]),
                    'heading': float(parts[4])
                }
        return vehicles
    # #}

    # #{ get_spawn_poses_from_args
    def get_spawn_poses_from_args(self, args):
        vehicles = {}
        vehicles['ids'] = []
        vehicles['names'] = []
        vehicles['spawn_poses'] = []
        vehicles['models'] = []
        vehicles['mavlink_gcs_udp_port_locals'] = []
        vehicles['mavlink_gcs_udp_port_remotes'] = []
        vehicles['help'] = None

        it = 0
        while it < len(args):
            if args[it].lower() == 'help':
                if it + 1 < len(args) and not args[it+1].startswith('--'):
                    vehicles['help'] = args[it+1]
                else:
                    vehicles['help'] = None
                return vehicles
            elif args[it].lower() == '--file':
                if it + 1 < len(args):
                    vehicles['spawn_poses'] = self.get_spawn_poses_from_file(args[it+1])
                    it += 1
                else:
                    raise WrongNumberOfArguments('Missing filename after --file')
            elif args[it].lower() == '--id':
                if it + 1 < len(args):
                    vehicles['ids'].append(self.assign_free_id(int(args[it+1])))
                    it += 1
                else:
                    raise WrongNumberOfArguments('Missing ID after --ID')
            elif args[it].lower() == '--name':
                if it + 1 < len(args):
                    vehicles['names'].append(args[it+1])
                    it += 1
                else:
                    raise WrongNumberOfArguments('Missing name after --name')
            elif args[it].lower() == '--model':
                if it + 1 < len(args):
                    vehicles['models'].append(args[it+1])
                    it += 1
                else:
                    raise WrongNumberOfArguments('Missing model after --model')
            elif args[it].lower() == '--mavlink_gcs_udp_ports':
                if it + 2 < len(args):
                    vehicles['mavlink_gcs_udp_port_locals'].append(int(args[it+1]))
                    vehicles['mavlink_gcs_udp_port_remotes'].append(int(args[it+2]))
                    it += 2
                else:
                    raise WrongNumberOfArguments('Missing local and remote port after --mavlink_gcs_udp_ports')
            else:
                # spawn pose
                try:
                    vehicles['spawn_poses'].append({
                        'x': float(args[it]),
                        'y': float(args[it+1]),
                        'z': float(args[it+2]),
                        'heading': float(args[it+3])
                    })
                    it += 3
                except:
                    raise FormattingError(f'Could not parse spawn pose from "{args[it:]}"')

            it += 1

        # fill in the missing ids
        if len(vehicles['ids']) < len(vehicles['spawn_poses']):
            for i in range(len(vehicles['spawn_poses']) - len(vehicles['ids'])):
                vehicles['ids'].append(self.assign_free_id(None))

        # fill in the missing names
        if len(vehicles['names']) < len(vehicles['ids']):
            for i in range(len(vehicles['ids']) - len(vehicles['names'])):
                vehicles['names'].append(f'{self.default_robot_name}{vehicles["ids"][i]}')

        # fill in the missing models
        if len(vehicles['models']) < len(vehicles['ids']):
            # get default model
            default_model_name = os.path.basename(list(self.jinja_templates.keys())[0])
            for i in range(len(vehicles['ids']) - len(vehicles['models'])):
                vehicles['models'].append(default_model_name)

        if len(vehicles['spawn_poses']) == 0 and len(vehicles['ids']) > 0:
            vehicles['spawn_poses'] = self.get_randomized_spawn_poses(len(vehicles['ids']))

        return vehicles
    # #}

    # #{ get_randomized_spawn_poses
    def get_randomized_spawn_poses(self, num_of_vehicles):
        poses = []
        for i in range(num_of_vehicles):
            poses.append({
                'x': random.uniform(-self.model_spacing, self.model_spacing),
                'y': random.uniform(-self.model_spacing, self.model_spacing),
                'z': random.uniform(0.1, 0.2),
                'heading': random.uniform(-math.pi, math.pi)
            })
        return poses
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
