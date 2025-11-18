#!/usr/bin/python3
import atexit
import copy
import datetime
import math
import os
import rclpy
from rclpy.node import Node
import rclpy.exceptions
import multiprocessing
import time
import tempfile

# ROS 2 Imports
from ament_index_python.packages import get_package_share_directory
from ros_gz_interfaces.srv import SpawnEntity, DeleteEntity

from mrs_uav_gazebo_simulator.utils.spawner_types import Px4MavlinkConfig
from mrs_uav_gazebo_simulator.utils.spawner_exceptions import *
from mrs_uav_gazebo_simulator.core.jinja_template_manager import JinjaTemplateManager
from mrs_uav_gazebo_simulator.core.ros_gz_bridge_manager import RosGzBridgeManager
from mrs_uav_gazebo_simulator.core.px4_mavlink_manager import Px4MavlinkManager
from mrs_uav_gazebo_simulator.core.user_input_manager import UserInputManager
from mrs_uav_gazebo_simulator.core.sdf_to_tf_publisher import SdfTfPublisherSingleton
from mrs_msgs.srv import String as StringSrv
from mrs_msgs.msg import GazeboSpawnerDiagnostics

glob_running_processes = []


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

        self._template_manager = JinjaTemplateManager(ros_node=self,
                                                      resource_paths=resource_paths,
                                                      template_suffix=self._template_suffix)
        self._jinja_templates = self._template_manager.get_jinja_templates()
        self._tempfile_folder = self._create_tempfile_folder()

        gazebo_simulator_path = get_package_share_directory('mrs_uav_gazebo_simulator')
        self._ros_gz_manager = RosGzBridgeManager(ros_node=self,
                                                  gazebo_simulator_path=gazebo_simulator_path,
                                                  tempfile_folder=self._tempfile_folder)
        self._px4_mavlink_manager = Px4MavlinkManager(ros_node=self,
                                                      gazebo_simulator_path=gazebo_simulator_path,
                                                      px4_mavlink_config=self._px4_mavlink_config,
                                                      tempfile_folder=self._tempfile_folder,
                                                      jinja_templates=self._jinja_templates)

        self._user_input_manager = UserInputManager(ros_node=self,
                                                    jinja_templates=self._jinja_templates,
                                                    model_spacing=self._model_spacing,
                                                    default_robot_name=self._default_robot_name)

        # Setup ROS 2 communications
        self._spawn_server = self.create_service(StringSrv, 'spawn', self.callback_spawn)
        self._diagnostics_pub = self.create_publisher(GazeboSpawnerDiagnostics, 'diagnostics', 1)
        self._diagnostics_timer = self.create_timer(0.1, self.callback_diagnostics_timer)
        self._action_timer = self.create_timer(0.1, self.callback_action_timer)

        self._gazebo_spawn_proxy = self.create_client(SpawnEntity, 'create_entity')
        self._gazebo_delete_proxy = self.create_client(DeleteEntity, 'delete_entity')

        # Setup system variables
        self._spawn_called = False
        self._processing = False
        self._vehicle_queue = []
        self._queue_mutex = multiprocessing.Lock()
        self._active_vehicles = []
        self._gazebo_spawn_future = None
        self._gazebo_delete_future = None
        self._gazebo_spawn_request_start_time = None

        # SdfToTf Publisher
        self._sdf_to_tf_publisher = SdfTfPublisherSingleton(ros_node=self,
                                                            base_frame=self._tf_base_frame,
                                                            ignored_sensors=self._tf_ignored_sensor_frames)

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

            self._default_robot_name = self.get_parameter('gazebo_models.default_robot_name').value
            self._model_spacing = self.get_parameter('gazebo_models.spacing').value

            self._template_suffix = self.get_parameter('jinja_templates.suffix').value

            self._tf_base_frame = self.get_parameter('tf_static_publisher.base_frame').value
            self._tf_ignored_sensor_frames = self.get_parameter('tf_static_publisher.ignored_sensor_frames').value

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
        sdf_content = self._template_manager.render_sdf(robot_params)

        if sdf_content is None:
            self.get_logger().error('Template did not render, spawn failed.')
            return

        self._sdf_to_tf_publisher.generate_sensor_tfs(sdf_content)

        filename = f'mrs_drone_spawner_{name}.sdf'
        filepath = os.path.join(self._tempfile_folder, filename)

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
        self._gazebo_spawn_future = self._gazebo_spawn_proxy.call_async(request)

        self._gazebo_spawn_future.add_done_callback(
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

            ros_gz_bridge_config, sensor_topics = self._ros_gz_manager.generate_uav_ros_gz_config(robot_params)

            try:
                if ros_gz_bridge_config != "":
                    ros_gz_bridge_process = self._ros_gz_manager.launch_uav_ros_gz_bridge(
                        robot_params['name'], ros_gz_bridge_config, sensor_topics)
                mavros_process = self._px4_mavlink_manager.launch_mavros(robot_params)
                firmware_process = self._px4_mavlink_manager.launch_px4_firmware(robot_params)

            except Exception as e:
                self.get_logger().error(f'Failed during spawn sequence for {robot_params["name"]}: {e}')
                self.delete_gazebo_model(robot_params['name'])
                if firmware_process and firmware_process.is_alive():
                    firmware_process.terminate()
                if mavros_process and mavros_process.is_alive():
                    mavros_process.terminate()
                if ros_gz_bridge_process and ros_gz_bridge_process.is_alive():
                    ros_gz_bridge_process.terminate()
                self._user_input_manager.assigned_ids.remove(robot_params['ID'])
                self._gazebo_spawn_future = None
                return

            glob_running_processes.append(firmware_process)
            glob_running_processes.append(mavros_process)
            if ros_gz_bridge_process is not None:
                glob_running_processes.append(ros_gz_bridge_process)

            self.get_logger().info(f'Vehicle {robot_params["name"]} successfully spawned')
            self._active_vehicles.append(robot_params['name'])
            self._gazebo_spawn_future = None

        except Exception as e:
            self.get_logger().error(
                f'Spawning failed for {robot_params["name"]} with error: {e}, aborting launch sequence.')
            self._user_input_manager.assigned_ids.remove(robot_params['ID'])
            self._gazebo_spawn_future = None
            return

    # #}

    # #{ delete_gazebo_model(self, name)
    def delete_gazebo_model(self, name):
        self.get_logger().info(f'Requesting delete for model {name}')
        request = DeleteEntity.Request()
        request.entity.name = name

        self._gazebo_delete_future = self._gazebo_delete_proxy.call_async(request)
        self._gazebo_delete_future.add_done_callback(
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

            self._gazebo_delete_future = None

        except Exception as e:
            self.get_logger().error(f'Failed to delete model {name}. Error: {e}')
            self._gazebo_spawn_future = None

    # #}

    # #{ callback_spawn(self, request, response)
    def callback_spawn(self, request, response):
        if not self._gazebo_spawn_proxy.wait_for_service(timeout_sec=5.0):
            service_name = self._gazebo_spawn_proxy.service_name
            self.get_logger().error(f'Gazebo spawn service "{service_name}" not available.')
            response.success = False
            response.message = f'Gazebo spawn service "{service_name}" not available.'
            return response

        self._spawn_called = True
        self.get_logger().info(f'Spawn called with args "{request.value}"')
        response.success = False

        params_dict = None
        already_assigned_ids = copy.deepcopy(self._user_input_manager.assigned_ids)
        try:
            params_dict = self._user_input_manager.parse_user_input(request.value)
        except Exception as e:
            self.get_logger().warn(f'While parsing user input: {e}')
            response.message = str(e.args[0])
            self._user_input_manager.assigned_ids = already_assigned_ids
            return response

        help_text = self._user_input_manager.get_help_text(params_dict)
        if help_text is not None:
            self.get_logger().info(help_text)
            response.message = help_text.replace('\n', ' ').replace('\t', ' ')
            response.success = True
            return response

        if not self._user_input_manager.check_user_request(params_dict):
            self.get_logger().warn("User request contains invalid arguments.")
            response.message = ("The request contains invalid arguments. "
                                "Use the --help option to see the supported arguments.")
            response.success = False
            self._user_input_manager.assigned_ids = already_assigned_ids
            return response

        self.get_logger().info(f'Spawner params assigned "{params_dict}"')

        self.get_logger().info('Adding vehicles to a spawn queue')
        self._processing = True
        with self._queue_mutex:
            for i, ID in enumerate(params_dict['ids']):
                robot_params = self.get_jinja_params_for_one_robot(params_dict, i, ID)
                self._vehicle_queue.append(robot_params)

        response.success = True
        response.message = f'Launch sequence queued for {len(params_dict["ids"])} robots'
        return response

    # #}

    # #{ callback_action_timer(self)
    def callback_action_timer(self):
        # Check for an ongoing request and if it has timed out
        if self._gazebo_spawn_future is not None and not self._gazebo_spawn_future.done(
        ) and self._gazebo_spawn_request_start_time is not None:
            if time.time() - self._gazebo_spawn_request_start_time > 5.0:
                self.get_logger().error('Service call timed out!')
                self._gazebo_spawn_future = None  # Reset state to allow a new request
            else:
                self.get_logger().warn('Previous gazebo_spawn service call is pending. Skipping this cycle.')
            return
        with self._queue_mutex:
            if not self._vehicle_queue:
                self._processing = False
                return
            robot_params = self._vehicle_queue.pop(0)

        self.spawn_gazebo_model(robot_params)

        if len(self._vehicle_queue) == 0:
            self._sdf_to_tf_publisher.publish_sensor_tfs()

    # #}

    # #{ callback_diagnostics_timer(self)
    def callback_diagnostics_timer(self):
        diagnostics = GazeboSpawnerDiagnostics()
        diagnostics.spawn_called = self._spawn_called
        diagnostics.processing = self._processing
        diagnostics.active_vehicles = self._active_vehicles
        self._queue_mutex.acquire()
        diagnostics.queued_vehicles = [params['name'] for params in self._vehicle_queue]
        diagnostics.queued_processes = len(self._vehicle_queue)
        self._queue_mutex.release()
        self._diagnostics_pub.publish(diagnostics)

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
