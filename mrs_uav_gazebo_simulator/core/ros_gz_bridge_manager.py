import os
import jinja2
import xml.dom.minidom
from xml.dom.minidom import Element
import multiprocessing

from mrs_uav_gazebo_simulator.utils.spawner_types import *
from mrs_uav_gazebo_simulator.utils.spawner_exceptions import *

# ROS 2 Imports
from rclpy.node import Node
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


class RosGzBridgeManager():

    # #{ __init__(self, ros_node, gazebo_simulator_path, tempfile_folder)
    def __init__(self, ros_node: Node, gazebo_simulator_path: str, tempfile_folder: str):
        self._ros_node = ros_node

        self._uav_ros_gz_bridge_launch_path = os.path.join(gazebo_simulator_path, 'launch',
                                                           'uav_ros_gz_bridge.launch.py')
        self._uav_ros_gz_bridge_config_path = os.path.join(gazebo_simulator_path, 'config')
        self._uav_ros_gz_bridge_config_template_name = 'uav_ros_gz_bridge_config.yaml.jinja'

        self._tempfile_folder = tempfile_folder

    # #}

    # #{ generate_uav_ros_gz_config(self, robot_params: dict) -> tuple[str, dict]:
    def generate_uav_ros_gz_config(self, robot_params: dict) -> tuple[str, dict]:
        uav_name = robot_params['name']

        jinja_env = jinja2.Environment(loader=jinja2.FileSystemLoader(self._uav_ros_gz_bridge_config_path),
                                       autoescape=False)

        template = jinja_env.get_template(self._uav_ros_gz_bridge_config_template_name)

        attached_sensors = self._get_attached_sensors(robot_params)
        attached_plugins = self._get_attached_plugins(robot_params)

        if not self._has_attached_sensors(attached_sensors) and not self._has_attached_plugins(attached_plugins):
            self._ros_node.get_logger().info(
                f"There are no additional sensors nor plugins. Skipping launching ros_gz_bridge for {uav_name}")
            return "", {}

        ros_gz_topics = {category: [] for category in RosGzBridgeCategory}
        self._get_sensor_topics(ros_gz_topics, attached_sensors)
        self._get_plugin_topics(ros_gz_topics, attached_plugins)

        rendered_template = template.render(camera_info_topic_list=ros_gz_topics[RosGzBridgeCategory.CAMERA_INFO],
                                            laser_scan_topic_list=ros_gz_topics[RosGzBridgeCategory.LASER_SCAN],
                                            point_cloud_topic_list=ros_gz_topics[RosGzBridgeCategory.POINTCLOUD],
                                            imu_topic_list=ros_gz_topics[RosGzBridgeCategory.IMU],
                                            odometry_topic_list=ros_gz_topics[RosGzBridgeCategory.ODOMETRY])

        filename = f'ros_gz_bridge_config_{uav_name}.yaml'
        filepath = os.path.join(self._tempfile_folder, filename)

        with open(filepath, 'w', encoding='utf-8') as f:
            f.write(rendered_template)
            self._ros_node.get_logger().info(f'ros_gz_bridge config for {uav_name} written to {filepath}')

        return filepath, ros_gz_topics

    # #}

    # #{ launch_uav_ros_gz_bridge(self, uav_name, ros_gz_bridge_config, sensor_topics)
    def launch_uav_ros_gz_bridge(self, uav_name, ros_gz_bridge_config, sensor_topics):
        self._ros_node.get_logger().info(f'Launching ros_gz_bridge for {uav_name}')

        launch_arguments = {
            'namespace': uav_name,
            'ros_gz_bridge_config': str(ros_gz_bridge_config),
            'ros_gz_image_topics': ' '.join(sensor_topics[RosGzBridgeCategory.IMAGE]),
            'bridge_debug': 'false',
        }

        ld = LaunchDescription([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(self._uav_ros_gz_bridge_launch_path),
                launch_arguments=launch_arguments.items(),
            )
        ])

        self._ros_node.get_logger().info(f'launch_arguments: {launch_arguments}')
        launch_service = LaunchService(debug=False)
        launch_service.include_launch_description(ld)
        ros_gz_bridge_process = multiprocessing.Process(target=launch_service.run)

        try:
            ros_gz_bridge_process.start()
        except Exception as e:
            self._ros_node.get_logger().error(
                f'Could not start ros_gz_bridge for {uav_name}. Node failed to launch: {e}')
            raise CouldNotLaunch('ros_gz_bridge failed to launch')

        self._ros_node.get_logger().info(f'ros_gz_bridge for {uav_name} launched')
        return ros_gz_bridge_process

    # #}

    # #{ _get_attached_plugins(self, robot_params: dict) -> dict:
    def _get_attached_plugins(self, robot_params: dict) -> dict:
        attached_plugins = {plugin: [] for plugin in GazeboPlugins}

        xmldoc = xml.dom.minidom.parse(robot_params['sdf_filepath'])
        plugin_blocks = xmldoc.getElementsByTagName('plugin')

        for plugin in plugin_blocks:
            plugin_name = plugin.getAttribute('name')
            if plugin_name == GazeboPlugins.ODOMETRY_PUBLISHER:
                self._get_odometry_plugin(attached_plugins, plugin)

        return attached_plugins

    # #}

    # #{ _get_odometry_plugin(self, attached_plugins: dict, plugin: Element) -> None
    def _get_odometry_plugin(self, attached_plugins: dict, plugin: Element) -> None:
        odom_topic = self._get_elem_topic_from_tag_name(plugin, 'odom_topic')
        odom_covariance_topic = self._get_elem_topic_from_tag_name(plugin, 'odom_covariance_topic')

        odometry = OdometryRosGzBridge(ros_odometry_topic=odom_topic,
                                       gz_odometry_topic=odom_topic,
                                       ros_odometry_cov_topic=odom_covariance_topic,
                                       gz_odometry_cov_topic=odom_covariance_topic)

        attached_plugins[GazeboPlugins.ODOMETRY_PUBLISHER].append(odometry)

    # #}

    # #{ _get_attached_sensors(self, robot_params: dict) -> dict:
    def _get_attached_sensors(self, robot_params: dict) -> dict:
        attached_sensors = {sensor: [] for sensor in AttachedSensors}

        # not using try-catch, it's already done during the sdf's generation
        xmldoc = xml.dom.minidom.parse(robot_params['sdf_filepath'])
        sensor_blocks = xmldoc.getElementsByTagName('sensor')

        for sensor in sensor_blocks:
            sensor_type = sensor.getAttribute('type')
            if sensor_type == GazeboSensors.CAMERA:
                self._get_attached_camera(attached_sensors, sensor)
            elif sensor_type == GazeboSensors.LIDAR:
                self._get_attached_lidar(attached_sensors, sensor)
            elif sensor_type == GazeboSensors.RGBD_CAMERA:
                self._get_attached_rgbd_camera(attached_sensors, sensor)
            elif sensor_type == GazeboSensors.DEPTH_CAMERA:
                self._get_attached_depth_camera(attached_sensors, sensor)
            elif sensor_type == GazeboSensors.IMU:
                self._get_attached_imu(attached_sensors, sensor)
        return attached_sensors

    # #}

    # #{ _get_attached_camera(self, attached_sensors, camera_sensor)
    def _get_attached_camera(self, attached_sensors: dict, camera_sensor: Element) -> None:
        gz_camera_info_topic = self._get_elem_topic_from_tag_name(camera_sensor, SdfTopicTags.GZ_CAMERA_INFO)
        ros_camera_info_topic = self._get_elem_topic_from_tag_name(camera_sensor, SdfTopicTags.ROS_CAMERA_INFO)
        ros_color_image_topic = self._get_elem_topic_from_tag_name(camera_sensor, SdfTopicTags.ROS_COLOR_IMAGE)

        required_topics = [gz_camera_info_topic, ros_camera_info_topic, ros_color_image_topic]

        if not all(required_topics):
            self._ros_node.get_logger().warn("Skipping RGB camera because one or more required topics are missing.")
            return

        camera = CameraRosGzBridge(image_topic=ros_color_image_topic,
                                   ros_info_topic=ros_camera_info_topic,
                                   gz_info_topic=gz_camera_info_topic)

        attached_sensors[AttachedSensors.CAMERAS].append(camera)

    # #}

    # #{ _get_attached_depth_camera(self, attached_sensors, camera_sensor)
    def _get_attached_depth_camera(self, attached_sensors: dict, camera_sensor: Element) -> None:
        gz_camera_info_topic = self._get_elem_topic_from_tag_name(camera_sensor, SdfTopicTags.GZ_CAMERA_INFO)
        gz_pointcloud_topic = self._get_elem_topic_from_tag_name(camera_sensor, SdfTopicTags.GZ_POINTCLOUD)
        ros_camera_info_topic = self._get_elem_topic_from_tag_name(camera_sensor, SdfTopicTags.ROS_CAMERA_INFO)
        ros_depth_image_topic = self._get_elem_topic_from_tag_name(camera_sensor, SdfTopicTags.ROS_DEPTH_IMAGE)
        ros_pointcloud_topic = self._get_elem_topic_from_tag_name(camera_sensor, SdfTopicTags.ROS_POINTCLOUD)

        required_topics = [
            gz_camera_info_topic, gz_pointcloud_topic, ros_camera_info_topic, ros_depth_image_topic,
            ros_pointcloud_topic
        ]

        if not all(required_topics):
            self._ros_node.get_logger().warn("Skipping Depth camera because one or more required topics are missing.")
            return

        depth_camera = DepthCameraRosGzBridge(image_topic=ros_depth_image_topic,
                                              ros_info_topic=ros_camera_info_topic,
                                              gz_info_topic=gz_camera_info_topic,
                                              ros_points_topic=ros_pointcloud_topic,
                                              gz_points_topic=gz_pointcloud_topic)

        attached_sensors[AttachedSensors.DEPTH_CAMERAS].append(depth_camera)

    # #}

    # #{ _get_attached_rgbd_camera(self, attached_sensors, camera_sensor)
    def _get_attached_rgbd_camera(self, attached_sensors: dict, camera_sensor: Element) -> None:
        gz_camera_info_topic = self._get_elem_topic_from_tag_name(camera_sensor, SdfTopicTags.GZ_CAMERA_INFO)
        gz_pointcloud_topic = self._get_elem_topic_from_tag_name(camera_sensor, SdfTopicTags.GZ_POINTCLOUD)
        ros_camera_info_topic = self._get_elem_topic_from_tag_name(camera_sensor, SdfTopicTags.ROS_CAMERA_INFO)
        ros_color_image_topic = self._get_elem_topic_from_tag_name(camera_sensor, SdfTopicTags.ROS_COLOR_IMAGE)
        ros_depth_image_topic = self._get_elem_topic_from_tag_name(camera_sensor, SdfTopicTags.ROS_DEPTH_IMAGE)
        ros_pointcloud_topic = self._get_elem_topic_from_tag_name(camera_sensor, SdfTopicTags.ROS_POINTCLOUD)

        required_topics = [
            gz_camera_info_topic,
            gz_pointcloud_topic,
            ros_camera_info_topic,
            ros_color_image_topic,
            ros_depth_image_topic,
            ros_pointcloud_topic,
        ]

        if not all(required_topics):
            self._ros_node.get_logger().warn("Skipping RGB-D camera because one or more required topics are missing.")
            return

        camera = RgbdCameraRosGzBridge(rgb_image_topic=ros_color_image_topic,
                                       depth_image_topic=ros_depth_image_topic,
                                       ros_info_topic=ros_camera_info_topic,
                                       gz_info_topic=gz_camera_info_topic,
                                       ros_points_topic=ros_pointcloud_topic,
                                       gz_points_topic=gz_pointcloud_topic)

        attached_sensors[AttachedSensors.RGBD_CAMERAS].append(camera)

    # #}

    # #{ _get_attached_lidar(self, attached_sensors, lidar_sensor)
    def _get_attached_lidar(self, attached_sensors: dict, lidar_sensor: Element) -> None:

        # NOTE: PX4 requires its own bridge with the Garmin rangefinder, so we do not set it up.
        # The Garmin rangefinder link is named 'lidar_sensor_link' in the garmin.sdf.jinja template.
        # Do not rename the Garmin link or the rangefinder plugin, as PX4 may fail to detect it otherwise.
        if lidar_sensor.getAttribute('name') == "lidar_sensor_link":  # Garmin rangefinder
            return

        gz_pointcloud_topic = self._get_elem_topic_from_tag_name(lidar_sensor, SdfTopicTags.GZ_POINTCLOUD)
        ros_pointcloud_topic = self._get_elem_topic_from_tag_name(lidar_sensor, SdfTopicTags.ROS_POINTCLOUD)

        required_topics = [gz_pointcloud_topic, ros_pointcloud_topic]

        if not all(required_topics):
            self._ros_node.get_logger().warn("Skipping Lidar because one or more required topics are missing.")
            return

        vertical_samples = self._get_number_of_vertical_samples(lidar_sensor)
        lidar = LidarRosGzBridge(ros_points_topic=ros_pointcloud_topic, gz_points_topic=gz_pointcloud_topic)
        if vertical_samples == 1:  # 2D lidar
            attached_sensors[AttachedSensors.TWO_D_LIDAR].append(lidar)
        elif vertical_samples > 1:  # 3D lidar
            attached_sensors[AttachedSensors.THREE_D_LIDAR].append(lidar)
        else:  # Incorrect lidar
            self._ros_node.get_logger().error(
                f"The lidar {lidar_sensor.getAttribute('name')} cannot be loaded. Check if the number of vertical samples is correct."
            )

        return

    # #}

    # #{ _get_attached_imu(self, attached_sensors, imu_sensor)
    def _get_attached_imu(self, attached_sensors, imu_sensor):
        gz_imu_topic = self._get_elem_topic_from_tag_name(imu_sensor, SdfTopicTags.GZ_IMU)
        ros_imu_topic = self._get_elem_topic_from_tag_name(imu_sensor, SdfTopicTags.ROS_IMU)

        required_topics = [gz_imu_topic, ros_imu_topic]

        if not all(required_topics):
            self._ros_node.get_logger().warn("Skipping IMU because one or more required topics are missing.")
            return

        imu = ImuRosGzBridge(ros_imu_topic=ros_imu_topic, gz_imu_topic=gz_imu_topic)

        attached_sensors[AttachedSensors.IMU].append(imu)

    # #}

    # #{ _get_elem_topic_from_tag_name(self, elem, tag_name)
    def _get_elem_topic_from_tag_name(self, elem, tag_name):
        topic = elem.getElementsByTagName(tag_name)
        if topic:
            topic = '/' + topic[0].firstChild.data
        return topic

    # #}

    # #{ _get_number_of_vertical_samples(self, lidar_sensor)
    def _get_number_of_vertical_samples(self, lidar_sensor):
        vertical_elements = lidar_sensor.getElementsByTagName('vertical')
        samples_elements = vertical_elements[0].getElementsByTagName('samples')
        vertical_samples = int(samples_elements[0].firstChild.nodeValue.strip())

        return vertical_samples

    # #}

    # #{ _has_attached_sensors(self, attached_sensors)
    def _has_attached_sensors(self, attached_sensors):
        return any(attached_sensors[s] for s in AttachedSensors)

    # #}

    # #{ _has_attached_plugins(self, attached_plugins)
    def _has_attached_plugins(self, attached_plugins):
        return any(attached_plugins[p] for p in GazeboPlugins)

    # #}

    # #{ _get_sensor_topics(self, attached_sensors: dict) -> dict:
    def _get_sensor_topics(self, sensor_topics: dict, attached_sensors: dict) -> dict:

        for camera in attached_sensors[AttachedSensors.CAMERAS]:
            sensor_topics[RosGzBridgeCategory.IMAGE].append(camera.image_topic)
            sensor_topics[RosGzBridgeCategory.CAMERA_INFO].append(
                RosGzBridgeTopics(gazebo=camera.gz_info_topic, ros=camera.ros_info_topic))

        for two_d_lidar in attached_sensors[AttachedSensors.TWO_D_LIDAR]:
            sensor_topics[RosGzBridgeCategory.LASER_SCAN].append(
                RosGzBridgeTopics(gazebo=two_d_lidar.gz_points_topic, ros=two_d_lidar.ros_points_topic))

        for three_d_lidar in attached_sensors[AttachedSensors.THREE_D_LIDAR]:
            sensor_topics[RosGzBridgeCategory.POINTCLOUD].append(
                RosGzBridgeTopics(gazebo=three_d_lidar.gz_points_topic, ros=three_d_lidar.ros_points_topic))

        for rgbd_camera in attached_sensors[AttachedSensors.RGBD_CAMERAS]:
            sensor_topics[RosGzBridgeCategory.IMAGE].append(rgbd_camera.rgb_image_topic)
            sensor_topics[RosGzBridgeCategory.IMAGE].append(rgbd_camera.depth_image_topic)
            sensor_topics[RosGzBridgeCategory.CAMERA_INFO].append(
                RosGzBridgeTopics(gazebo=rgbd_camera.gz_info_topic, ros=rgbd_camera.ros_info_topic))
            #NOTE: Real-world cameras do not provide this topic, so we do not publish it either.
            # sensor_topics[RosGzBridgeCategory.POINTCLOUD].append(
            #     RosGzBridgeTopics(gazebo=rgbd_camera.gz_points_topic,
            #                       ros=rgbd_camera.ros_points_topic))

        for depth_camera in attached_sensors[AttachedSensors.DEPTH_CAMERAS]:
            sensor_topics[RosGzBridgeCategory.IMAGE].append(depth_camera.image_topic)
            sensor_topics[RosGzBridgeCategory.CAMERA_INFO].append(
                RosGzBridgeTopics(gazebo=depth_camera.gz_info_topic, ros=depth_camera.ros_info_topic))
            #NOTE: Real-world cameras do not provide this topic, so we do not publish it either.
            # sensor_topics[RosGzBridgeCategory.POINTCLOUD].append(
            #     RosGzBridgeTopics(gazebo=depth_camera.gz_points_topic, ros=depth_camera.ros_points_topic))

        for imu in attached_sensors[AttachedSensors.IMU]:
            sensor_topics[RosGzBridgeCategory.IMU].append(
                RosGzBridgeTopics(gazebo=imu.gz_imu_topic, ros=imu.ros_imu_topic))

    # #}

    # #{ _get_plugin_topics(self, attached_plugins: dict) -> dict:
    def _get_plugin_topics(self, plugin_topics, attached_plugins: dict) -> dict:
        for odometry_plugin in attached_plugins[GazeboPlugins.ODOMETRY_PUBLISHER]:
            plugin_topics[RosGzBridgeCategory.ODOMETRY].append(
                RosGzBridgeTopics(gazebo=odometry_plugin.gz_odometry_topic, ros=odometry_plugin.ros_odometry_topic))
            # plugin_topics[RosGzBridgeCategory.ODOMETRY_WITH_COV].append(
            #     RosGzBridgeTopics(gazebo=odometry_plugin.gz_odometry_cov_topic,
            #                       ros=odometry_plugin.ros_odometry_cov_topic))

    # #}
