# Frame and Topic Naming Conventions

## Table of Contents  
- [Optical Frame for Cameras](#optical-frame-for-cameras)  
- [Naming Rules](#naming-rules)  
  - [RGB Camera](#rgb-camera)
  - [Depth Camera](#depth-camera)
  - [RGB-D Camera](#rgb-d-camera)
  - [Lidar](#lidar)
  - [Debugging](#debugging)

## Optical Frame for Cameras
Due to different frame conventions, an additional optical frame is required. The camera <sensor> plugin uses the optical frame specified in the message header, even though the plugin is defined under a different link. You may change its name, since the TF publisher will read it from the SDF file.
```
{%- set camera_gz_frame_id = spawner_args['name'] + '/' + camera_name + '_optical' -%}
```

## Naming Rules
To reduce the Sim2Real gap, we provide topic arguments that are passed into a dynamically generated SDF file. These arguments are inserted as custom SDF tags, which are ignored by Gazebo but read by `mrs_drone_spawner` to generate the ros_gz_bridge config file. While this allows us to customize topic names so that they match those used by real-world sensors, there are certain constraints.


### RGB Camera
For bridging images from Gazebo to ROS, we use `ros_gz_image`. This bridge imposes a constraint that the image topic names in Gazebo and ROS must be the same. For RGB cameras the sensor plugin topic is passed as `camera_sdf_topic`:
```
{%- set root_sdf_topic = spawner_args['name'] + '/' + camera_name -%}     {# -- Can be modified -- #}
{%- set camera_sdf_topic = root_sdf_topic + '/image_raw' -%}              {# -- Can be modified -- #}
{%- set gz_image_topic = camera_sdf_topic -%}                             {# -- Do not modify -- #}
{%- set ros_image_topic = camera_sdf_topic -%}                            {# -- Do not modify -- #}
```
The Gazebo RGB camera plugin then automatically publishes the `CameraInfo` message to the topic:
```
{%- set gz_camera_info_topic = root_sdf_topic + '/camera_info' -%}      {# -- Do not modify -- #}
```
This topic name cannot be changed, as it is fixed by the plugin. However, the corresponding ROS topic can be arbitrarily named:
```
{%- set ros_camera_info_topic = spawner_args['name'] + '/' + camera_name + '/camera_info' -%}
```
For a working example, we recommend checking the [bluefox_camera_macro](./components/camera/bluefox.sdf.jinja). With the `spawner_args['name']='/uav'` and `camera_name='bluefox'`, the topics look as follows:
<table>
  <thead>
    <tr>
      <th colspan="2" style="border: 1px solid; text-align:center;">Gazebo topics</th>
      <th colspan="2" style="border: 1px solid; text-align:center;">ROS topics</th>
    </tr>
    <tr>
      <th style="border: 1px solid; text-align:center;">Topic name</th>
      <th style="border: 1px solid; text-align:center;">Msg type</th>
      <th style="border: 1px solid; text-align:center;">Topic name</th>
      <th style="border: 1px solid; text-align:center;">Msg type</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td style="border: 1px solid;">/uav1/bluefox/camera_info</td>
      <td style="border: 1px solid;">gz.msgs.CameraInfo</td>
      <td style="border: 1px solid;">/uav1/bluefox/camera_info</td>
      <td style="border: 1px solid;">sensor_msgs/msg/CameraInfo</td>
    </tr>
    <tr>
      <td style="border: 1px solid;">/uav1/bluefox/image_raw</td>
      <td style="border: 1px solid;">gz.msgs.Image</td>
      <td style="border: 1px solid;">/uav1/bluefox/image_raw</td>
      <td style="border: 1px solid;">sensor_msgs/msg/Image</td>
    </tr>
  </tbody>
</table>

The `ros_gz_image` node also provides the following ROS topics:
- `/namespace/camera_name/image_raw/compressed`
- `/namespace/camera_name/image_raw/compressedDepth`
- `/namespace/camera_name/image_raw/theora`
- `/namespace/camera_name/image_raw/zstd`


### Depth Camera
For bridging images from Gazebo to ROS, we use `ros_gz_image`. This bridge imposes the constraint that the image topic names in Gazebo and ROS must be identical. For RGB cameras, the sensor plugin topic is passed as `camera_sdf_topic`:
```
{%- set root_sdf_topic = spawner_args['name'] + '/' + camera_name -%}           {# -- Can be modified -- #}
{%- set camera_sdf_topic = root_sdf_topic + '/image_raw' -%}                    {# -- Can be modified -- #}
{%- set gz_depth_image_topic = camera_sdf_topic -%}                             {# -- Do not modify -- #}
{%- set ros_depth_image_topic = camera_sdf_topic -%}                            {# -- Do not modify -- #}
```
The depth camera plugin also publishes a point cloud message based on the depth image, but it always uses a fixed topic name:
```
{%- set gz_pointcloud_topic = camera_sdf_topic + '/points' -%}          {# -- Do not modify -- #}
```
However, since the point cloud and camera info are bridged using the standard `ros_gz_bridge`, the corresponding ROS topic names can be chosen freely:
```
{%- set ros_camera_info_topic = spawner_args['name'] + '/' + camera_name + '/camera_info' -%}                 
{%- set ros_pointcloud_topic = spawner_args['name'] + '/' + camera_name + '/points' -%}    
```
For a working example, we recommend checking the [realsense_depth_macro](./components/camera/realsense.sdf.jinja). The table below shows the Gazebo and ROS topics for Realsense depth-only camera:
<table>
  <thead>
    <tr>
      <th colspan="2" style="border: 1px solid; text-align:center;">Gazebo topics</th>
      <th colspan="2" style="border: 1px solid; text-align:center;">ROS topics</th>
    </tr>
    <tr>
      <th style="border: 1px solid; text-align:center;">Topic name</th>
      <th style="border: 1px solid; text-align:center;">Msg type</th>
      <th style="border: 1px solid; text-align:center;">Topic name</th>
      <th style="border: 1px solid; text-align:center;">Msg type</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td style="border: 1px solid;">/uav1/realsense/camera_info</td>
      <td style="border: 1px solid;">gz.msgs.CameraInfo</td>
      <td style="border: 1px solid;">/uav1/realsense/camera_info</td>
      <td style="border: 1px solid;">sensor_msgs/msg/CameraInfo</td>
    </tr>
    <tr>
      <td style="border: 1px solid;">/uav1/realsense/image_raw</td>
      <td style="border: 1px solid;">gz.msgs.Image</td>
      <td style="border: 1px solid;">/uav1/realsense/image_raw</td>
      <td style="border: 1px solid;">sensor_msgs/msg/Image</td>
    </tr>
    <tr>
      <td style="border: 1px solid;">/uav1/realsense/image_raw/points</td>
      <td style="border: 1px solid;">gz.msgs.PointCloudPacked</td>
      <td style="border: 1px solid;">/uav1/realsense/points</td>
      <td style="border: 1px solid;">sensor_msgs/msg/PointCloud2</td>
    </tr>
  </tbody>
</table>

The `ros_gz_image` node also provides the following ROS topics:
- `/namespace/camera_name/image_raw/compressed`
- `/namespace/camera_name/image_raw/compressedDepth`
- `/namespace/camera_name/image_raw/theora`
- `/namespace/camera_name/image_raw/zstd`

### RGB-D Camera
There are technically two ways to create a macro for RGB-D cameras:
- [Gazebo RGB-D Camera Plugin](#gazebo-rgbd-camera-plugin)
- [Combination of RGB and Depth Camera Plugins](#combination-of-rgb-and-depth-camera-plugins)

#### Gazebo RGBD Camera Plugin
The gazebo provides a plugin for RGB-D cameras, however it has restrictions regarding names as well.
```
{%- set root_sdf_topic = spawner_args['name'] + '/' + camera_name -%}
{%- set camera_sdf_topic = root_sdf_topic -%}
```
The topics for publishing color and depth image will have their names based on the `camera_sdf_topic` as follows
```
{%- set gz_image_topic = camera_sdf_topic + '/image' -%}                  {# -- Do not modify -- #}
{%- set gz_depth_image_topic = camera_sdf_topic + '/depth_image' -%}      {# -- Do not modify -- #}
```
Since these image topics are transported through `ros_gz_image` bridge, the ROS topic names will have the same name and cannot be modified.

The Gazebo camera info and point cloud topics are named by the plugin, but we can modify their ROS version:
```
{%- set gz_pointcloud_topic = camera_sdf_topic + '/points' -%}          {# -- Do not modify -- #}
{%- set gz_camera_info_topic = root_sdf_topic + '/camera_info' -%}      {# -- Do not modify -- #}
{%- set ros_camera_info_topic = root_sdf_topic + '/camera_info' -%}     {# -- Can be modified -- #}          
{%- set ros_pointcloud_topic = root_sdf_topic + '/points' -%}           {# -- Can be modified -- #} 
```

For a working example, we recommend checking the [realsense_rgbd_single_plugin](./components/camera/realsense_rgbd_single_plugin.sdf.jinja). The table below shows the Gazebo and ROS topics for Realsense RGB-D camera:
<table>
  <thead>
    <tr>
      <th colspan="2" style="border: 1px solid; text-align:center;">Gazebo topics</th>
      <th colspan="2" style="border: 1px solid; text-align:center;">ROS topics</th>
    </tr>
    <tr>
      <th style="border: 1px solid; text-align:center;">Topic name</th>
      <th style="border: 1px solid; text-align:center;">Msg type</th>
      <th style="border: 1px solid; text-align:center;">Topic name</th>
      <th style="border: 1px solid; text-align:center;">Msg type</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td style="border: 1px solid;">/uav1/realsense/camera_info</td>
      <td style="border: 1px solid;">gz.msgs.CameraInfo</td>
      <td style="border: 1px solid;">/uav1/realsense/camera_info</td>
      <td style="border: 1px solid;">sensor_msgs/msg/CameraInfo</td>
    </tr>
    <tr>
      <td style="border: 1px solid;">/uav1/realsense/depth_image</td>
      <td style="border: 1px solid;">gz.msgs.Image</td>
      <td style="border: 1px solid;">/uav1/realsense/depth_image</td>
      <td style="border: 1px solid;">sensor_msgs/msg/Image</td>
    </tr>
    <tr>
      <td style="border: 1px solid;">/uav1/realsense/image</td>
      <td style="border: 1px solid;">gz.msgs.Image</td>
      <td style="border: 1px solid;">/uav1/realsense/image</td>
      <td style="border: 1px solid;">sensor_msgs/msg/Image</td>
    </tr>
    <tr>
      <td style="border: 1px solid;">/uav1/realsense/points</td>
      <td style="border: 1px solid;">gz.msgs.PointCloudPacked</td>
      <td style="border: 1px solid;">/uav1/realsense/points</td>
      <td style="border: 1px solid;">sensor_msgs/msg/PointCloud2</td>
    </tr>
  </tbody>
</table>

#### Combination of RGB and Depth Camera Plugins
The RGB-D camera can be created by combining the RGB camera and Depth camera plugins. The advantage of this approach is that it allows us to modify the topic names more flexibly and assign separate properties to each camera. Since we are still using `ros_gz_image`, the image topics must have the same name between Gazebo and ROS. However, because we technically have two separate cameras, we can organize the RGB and Depth topics into two groups:

1. RGB Camera
```
{%- set color_root_sdf_topic = spawner_args['name'] + '/' + camera_name + '/color' -%}
{%- set camera_color_sdf_topic = color_root_sdf_topic + '/image_raw' -%}
{%- set gz_color_camera_info_topic = color_root_sdf_topic + '/camera_info' -%}      {# -- Do not modify -- #}          
{%- set gz_color_image_topic = camera_color_sdf_topic -%}                           {# -- Do not modify -- #}
{%- set ros_color_image_topic = camera_color_sdf_topic -%}                          {# -- Do not modify -- #}
{%- set ros_color_camera_info_topic = color_root_sdf_topic + '/camera_info' -%}          
```
2. Depth Camera
```
{%- set depth_root_sdf_topic = spawner_args['name'] + '/' + camera_name + '/depth' -%}
{%- set camera_depth_sdf_topic = depth_root_sdf_topic + '/image_raw' -%}
{%- set gz_depth_camera_info_topic = depth_root_sdf_topic + '/camera_info' -%}      {# -- Do not modify -- #}          
{%- set gz_depth_image_topic = camera_depth_sdf_topic -%}                           {# -- Do not modify -- #}
{%- set gz_pointcloud_topic = camera_depth_sdf_topic + '/points' -%}                {# -- Do not modify -- #}
{%- set ros_depth_image_topic = camera_depth_sdf_topic -%}                          {# -- Do not modify -- #}
{%- set ros_depth_camera_info_topic = depth_root_sdf_topic + '/camera_info' -%}        
{%- set ros_pointcloud_topic = depth_root_sdf_topic + '/points' -%}                     
```
This results in the following topics:
<table>
  <thead>
    <tr>
      <th colspan="2" style="border: 1px solid; text-align:center;">Gazebo topics</th>
      <th colspan="2" style="border: 1px solid; text-align:center;">ROS topics</th>
    </tr>
    <tr>
      <th style="border: 1px solid; text-align:center;">Topic name</th>
      <th style="border: 1px solid; text-align:center;">Msg type</th>
      <th style="border: 1px solid; text-align:center;">Topic name</th>
      <th style="border: 1px solid; text-align:center;">Msg type</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td style="border: 1px solid;">/uav1/realsense/color/camera_info</td>
      <td style="border: 1px solid;">gz.msgs.CameraInfo</td>
      <td style="border: 1px solid;">/uav1/realsense/color/camera_info</td>
      <td style="border: 1px solid;">sensor_msgs/msg/CameraInfo</td>
    </tr>
    <tr>
      <td style="border: 1px solid;">/uav1/realsense/color/image_raw</td>
      <td style="border: 1px solid;">gz.msgs.Image</td>
      <td style="border: 1px solid;">/uav1/realsense/color/image_raw</td>
      <td style="border: 1px solid;">sensor_msgs/msg/Image</td>
    </tr>
    <tr>
      <td style="border: 1px solid;">/uav1/realsense/depth/camera_info</td>
      <td style="border: 1px solid;">gz.msgs.CameraInfo</td>
      <td style="border: 1px solid;">/uav1/realsense/depth/camera_info</td>
      <td style="border: 1px solid;">sensor_msgs/msg/CameraInfo</td>
    </tr>
    <tr>
      <td style="border: 1px solid;">/uav1/realsense/depth/image_raw</td>
      <td style="border: 1px solid;">gz.msgs.Image</td>
      <td style="border: 1px solid;">/uav1/realsense/depth/image_raw</td>
      <td style="border: 1px solid;">sensor_msgs/msg/Image</td>
    </tr>
    <tr>
      <td style="border: 1px solid;">/uav1/realsense/depth/image_raw/points</td>
      <td style="border: 1px solid;">gz.msgs.PointCloudPacked</td>
      <td style="border: 1px solid;">/uav1/realsense/depth/points</td>
      <td style="border: 1px solid;">sensor_msgs/msg/PointCloud2</td>
    </tr>
  </tbody>
</table>


For more details, please check the [realsense_rgb_plus_depth.sdf.jinja](./components/camera/realsense/realsense_rgb_plus_depth.sdf.jinja) example.





### Lidar
To keep consistency with the previous sensors, we define three topics for each lidar:
1. `lidar_sdf_topic_name` - the topic used by the plugin, serving as the root topic from which other topics are derived.
2. `lidar_gz_topic_name` - the Gazebo topic that will be bridged to ROS by mrs_drone_spawner
3. `lidar_ros_topic_name` - the resulting ROS topic created by the bridge

#### 2D lidar
Example topics of 2D RPLidar. For more details, please check the [rplidar_macro.sdf.jinja](./components/lidar/rplidar/rplidar.sdf.jinja) example.
<table>
  <thead>
    <tr>
      <th colspan="2" style="border: 1px solid; text-align:center;">Gazebo topics</th>
      <th colspan="2" style="border: 1px solid; text-align:center;">ROS topics</th>
    </tr>
    <tr>
      <th style="border: 1px solid; text-align:center;">Topic name</th>
      <th style="border: 1px solid; text-align:center;">Msg type</th>
      <th style="border: 1px solid; text-align:center;">Topic name</th>
      <th style="border: 1px solid; text-align:center;">Msg type</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td style="border: 1px solid;">/uav1/rplidar/scan</td>
      <td style="border: 1px solid;">gz.msgs.LaserScan</td>
      <td style="border: 1px solid;">/uav1/rplidar/scan</td>
      <td style="border: 1px solid;">sensor_msgs/msg/LaserScan</td>
    </tr>
  </tbody>
</table>

There is one more related topic on the Gazebo server that converts `/scan` into a point cloud representation, but it is not being to ROS:
- `/namespace/sensor_name/scan/points`

#### 3D Lidar
Example topics of 3D Ouster. For more details, please check the [realsense_rgb_plus_depth.sdf.jinja](./components/lidar/ouster/ouster.sdf.jinja) example.
<table>
  <thead>
    <tr>
      <th colspan="2" style="border: 1px solid; text-align:center;">Gazebo topics</th>
      <th colspan="2" style="border: 1px solid; text-align:center;">ROS topics</th>
    </tr>
    <tr>
      <th style="border: 1px solid; text-align:center;">Topic name</th>
      <th style="border: 1px solid; text-align:center;">Msg type</th>
      <th style="border: 1px solid; text-align:center;">Topic name</th>
      <th style="border: 1px solid; text-align:center;">Msg type</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td style="border: 1px solid;">/uav1/ouster/points</td>
      <td style="border: 1px solid;">gz.msgs.PointCloudPacked</td>
      <td style="border: 1px solid;">/uav1/ouster/points</td>
      <td style="border: 1px solid;">sensor_msgs/msg/PointCloud2</td>
    </tr>
  </tbody>
</table>


There is one more related topic on the Gazebo server that converts `/points` into a 2D lidar scan representation, but it is not bridged to ROS:
- `/namespace/sensor_name`


### Debugging
The most common issue is that `ros_gz_bridge` connects to the wrong topics. The quickest way to diagnose this is to inspect the Gazebo topic information:
```bash
gz topic -i -t topic_name
```
- if there is `no publisher` the topic is not being published from Gazebo
- if there is `no subscriber` the topic is not being subscribed by `ros_gz_bridge`
Both issues are most likely caused by incorrect topic names. Verify that the topic names in your sensor’s SDF file do not violate the rules described above, and check the autogenerated `ros_gz_bridge` config file located in the `/tmp` directory.

For more information about which message types can be bridged, we recommend the following [documentation](https://docs.ros.org/en/rolling/p/ros_gz_bridge/).

#### Tests
There are unit tests that can partially verify the Jinja syntax within macros.
Be aware that these tests only ensure the Jinja templates can be rendered successfully. They do not check whether the resulting SDF is valid. Additionally, there is a camera test that verifies the naming conventions for all types of camera sensors.
These tests automatically detect the sensor folders, so you should not need to modify them. You can run these tests in the `tests` folder by running:
```bash
pytest -vv
```








