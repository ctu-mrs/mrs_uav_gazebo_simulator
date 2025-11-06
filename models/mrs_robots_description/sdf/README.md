## Camera frame and Topic naming
### RGB Camera
Due to different frame conventions, an additional optical frame is required. You may change its name, since the TF publisher will read it from the SDF file. For example, I recommend checking the [bluefox_camera_macro](./components/camera/bluefox.sdf.jinja).
```
{%- set camera_gz_frame_id = spawner_args['name'] + '/' + camera_name + '/color_optical' -%}
```

The resulting camera-related topics will look like as follows:
- `/namespace/camera_name/image`
- `/namespace/camera_name/camera_info`

### Depth Camera
Due to different frame conventions, an additional optical frame is required. You may change its name, since the TF publisher will read it from the SDF file. For example, I recommend checking the [realsense_depth_macro](./components/camera/realsense.sdf.jinja).
```
{%- set camera_gz_frame_id = spawner_args['name'] + '/' + camera_name + '/depth_optical' -%}
```

The resulting camera-related topics will look like as follows:
- `/namespace/camera_name/camera_info`
- `/namespace/camera_name/depth_image`
- `/namespace/camera_name/depth_image/points`


### RGB-D Camera
Due to different frame conventions, an additional optical frame is required. However, there is currently a bug that causes the point cloud to be misaligned with the camera's optical frame. As a result, the point cloud data must be manually transformed, which is not implemented on our side. Nevertheless, we still provide the optical frame for the RGB data.
```
{%- set camera_gz_frame_id = spawner_args['name'] + '/' + camera_name + '/camera_optical' -%}
```

The resulting camera-related topics will look like as follows:
- `/namespace/camera_name/camera_info`
- `/namespace/camera_name/image`
- `/namespace/camera_name/depth_image`
- `/namespace/camera_name/points`

### 2D lidars
The resulting 2D lidar-related topics will look like as follows:
- /namespace/sensor_name/scan

There is one more related topic on the Gazebo server that converts `/scan` into a point cloud representation, but it is not being bridged to ROS:
- /namespace/sensor_name/scan/points

### 3D lidars
The resulting 2D lidar-related topics will look like as follows:
- /namespace/sensor_name/points

There is one more related topic on the Gazebo server that converts `/points` into a 2D lidar scan representation, but it is not being bridged to ROS:
- /namespace/sensor_name
