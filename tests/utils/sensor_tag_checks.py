from utils.common_utils import *


# #{ check_required_rgb_camera_tags(camera_sensor)
def check_required_rgb_camera_tags(camera_sensor):
    pose_str = get_elem_by_tag_name(camera_sensor, 'pose')
    if not check_str_to_pose(pose_str):
        raise AssertionError(f"The <pose> tag should have 6 elements.")

    get_elem_by_tag_name(camera_sensor, 'gz_frame_id')
    get_elem_by_tag_name(camera_sensor, 'update_rate')

    get_elem_by_tag_name(camera_sensor, 'topic')
    get_elem_by_tag_name(camera_sensor, 'ros_camera_info_topic')
    get_elem_by_tag_name(camera_sensor, 'ros_color_image_topic')
    get_elem_by_tag_name(camera_sensor, 'gz_camera_info_topic')

    camera = camera_sensor.getElementsByTagName('camera')[0]
    get_elem_by_tag_name(camera, 'horizontal_fov')
    image = camera.getElementsByTagName('image')[0]
    get_elem_by_tag_name(image, 'width')
    get_elem_by_tag_name(image, 'height')

    clip = camera.getElementsByTagName('clip')[0]
    get_elem_by_tag_name(clip, 'near')
    get_elem_by_tag_name(clip, 'far')

    noise = camera.getElementsByTagName('noise')[0]
    get_elem_by_tag_name(noise, 'type')
    get_elem_by_tag_name(noise, 'mean')
    get_elem_by_tag_name(noise, 'stddev')


# #}


# #{ check_required_depth_camera_tags(camera_sensor)
def check_required_depth_camera_tags(camera_sensor):
    pose_str = get_elem_by_tag_name(camera_sensor, 'pose')
    if not check_str_to_pose(pose_str):
        raise AssertionError(f"The <pose> tag should have 6 elements.")

    get_elem_by_tag_name(camera_sensor, 'gz_frame_id')
    get_elem_by_tag_name(camera_sensor, 'update_rate')

    get_elem_by_tag_name(camera_sensor, 'topic')
    get_elem_by_tag_name(camera_sensor, 'ros_camera_info_topic')
    get_elem_by_tag_name(camera_sensor, 'ros_depth_image_topic')
    get_elem_by_tag_name(camera_sensor, 'ros_pointcloud_topic')
    get_elem_by_tag_name(camera_sensor, 'gz_camera_info_topic')
    get_elem_by_tag_name(camera_sensor, 'gz_pointcloud_topic')

    camera = camera_sensor.getElementsByTagName('camera')[0]
    get_elem_by_tag_name(camera, 'horizontal_fov')
    image = camera.getElementsByTagName('image')[0]
    get_elem_by_tag_name(image, 'width')
    get_elem_by_tag_name(image, 'height')

    clip = camera.getElementsByTagName('clip')[0]
    get_elem_by_tag_name(clip, 'near')
    get_elem_by_tag_name(clip, 'far')

    noise = camera.getElementsByTagName('noise')[0]
    get_elem_by_tag_name(noise, 'type')
    get_elem_by_tag_name(noise, 'mean')
    get_elem_by_tag_name(noise, 'stddev')


# #}


# #{ check_required_rgbd_camera_tags(camera_sensor)
def check_required_rgbd_camera_tags(camera_sensor):
    pose_str = get_elem_by_tag_name(camera_sensor, 'pose')
    if not check_str_to_pose(pose_str):
        raise AssertionError(f"The <pose> tag should have 6 elements.")

    get_elem_by_tag_name(camera_sensor, 'gz_frame_id')
    get_elem_by_tag_name(camera_sensor, 'update_rate')

    get_elem_by_tag_name(camera_sensor, 'topic')
    get_elem_by_tag_name(camera_sensor, 'ros_camera_info_topic')
    get_elem_by_tag_name(camera_sensor, 'ros_color_image_topic')
    get_elem_by_tag_name(camera_sensor, 'ros_depth_image_topic')
    get_elem_by_tag_name(camera_sensor, 'ros_pointcloud_topic')
    get_elem_by_tag_name(camera_sensor, 'gz_camera_info_topic')
    get_elem_by_tag_name(camera_sensor, 'gz_pointcloud_topic')

    camera = camera_sensor.getElementsByTagName('camera')[0]
    get_elem_by_tag_name(camera, 'horizontal_fov')
    image = camera.getElementsByTagName('image')[0]
    get_elem_by_tag_name(image, 'width')
    get_elem_by_tag_name(image, 'height')
    get_elem_by_tag_name(image, 'format')

    clip = camera.getElementsByTagName('clip')[0]
    get_elem_by_tag_name(clip, 'near')
    get_elem_by_tag_name(clip, 'far')

    noise = camera.getElementsByTagName('noise')[0]
    get_elem_by_tag_name(noise, 'type')
    get_elem_by_tag_name(noise, 'mean')
    get_elem_by_tag_name(noise, 'stddev')


# #}


# #{ check_required_lidar_tags(sensor)
def check_required_lidar_tags(sensor):
    pose_str = get_elem_by_tag_name(sensor, 'pose')
    if not check_str_to_pose(pose_str):
        raise AssertionError(f"The <pose> tag should have 6 elements.")

    get_elem_by_tag_name(sensor, 'topic')
    get_elem_by_tag_name(sensor, 'update_rate')
    get_elem_by_tag_name(sensor, 'gz_frame_id')
    get_elem_by_tag_name(sensor, 'gz_pointcloud_topic')
    get_elem_by_tag_name(sensor, 'ros_pointcloud_topic')

    ray = sensor.getElementsByTagName('ray')[0]

    scan = ray.getElementsByTagName('scan')[0]
    for direct in ['horizontal', 'vertical']:
        direction = scan.getElementsByTagName(direct)[0]
        get_elem_by_tag_name(direction, 'samples')
        get_elem_by_tag_name(direction, 'resolution')
        get_elem_by_tag_name(direction, 'min_angle')
        get_elem_by_tag_name(direction, 'max_angle')

    range = ray.getElementsByTagName('range')[0]
    get_elem_by_tag_name(range, 'min')
    get_elem_by_tag_name(range, 'max')
    get_elem_by_tag_name(range, 'resolution')

    noise = ray.getElementsByTagName('noise')[0]
    get_elem_by_tag_name(noise, 'type')
    get_elem_by_tag_name(noise, 'mean')
    get_elem_by_tag_name(noise, 'stddev')


# #}


# #{ check_required_imu_tags(sensor)
def check_required_imu_tags(sensor):
    pose_str = get_elem_by_tag_name(sensor, 'pose')
    if not check_str_to_pose(pose_str):
        raise AssertionError(f"The <pose> tag should have 6 elements.")

    get_elem_by_tag_name(sensor, 'topic')
    get_elem_by_tag_name(sensor, 'update_rate')
    get_elem_by_tag_name(sensor, 'gz_frame_id')
    get_elem_by_tag_name(sensor, 'imu_gz_topic')
    get_elem_by_tag_name(sensor, 'imu_ros_topic')

    imu = sensor.getElementsByTagName('imu')[0]

    angular_vel = imu.getElementsByTagName('angular_velocity')[0]
    linear_acc = imu.getElementsByTagName('linear_acceleration')[0]

    # For angular velocity:
    for axis in ['x', 'y', 'z']:
        axis_elem = angular_vel.getElementsByTagName(axis)[0]
        noise = axis_elem.getElementsByTagName('noise')[0]
        get_elem_by_tag_name(noise, 'mean')
        get_elem_by_tag_name(noise, 'stddev')

    # For linear acceleration:
    for axis in ['x', 'y', 'z']:
        axis_elem = linear_acc.getElementsByTagName(axis)[0]
        noise = axis_elem.getElementsByTagName('noise')[0]
        get_elem_by_tag_name(noise, 'mean')
        get_elem_by_tag_name(noise, 'stddev')


# #}


# #{ check_required_lidar_tags(sensor)
def check_required_rangefinder_tags(sensor):
    pose_str = get_elem_by_tag_name(sensor, 'pose')
    if not check_str_to_pose(pose_str):
        raise AssertionError(f"The <pose> tag should have 6 elements.")

    get_elem_by_tag_name(sensor, 'update_rate')

    lidar = sensor.getElementsByTagName('lidar')[0]

    scan = lidar.getElementsByTagName('scan')[0]
    for direct in ['horizontal', 'vertical']:
        direction = scan.getElementsByTagName(direct)[0]
        get_elem_by_tag_name(direction, 'samples')
        get_elem_by_tag_name(direction, 'resolution')
        get_elem_by_tag_name(direction, 'min_angle')
        get_elem_by_tag_name(direction, 'max_angle')

    range = lidar.getElementsByTagName('range')[0]
    get_elem_by_tag_name(range, 'min')
    get_elem_by_tag_name(range, 'max')
    get_elem_by_tag_name(range, 'resolution')


# #}
