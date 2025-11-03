from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace

def launch_image_bridge(context, *args, **kwargs):
    """Dynamically create ros_gz_image bridge node with split topic arguments."""
    topics_str = LaunchConfiguration('ros_gz_image_topics').perform(context).strip()

    if not topics_str:
        return []  # Correctly returns an empty list if no topics

    topics = topics_str.split()
    
    # Get the string value of the debug argument
    bridge_debug_str = LaunchConfiguration('bridge_debug').perform(context)
    
    bridge_log_level = PythonExpression([
        "'debug' if \"", bridge_debug_str, "\" == 'true' else 'info'"
    ])

    image_bridge_node = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=topics + [ 
            '--ros-args', '--log-level', bridge_log_level,
        ],
        parameters=[
            {'qos_history': 'keep_last'},
            {'qos_reliability': 'best_effort'},
        ],
        output='screen'
    )
    return [image_bridge_node]


def generate_launch_description():
    # Launch Configurations
    namespace = LaunchConfiguration('namespace')
    ros_gz_bridge_config = LaunchConfiguration('ros_gz_bridge_config')
    ros_gz_image_topics = LaunchConfiguration('ros_gz_image_topics')
    bridge_debug = LaunchConfiguration('bridge_debug')

    # Declare Launch Arguments
    declare_namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for all launched nodes.'
    )

    declare_ros_gz_bridge_config_arg = DeclareLaunchArgument(
        'ros_gz_bridge_config',
        description='Configuration file for the ros_gz_bridge.'
    )

    declare_ros_gz_image_topics_arg = DeclareLaunchArgument(
        'ros_gz_image_topics',
        default_value='',
        description='Space-separated list of Gazebo image topics to bridge.'
    )

    declare_bridge_debug_arg = DeclareLaunchArgument(
        'bridge_debug',
        default_value='false',
        description='Run bridges with debug log level.'
    )

    # Bridge log level expression
    bridge_log_level = PythonExpression([
    "'debug' if \"", bridge_debug, "\" == 'true' else 'info'"
    ])

    # ros_gz_bridge node
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p', ['config_file:=', ros_gz_bridge_config],
            '--log-level', bridge_log_level,
        ],
        output='screen'
    )

    # ros_gz_image bridge node (via OpaqueFunction to split topics)
    image_bridge_action = OpaqueFunction(function=launch_image_bridge)

    # Namespace group
    namespace_group = GroupAction([
        PushRosNamespace(namespace),
        ros_gz_bridge,
        image_bridge_action,
    ])

    return LaunchDescription([
        declare_namespace_arg,
        declare_ros_gz_bridge_config_arg,
        declare_ros_gz_image_topics_arg,
        declare_bridge_debug_arg,
        namespace_group
    ])
