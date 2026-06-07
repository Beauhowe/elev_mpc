from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    parameter_file = LaunchConfiguration('parameter_file')
    node_parameter_file = LaunchConfiguration('node_parameter_file')
    elevation_topic = LaunchConfiguration('elevation_topic')
    height_layer = LaunchConfiguration('height_layer')
    target_frame_id = LaunchConfiguration('target_frame_id')
    use_sim_time = LaunchConfiguration('use_sim_time')

    default_parameter_file = PathJoinSubstitution([
        FindPackageShare('convex_plane_decomposition_ros'),
        'config',
        'parameters.yaml',
    ])
    default_node_parameter_file = PathJoinSubstitution([
        FindPackageShare('convex_plane_decomposition_ros'),
        'config',
        'node.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('parameter_file', default_value=default_parameter_file),
        DeclareLaunchArgument('node_parameter_file', default_value=default_node_parameter_file),
        DeclareLaunchArgument('elevation_topic', default_value='/elevation_mapping/elevation_map_raw'),
        DeclareLaunchArgument('height_layer', default_value='elevation'),
        DeclareLaunchArgument('target_frame_id', default_value='odom'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='convex_plane_decomposition_ros',
            executable='convex_plane_decomposition_ros_node',
            name='convex_plane_decomposition_ros',
            output='screen',
            parameters=[
                parameter_file,
                node_parameter_file,
                {
                    'elevation_topic': elevation_topic,
                    'height_layer': height_layer,
                    'target_frame_id': target_frame_id,
                    'use_sim_time': use_sim_time,
                },
            ],
            # Node stays at / so node.yaml parameters load; remap publishes where PerceptiveController subscribes.
            remappings=[
                ('planar_terrain', '/convex_plane_decomposition_ros/planar_terrain'),
            ],
        ),
    ])
