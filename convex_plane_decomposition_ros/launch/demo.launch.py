from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    datafile = LaunchConfiguration('datafile')
    max_height = LaunchConfiguration('max_height')
    package_share = FindPackageShare('convex_plane_decomposition_ros')

    demo_node_file = PathJoinSubstitution([package_share, 'config', 'demo_node.yaml'])
    parameter_file = PathJoinSubstitution([package_share, 'config', 'parameters.yaml'])
    decomposition_launch = PathJoinSubstitution([package_share, 'launch', 'convex_plane_decomposition.launch.py'])
    image_path = PathJoinSubstitution([package_share, 'data', datafile])
    rviz_config = PathJoinSubstitution([package_share, 'rviz', 'config_demo.rviz'])

    return LaunchDescription([
        DeclareLaunchArgument('datafile', default_value='terrain.png'),
        DeclareLaunchArgument('max_height', default_value='1.0'),
        Node(
            package='grid_map_demos',
            executable='image_publisher.py',
            name='image_publisher',
            output='screen',
            parameters=[{
                'image_path': image_path,
                'topic': '/image',
            }],
        ),
        Node(
            package='grid_map_demos',
            executable='image_to_gridmap_demo',
            name='image_to_gridmap_demo',
            output='screen',
            parameters=[{
                'image_topic': '/image',
                'min_height': 0.0,
                'max_height': max_height,
                'resolution': 0.04,
            }],
        ),
        Node(
            package='convex_plane_decomposition_ros',
            executable='convex_plane_decomposition_ros_add_noise',
            name='convex_plane_decomposition_ros_add_noise',
            output='screen',
            parameters=[{
                'noiseGauss': 0.01,
                'noiseUniform': 0.01,
                'outlier_percentage': 5.0,
                'blur': False,
                'frequency': 30.0,
                'elevation_topic_in': '/image_to_gridmap_demo/grid_map',
                'elevation_topic_out': '/elevation_mapping/elevation_map_raw',
                'height_layer': 'elevation',
            }],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map2odom_broadcaster',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom'],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(decomposition_launch),
            launch_arguments={
                'parameter_file': parameter_file,
                'node_parameter_file': demo_node_file,
            }.items(),
        ),
        Node(
            package='convex_plane_decomposition_ros',
            executable='convex_plane_decomposition_ros_approximation_demo_node',
            name='convex_plane_decomposition_ros_approximation_demo_node',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
        ),
    ])
