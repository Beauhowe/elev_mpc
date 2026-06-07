from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    image_name = PathJoinSubstitution([
        FindPackageShare('convex_plane_decomposition_ros'),
        'data',
        'elevationMap',
    ])

    return LaunchDescription([
        Node(
            package='convex_plane_decomposition_ros',
            executable='convex_plane_decomposition_ros_save_elevationmap',
            name='save_elevation_map_to_image',
            output='screen',
            parameters=[{
                'frequency': 0.1,
                'elevation_topic': '/elevation_mapping/elevation_map_raw',
                'height_layer': 'elevation',
                'imageName': image_name,
            }],
        ),
    ])
