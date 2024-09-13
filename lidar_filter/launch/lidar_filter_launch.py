from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_filter',
            executable='lidar_filter_node',
            name='lidar_filter_node',
            parameters=[PathJoinSubstitution([
                FindPackageShare('lidar_filter'),
                'config',
                'params.yaml'
            ])]
        ),
    ])
