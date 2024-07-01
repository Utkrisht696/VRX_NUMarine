from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Paths to parameter files
    config_dir = FindPackageShare(package='my_navigation_pkg').find('my_navigation_pkg') + '/config/'
    nav2_params_path = os.path.join(config_dir, 'nav2_params.yaml')
    mppi_params_path = os.path.join(config_dir, 'mppi_params.yaml')

    # Path to the bringup launch file
    bringup_launch_file = os.path.join(
        FindPackageShare('nav2_bringup').find('nav2_bringup'),
        'launch',
        'bringup_launch.py'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'),

        Node(
            package='my_navigation_pkg',
            executable='gps_to_ned_node',
            name='gps_to_ned_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch_file),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav2_params_path,
                'autostart': 'true'
            }.items()
        )
    ])
