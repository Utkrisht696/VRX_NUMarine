import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def launch(context, *args, **kwargs): []
    # return [

    #     Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         name='static_transform_publisher_imu',
    #         arguments=['--x', '0', '--y', '0', '--z', '2', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'wamv/wamv/base_link', '--child-frame-id', 'wamv/sensors/imu/imu/data'],
    #         output='screen'
    #     ),
    #     Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         name='static_transform_publisher_gps',
    #         arguments=['--x', '0', '--y', '0', '--z', '2', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'wamv/wamv/base_link', '--child-frame-id', 'wamv/sensors/gps/gps/fix'],
    #         output='screen'
    #     )
    # ]

def generate_launch_description():
    pkg_vrx_gazebo = get_package_share_directory('vrx_gazebo')

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_vrx_gazebo, 'launch', 'rviz.launch.py')
        ))

    return LaunchDescription([
        OpaqueFunction(function=launch),
        rviz
    ])
