import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directories
    pkg_vrx_gazebo = get_package_share_directory('vrx_gazebo')
    pkg_wamv_description = get_package_share_directory('wamv_description')

    # Define the path to the URDF file
    urdf_file = os.path.join(pkg_wamv_description, 'urdf', 'wamv_target.urdf')

    # Include the RViz launch file
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_vrx_gazebo, 'launch', 'rviz.launch.py')
        )
    )

    # Define the robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_file).read()}]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz
    ])

if __name__ == '__main__':
    generate_launch_description()
