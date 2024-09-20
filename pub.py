import rclpy
from rclpy.node import Node
import yaml
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Header
from ament_index_python.packages import get_package_share_directory
import os

class GPSWaypointPublisher(Node):
    def __init__(self):
        super().__init__('gps_waypoint_publisher')
        self.publisher_ = self.create_publisher(PoseArray, '/vrx/wayfinding/waypoints', 10)
        self.timer = self.create_timer(2.0, self.publish_waypoints)
        self.pose_array_msg = self.load_waypoints()

    def load_waypoints(self):
        """
        Load the PoseArray from the YAML file.
        """
        package_share_directory = get_package_share_directory('gps_waypoint_logger')
        waypoints_file = os.path.join(package_share_directory, 'config', 'gps_waypoints.yaml')
        with open(waypoints_file, 'r') as file:
            pose_array_data = yaml.safe_load(file)

        # Create a PoseArray message and populate it with the loaded poses
        pose_array_msg = PoseArray()
        pose_array_msg.header = Header()
        pose_array_msg.header.frame_id = ''  # Adjust frame_id as needed

        for pose_data in pose_array_data['poses']:
            pose = self.pose_from_yaml(pose_data)
            pose_array_msg.poses.append(pose)

        return pose_array_msg

    def pose_from_yaml(self, pose_data):
        """
        Convert the pose data from YAML format to a Pose message.
        """
        from geometry_msgs.msg import Pose

        pose = Pose()
        pose.position.x = pose_data['position']['x']
        pose.position.y = pose_data['position']['y']
        pose.position.z = pose_data['position']['z']
        pose.orientation.x = pose_data['orientation']['x']
        pose.orientation.y = pose_data['orientation']['y']
        pose.orientation.z = pose_data['orientation']['z']
        pose.orientation.w = pose_data['orientation']['w']

        return pose

    def publish_waypoints(self):
        """
        Publish the PoseArray loaded from the YAML file.
        """
        # Update the timestamp before publishing
        self.pose_array_msg.header.stamp = self.get_clock().now().to_msg()

        # Publish the pose array message
        self.publisher_.publish(self.pose_array_msg)
        self.get_logger().info('Published pose array with {} waypoints'.format(len(self.pose_array_msg.poses)))

def main(args=None):
    rclpy.init(args=args)
    node = GPSWaypointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()