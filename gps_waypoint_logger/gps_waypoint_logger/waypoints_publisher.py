import rclpy
from rclpy.node import Node
import yaml
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Header
from ament_index_python.packages import get_package_share_directory
import os
import math

class GPSWaypointPublisher(Node):
    def __init__(self):
        super().__init__('gps_waypoint_publisher')
        self.publisher_ = self.create_publisher(PoseArray, '/vrx/wayfinding/waypoints', 10)
        self.timer = self.create_timer(2.0, self.publish_waypoints)
        self.waypoints = self.load_waypoints()

    def load_waypoints(self):
        package_share_directory = get_package_share_directory('gps_waypoint_logger')
        waypoints_file = os.path.join(package_share_directory, 'config', 'gps_waypoints.yaml')
        with open(waypoints_file, 'r') as file:
            return yaml.safe_load(file)['waypoints']

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return [qx, qy, qz, qw]

    def publish_waypoints(self):
        pose_array_msg = PoseArray()
        pose_array_msg.header = Header()
        pose_array_msg.header.stamp = self.get_clock().now().to_msg()
        pose_array_msg.header.frame_id = 'map'  # Adjust frame_id as needed

        for waypoint in self.waypoints:
            pose = Pose()
            pose.position.x = waypoint['latitude']
            pose.position.y = waypoint['longitude']
            pose.position.z = 0.0  # Set Z position if needed

            # Convert yaw (Euler angle) to quaternion
            quaternion = self.euler_to_quaternion(0, 0, waypoint['yaw'])
            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]

            pose_array_msg.poses.append(pose)

        self.publisher_.publish(pose_array_msg)
        self.get_logger().info('Published pose array with {} waypoints'.format(len(self.waypoints)))

def main(args=None):
    rclpy.init(args=args)
    node = GPSWaypointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
