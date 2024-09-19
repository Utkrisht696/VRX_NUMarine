import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray, Pose
from sensor_msgs.msg import NavSatFix
import math

# Earth's radius in meters
EARTH_RADIUS = 6371000

def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.
    :param roll: Rotation around the x-axis in radians.
    :param pitch: Rotation around the y-axis in radians.
    :param yaw: Rotation around the z-axis in radians.
    :return: A list containing the x, y, z, and w of the quaternion.
    """
    qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
    qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
    qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    return [qx, qy, qz, qw]

def haversine(lat1, lon1, lat2, lon2):
    """
    Calculate the great-circle distance between two points on the Earth's surface using the Haversine formula.
    :param lat1: Latitude of point 1
    :param lon1: Longitude of point 1
    :param lat2: Latitude of point 2
    :param lon2: Longitude of point 2
    :return: Distance between the two points in meters
    """
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat / 2) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return EARTH_RADIUS * c

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')

        # Subscribe to the buoy state
        self.buoy_state_subscriber = self.create_subscription(
            String,
            'buoy_state',
            self.buoy_state_callback,
            10
        )

        # Subscribe to the boat's GPS position
        self.boat_position_subscriber = self.create_subscription(
            NavSatFix,
            '/wamv/sensors/gps/gps/fix',
            self.boat_position_callback,
            10
        )

        # Publisher for waypoints
        self.waypoint_publisher = self.create_publisher(PoseArray, '/vrx/wayfinding/waypoints', 10)

        # Initial buoy position (replace with actual buoy lat/lon if needed)
        self.buoy_position = (-33.72268264867066, 150.67419239401727)

        # Radius of the circle for waypoints (in meters)
        self.radius = 10.0

        # Flag to track if waypoints have been published
        self.waypoints_published = False

        # Timer for publishing waypoints every 2 seconds
        self.timer = self.create_timer(2.0, self.publish_waypoints)

        # Current boat position (initialized to None)
        self.boat_position = None

        # Store waypoints once ordered
        self.ordered_waypoints = None

        self.get_logger().info("Waiting for buoy state to turn green...")

    def buoy_state_callback(self, msg):
        if msg.data == 'green' and not self.waypoints_published and self.boat_position is not None:
            self.order_waypoints_once()
            self.waypoints_published = True
            self.get_logger().info("Buoy is green, waypoints will be published every 2 seconds.")
        elif msg.data != 'green':
            self.waypoints_published = False

    def boat_position_callback(self, msg):
        # Update the boat's current position from the GPS sensor
        self.boat_position = (msg.latitude, msg.longitude)
        self.get_logger().info(f"Received boat position: lat={msg.latitude}, lon={msg.longitude}")

    def order_waypoints_once(self):
        # Create waypoints around the buoy in a clockwise direction
        lat_buoy, lon_buoy = self.buoy_position
        num_waypoints = 13
        angle_increment = 2 * math.pi / num_waypoints
        lat_radius = self.radius / EARTH_RADIUS * (180 / math.pi)
        lon_radius = self.radius / (EARTH_RADIUS * math.cos(math.radians(lat_buoy))) * (180 / math.pi)

        waypoints = []
        for i in range(num_waypoints):
            # Create clockwise waypoints by adjusting the angle increment
            angle = -i * angle_increment
            delta_lat = lat_radius * math.sin(angle)
            delta_lon = lon_radius * math.cos(angle)

            lat = lat_buoy + delta_lat
            lon = lon_buoy + delta_lon

            waypoint = Pose()
            waypoint.position.x = lat  # Latitude as x
            waypoint.position.y = lon  # Longitude as y
            waypoint.position.z = 0.0  # Waypoints on a 2D plane

            waypoints.append(waypoint)

        # Find the waypoint closest to the boat's position
        closest_index = None
        min_distance = float('inf')

        for i, waypoint in enumerate(waypoints):
            distance = haversine(waypoint.position.x, waypoint.position.y, self.boat_position[0], self.boat_position[1])
            if distance < min_distance:
                min_distance = distance
                closest_index = i

        # Reorder waypoints so the closest waypoint is first
        self.ordered_waypoints = waypoints[closest_index:] + waypoints[:closest_index]

        # Append the first waypoint as the last waypoint to close the loop
        self.ordered_waypoints.append(self.ordered_waypoints[0])

        # Set orientation based on the direction to the next waypoint (consistent yaw changes)
        for i in range(len(self.ordered_waypoints) - 1):
            current_pose = self.ordered_waypoints[i]
            next_pose = self.ordered_waypoints[i + 1]

            delta_lat = next_pose.position.x - current_pose.position.x
            delta_lon = next_pose.position.y - current_pose.position.y
            yaw = math.atan2(delta_lat, delta_lon)  # Heading towards the next waypoint

            quaternion = euler_to_quaternion(0, 0, yaw)
            current_pose.orientation.x = quaternion[0]
            current_pose.orientation.y = quaternion[1]
            current_pose.orientation.z = quaternion[2]
            current_pose.orientation.w = quaternion[3]

        self.get_logger().info(f"Ordered waypoints with closest to boat at index {closest_index}. Loop closes on this waypoint.")

    def publish_waypoints(self):
        if not self.waypoints_published or self.ordered_waypoints is None:
            return

        # Create a PoseArray message from the ordered waypoints
        pose_array = PoseArray()
        pose_array.poses = self.ordered_waypoints

        # Publish the PoseArray
        self.waypoint_publisher.publish(pose_array)
        self.get_logger().info(f"Published waypoints around the buoy in fixed order.")

def main(args=None):
    rclpy.init(args=args)
    waypoint_publisher = WaypointPublisher()
    rclpy.spin(waypoint_publisher)

    # Shutdown
    waypoint_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
