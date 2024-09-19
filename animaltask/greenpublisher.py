import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class BuoyStatePublisher(Node):
    def __init__(self):
        super().__init__('buoy_state_publisher')

        # Create a publisher for the 'buoy_state' topic
        self.buoy_state_publisher = self.create_publisher(String, 'buoy_state', 10)

        # Set a timer to publish the state every second
        self.timer = self.create_timer(1.0, self.publish_buoy_state)

        self.get_logger().info("Buoy state publisher started, publishing 'green' to 'buoy_state'...")

    def publish_buoy_state(self):
        # Create the message with the state 'green'
        state_msg = String()
        state_msg.data = 'green'

        # Publish the message
        self.buoy_state_publisher.publish(state_msg)
        self.get_logger().info("Published 'green' to 'buoy_state' topic.")

def main(args=None):
    rclpy.init(args=args)
    buoy_state_publisher = BuoyStatePublisher()

    # Spin the node to keep it active
    rclpy.spin(buoy_state_publisher)

    # Shutdown once done
    buoy_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
