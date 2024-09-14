import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        # Subscribe to the camera image topic
        self.subscription = self.create_subscription(
            Image,
            '/wamv/sensors/cameras/front_left_camera_sensor/image_raw',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        # Initialize CvBridge to convert ROS Image messages to OpenCV format
        self.br = CvBridge()

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        current_frame = self.br.imgmsg_to_cv2(msg)

        # Display the image using OpenCV
        cv2.imshow("Camera Image", current_frame)

        # Add waitKey to allow OpenCV to process UI events
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()

    # Spin the node so the callback is called when messages are received
    rclpy.spin(image_subscriber)

    # Shutdown the ROS client library for Python
    rclpy.shutdown()

if __name__ == '__main__':
    main()
