from ultralytics import YOLO
import cv2
import torch
import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Check if CUDA is available
print(torch.cuda.is_available())

# Load the YOLOv8 model
model = YOLO('scanDockDeliver.pt')  # Replace with your model path
model.to('cuda')  # Ensure the model uses GPU

# Get the class names
class_names = model.names  # This returns a dictionary where the key is the index and value is the class name

# Find the index of the 'matrix' class
matrix_class_index = [idx for idx, name in class_names.items() if name == 'matrix']

print(f"Index for 'matrix' class: {matrix_class_index}")

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
        # Convert from RGB to BGR for correct color display in OpenCV
        current_frame_bgr = cv2.cvtColor(current_frame, cv2.COLOR_RGB2BGR)

        # Perform inference using the YOLOv8 model
        results = model.predict(source=current_frame_bgr, device='cuda', save=False, classes=matrix_class_index, conf=0.2)
        time.sleep(1)
        # Display results on the image
        annotated_image = results[0].plot()  # Draw bounding boxes and labels

        # Create a window that allows for resizing
        cv2.namedWindow('Annotated Image', cv2.WINDOW_NORMAL)

        # Show the image with bounding boxes
        cv2.imshow("Annotated Image", annotated_image)

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
