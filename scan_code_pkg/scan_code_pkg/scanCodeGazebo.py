from ultralytics import YOLO
import cv2
import torch
import rclpy
import time
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import logging
from std_msgs.msg import String

# Configure logging to silence YOLO's output
logging.getLogger("ultralytics").setLevel(logging.ERROR)
logging.getLogger("pytorch_lightning").setLevel(logging.ERROR)

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

# Helper functions for color detection
def get_color_mask(hsv_roi, color):
    """Returns a mask for the given color in the HSV color space."""
    if color == 'red':
        lower_bound = np.array([0, 100, 100])
        upper_bound = np.array([10, 255, 255])
        lower_bound2 = np.array([170, 100, 100])
        upper_bound2 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv_roi, lower_bound, upper_bound)
        mask2 = cv2.inRange(hsv_roi, lower_bound2, upper_bound2)
        mask = mask1 | mask2
    elif color == 'green':
        lower_bound = np.array([36, 50, 50])
        upper_bound = np.array([89, 255, 255])
        mask = cv2.inRange(hsv_roi, lower_bound, upper_bound)
    elif color == 'blue':
        lower_bound = np.array([94, 80, 50])
        upper_bound = np.array([126, 255, 255])
        mask = cv2.inRange(hsv_roi, lower_bound, upper_bound)
    else:
        mask = None
    return mask

def get_predominant_color(roi):
    """Detects and returns the predominant color in the given ROI."""
    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    
    red_mask = get_color_mask(hsv_roi, 'red')
    green_mask = get_color_mask(hsv_roi, 'green')
    blue_mask = get_color_mask(hsv_roi, 'blue')
    
    red_count = cv2.countNonZero(red_mask)
    green_count = cv2.countNonZero(green_mask)
    blue_count = cv2.countNonZero(blue_mask)
    
    pixel_threshold = 200  # Adjust this value as needed
    
    if red_count > green_count and red_count > blue_count and red_count >= pixel_threshold:
        return 'Red', red_count
    elif green_count > red_count and green_count > blue_count and green_count >= pixel_threshold:
        return 'Green', green_count
    elif blue_count > red_count and blue_count > green_count and blue_count >= pixel_threshold:
        return 'Blue', blue_count
    else:
        return 'None', 0

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
        
        # Create a publisher for detected color sequence
        self.color_publisher = self.create_publisher(String, 'code_sequence', 10)
        
        # Initialize tracking variables
        self.detected_colors = []
        self.last_detected_color = None
        self.color_index = 1

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        current_frame = self.br.imgmsg_to_cv2(msg)
        # Convert from RGB to BGR for correct color display in OpenCV
        current_frame_bgr = cv2.cvtColor(current_frame, cv2.COLOR_RGB2BGR)

        # Perform inference using the YOLOv8 model
        results = model.predict(source=current_frame_bgr, device='cuda', save=False, classes=matrix_class_index, conf=0.2)
        time.sleep(1)
        
        # Get the bounding box results
        boxes = results[0].boxes.xyxy.cpu().numpy()  # xyxy format for bounding boxes
        detected_colors = []
        for box in boxes:
            x1, y1, x2, y2 = map(int, box[:4])  # Extract bounding box coordinates

            # Extract the Region of Interest (ROI) for color detection
            roi = current_frame_bgr[y1:y2, x1:x2]

            # Detect the predominant color within the bounding box
            detected_color, pixel_count = get_predominant_color(roi)
            
            if detected_color == 'None':
                # If no color is detected, reset the sequence
                self.detected_colors = []
                self.color_index = 1
                self.last_detected_color = None
                continue  # Skip to next bounding box if applicable
            
            # Avoid reporting the same color again
            if detected_color != self.last_detected_color:
                # Add the detected color to the list if not already present
                if detected_color not in self.detected_colors:
                    self.detected_colors.append(detected_color)
                    # Publish the detected color with index
                    color_msg = String()
                    color_msg.data = f"{self.color_index}: {detected_color}"
                    self.color_publisher.publish(color_msg)
                    print(f"Published: {self.color_index}: {detected_color}")
                    self.color_index += 1
                self.last_detected_color = detected_color

            # Draw bounding box and display detected color
            cv2.rectangle(current_frame_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"{detected_color}: {pixel_count} pixels"
            cv2.putText(current_frame_bgr, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        # Create a window that allows for resizing
        cv2.namedWindow('Annotated Image', cv2.WINDOW_NORMAL)

        # Show the image with bounding boxes and color labels
        cv2.imshow("Annotated Image", current_frame_bgr)

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
