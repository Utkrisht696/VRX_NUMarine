# import numpy as np
# import cv2
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image, PointCloud2
# from cv_bridge import CvBridge
# import sensor_msgs_py.point_cloud2 as pc2

# class CameraLidarFusion(Node):
#     def __init__(self):
#         super().__init__('camera_lidar_fusion_node')

#         # Camera intrinsic parameters from the image you provided
#         self.camera_matrix = np.array([[761.7297, 0, 640.3348],
#                                        [0, 762.2555, 362.3959],
#                                        [0, 0, 1.0000]])

#         # Distortion coefficients (radial distortion, padded with zeros for tangential distortion)
#         self.dist_coeffs = np.array([-0.0079, -6.2538e-04, 0, 0, 0])  # k1, k2, p1, p2, k3 (tangential terms are zeros)

#         # Extrinsic parameters (rotation and translation)
#         self.rotation_matrix = np.array([[-0.0021, -0.9988, 0.0486],
#                                          [-0.2391, -0.0467, -0.9699],
#                                          [0.9710, -0.0137, -0.2387]])

#         self.translation_vector = np.array([0.1116, -0.3260, -0.1386])


#         self.extrinsicMatrix = np.array([[-0.0021, -0.9988, 0.0486,0.1116],
#                                          [-0.2391, -0.0467, -0.9699,-0.3260],
#                                          [0.9710, -0.0137, -0.2387,-0.1386],
#                                          [0     ,    0     ,0  ,        1]  
#                                          ])

#         self.bridge = CvBridge()
#         self.image_undistorted = None  # Initialize the undistorted image

#         # Set up ROS subscribers
#         self.camera_sub = self.create_subscription(Image, '/wamv/sensors/cameras/front_left_camera_sensor/image_raw', self.image_callback, 10)
#         self.lidar_sub = self.create_subscription(PointCloud2, '/wamv/sensors/lidars/lidar_wamv_sensor/points', self.lidar_callback, 10)

#         self.image_frame = None  # Store the latest image frame for use with LiDAR data

#     def image_callback(self, msg):
#         try:
#             # Convert ROS image to OpenCV image
#             self.image_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

#             # Undistort the image using the camera intrinsic parameters (intrinsics)
#             self.image_undistorted = cv2.undistort(self.image_frame, self.camera_matrix, self.dist_coeffs)

#             # Show undistorted image
#             cv2.imshow('Undistorted Image', self.image_undistorted)
#             cv2.waitKey(1)
#         except Exception as e:
#             self.get_logger().error(f"Error in image callback: {e}")

#     def lidar_callback(self, msg):
#         try:
#             if self.image_frame is None or self.image_undistorted is None:
#                 return  # Wait for the first image frame to arrive and undistorted image to be set

#             # Convert ROS PointCloud2 to XYZ data
#             pc_data = []
#             for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
#                 pc_data.append([point[0], point[1], point[2]])

#             pc_data = np.array(pc_data)

#             # Filter the LiDAR point cloud (within 10 meters)
#             distances = np.linalg.norm(pc_data, axis=1)
#             valid_indices = np.logical_and(distances <= 10, np.isfinite(pc_data).all(axis=1))
#             pt_cloud_data_filtered = pc_data[valid_indices]

#             # Project the filtered LiDAR points onto the camera image
#             self.project_lidar_to_image(pt_cloud_data_filtered)

#         except Exception as e:
#             self.get_logger().error(f"Error in LiDAR callback: {e}")

#     def project_lidar_to_image(self, lidar_points):
#         # Convert LiDAR points to homogeneous coordinates (add 1 for the translation part)
#         lidar_points_homogeneous = np.hstack((lidar_points, np.ones((lidar_points.shape[0], 1))))

#         # Apply the extrinsic transformation (rotation and translation)
#         # extrinsic_matrix = np.hstack((self.rotation_matrix, self.translation_vector.reshape(3, 1)))
#         camera_points = lidar_points_homogeneous @ self.extrinsicMatrix.T  # Transform LiDAR points to camera frame

#         # Keep only points in front of the camera (positive Z in camera coordinates)
#         valid_indices = camera_points[:, 2] > 0
#         camera_points = camera_points[valid_indices]

#         # Project the 3D points in the camera frame to the 2D image plane using the intrinsic matrix
#         image_points = (self.camera_matrix @ camera_points[:, :3].T).T

#         # Normalize homogeneous coordinates to get pixel coordinates
#         image_points[:, 0] /= image_points[:, 2]
#         image_points[:, 1] /= image_points[:, 2]

#         # Only keep the 2D (x, y) coordinates
#         image_points = image_points[:, :2]

#         # Draw the projected points onto the undistorted image
#         for point in image_points:
#             cv2.circle(self.image_undistorted, (int(point[0]), int(point[1])), 2, (0, 0, 255), -1)

#         # Display the result with the LiDAR points projected on the camera image
#         cv2.imshow('LiDAR Projected to Image', self.image_undistorted)
#         cv2.waitKey(1)

# def main(args=None):
#     rclpy.init(args=args)
#     node = CameraLidarFusion()
    
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()


#########################

# import numpy as np
# import cv2
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image, PointCloud2
# from cv_bridge import CvBridge
# import sensor_msgs_py.point_cloud2 as pc2
# from ultralytics import YOLO
# import time
# import logging

# # Suppress YOLOv8 logging
# logging.getLogger("ultralytics").setLevel(logging.ERROR)

# class CameraLidarFusion(Node):
#     def __init__(self):
#         super().__init__('camera_lidar_fusion_node')

#         # Load YOLOv8 model (pretrained) with verbose disabled
#         self.model = YOLO("yolov8n.pt", verbose=False)  # YOLOv8n model

#         # Camera intrinsic parameters from the image you provided
#         self.camera_matrix = np.array([[761.7297, 0, 640.3348],
#                                        [0, 762.2555, 362.3959],
#                                        [0, 0, 1.0000]])

#         # Distortion coefficients (radial distortion, padded with zeros for tangential distortion)
#         self.dist_coeffs = np.array([-0.0079, -6.2538e-04, 0, 0, 0])

#         # Extrinsic parameters (rotation and translation)
#         self.rotation_matrix = np.array([[-0.0021, -0.9988, 0.0486],
#                                          [-0.2391, -0.0467, -0.9699],
#                                          [0.9710, -0.0137, -0.2387]])

#         self.translation_vector = np.array([0.1116, -0.3260, -0.1386])

#         self.bridge = CvBridge()
#         self.image_undistorted = None  # Initialize the undistorted image
#         self.bounding_boxes = []  # Initialize bounding boxes to avoid access before detection
#         self.labels = []  # Initialize labels
#         self.confidences = []  # Initialize confidences

#         # Set up ROS subscribers
#         self.camera_sub = self.create_subscription(Image, '/wamv/sensors/cameras/front_left_camera_sensor/image_raw', self.image_callback, 10)
#         self.lidar_sub = self.create_subscription(PointCloud2, '/wamv/sensors/lidars/lidar_wamv_sensor/points', self.lidar_callback, 10)

#         self.image_frame = None  # Store the latest image frame for use with LiDAR data

#     def image_callback(self, msg):
#         try:
#             # Convert ROS image to OpenCV image
#             self.image_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

#             # Undistort the image using the camera intrinsic parameters (intrinsics)
#             self.image_undistorted = cv2.undistort(self.image_frame, self.camera_matrix, self.dist_coeffs)

#             # Run YOLOv8 object detection
#             results = self.model(self.image_undistorted)

#             # Ensure there are results and bounding boxes
#             if results and results[0].boxes is not None and len(results[0].boxes) > 0:
#                 # Filter results by confidence threshold (e.g., 0.5)
#                 valid_indices = results[0].boxes.conf.cpu().numpy() >= 0.1

#                 # Apply the confidence filter
#                 self.bounding_boxes = results[0].boxes.xyxy.cpu().numpy()[valid_indices]  # Get the bounding boxes (xmin, ymin, xmax, ymax)
#                 self.labels = [self.model.names[int(cls)] for cls in results[0].boxes.cls.cpu().numpy()[valid_indices]]
#                 self.confidences = results[0].boxes.conf.cpu().numpy()[valid_indices]

#         except Exception as e:
#             self.get_logger().error(f"Error in image callback: {e}")

#     def lidar_callback(self, msg):
#         try:
#             if self.image_frame is None or self.image_undistorted is None or len(self.bounding_boxes) == 0:
#                 return  # Wait for the first image frame to arrive and undistorted image to be set, and check if bounding boxes exist

#             # Convert ROS PointCloud2 to XYZ data
#             pc_data = []
#             for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
#                 pc_data.append([point[0], point[1], point[2]])

#             pc_data = np.array(pc_data)

#             # Filter the LiDAR point cloud (within 10 meters)
#             distances = np.linalg.norm(pc_data, axis=1)
#             valid_indices = np.logical_and(distances <= 10, np.isfinite(pc_data).all(axis=1))
#             pt_cloud_data_filtered = pc_data[valid_indices]

#             # Ensure that valid LiDAR data exists
#             if len(pt_cloud_data_filtered) == 0:
#                 return

#             # Project the filtered LiDAR points onto the camera image and get the distance for objects
#             self.project_lidar_to_image(pt_cloud_data_filtered)

#         except Exception as e:
#             self.get_logger().error(f"Error in LiDAR callback: {e}")

#     def project_lidar_to_image(self, lidar_points):
#         # Convert LiDAR points to homogeneous coordinates (add 1 for the translation part)
#         lidar_points_homogeneous = np.hstack((lidar_points, np.ones((lidar_points.shape[0], 1))))

#         # Apply the extrinsic transformation (rotation and translation)
#         extrinsic_matrix = np.hstack((self.rotation_matrix, self.translation_vector.reshape(3, 1)))
#         camera_points = lidar_points_homogeneous @ extrinsic_matrix.T  # Transform LiDAR points to camera frame

#         # Keep only points in front of the camera (positive Z in camera coordinates)
#         valid_indices = camera_points[:, 2] > 0
#         camera_points = camera_points[valid_indices]

#         # Project the 3D points in the camera frame to the 2D image plane using the intrinsic matrix
#         image_points = (self.camera_matrix @ camera_points[:, :3].T).T

#         # Normalize homogeneous coordinates to get pixel coordinates
#         image_points[:, 0] /= image_points[:, 2]
#         image_points[:, 1] /= image_points[:, 2]

#         # Calculate object distances based on bounding boxes
#         detected_objects_info = []
#         for i, bbox in enumerate(self.bounding_boxes):
#             xmin, ymin, xmax, ymax = map(int, bbox)
#             label = self.labels[i] if i < len(self.labels) else "Unknown"
#             confidence = self.confidences[i] if i < len(self.confidences) else 0

#             # Get LiDAR points within the bounding box in the image
#             in_bbox_indices = np.logical_and.reduce((
#                 image_points[:, 0] >= xmin,
#                 image_points[:, 0] <= xmax,
#                 image_points[:, 1] >= ymin,
#                 image_points[:, 1] <= ymax
#             ))

#             if np.any(in_bbox_indices):
#                 points_in_bbox = camera_points[in_bbox_indices]

#                 # Calculate the average distance for the object
#                 avg_distance = np.mean(np.linalg.norm(points_in_bbox[:, :3], axis=1))

#                 # Store object information
#                 detected_objects_info.append({
#                     "label": label,
#                     "bbox": [xmin, ymin, xmax, ymax],
#                     "distance": avg_distance,
#                     "confidence": confidence
#                 })

#                 # Draw bounding box, label, confidence, and distance on the image
#                 cv2.rectangle(self.image_undistorted, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
#                 text = f'{label} ({confidence*100:.1f}%): {avg_distance:.2f}m'
#                 cv2.putText(self.image_undistorted, text, 
#                             (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

#                 # Log detected object with distance and confidence
#                 self.get_logger().info(f'Detected {label}: Confidence = {confidence*100:.1f}%, Distance = {avg_distance:.2f}m')

#         # Display the result with YOLO, distances, and bounding boxes
#         cv2.imshow('YOLOv8 Detection with Distances and Confidence', self.image_undistorted)
#         cv2.waitKey(1)

#         # Slow down the output to 500ms
#         # time.sleep(0.1)

#         # Print useful information to the console
#         self.print_detected_objects(detected_objects_info)

#     def print_detected_objects(self, detected_objects_info):
#         if detected_objects_info:
#             print(f"Detected {len(detected_objects_info)} objects:")
#             for obj in detected_objects_info:
#                 print(f"Object: {obj['label']}, Confidence: {obj['confidence']*100:.1f}%, Distance: {obj['distance']:.2f}m, BBox: {obj['bbox']}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = CameraLidarFusion()
    
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()


#############################

# import numpy as np
# import cv2
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image, PointCloud2
# from cv_bridge import CvBridge
# import sensor_msgs_py.point_cloud2 as pc2
# from ultralytics import YOLO
# import time
# import logging

# # Suppress YOLOv8 logging
# logging.getLogger("ultralytics").setLevel(logging.ERROR)

# class CameraLidarFusion(Node):
#     def __init__(self):
#         super().__init__('camera_lidar_fusion_node')

#         # Load YOLOv8 model (pretrained) with verbose disabled
#         self.model = YOLO("yolov8n.pt", verbose=False)  # YOLOv8n model

#         # Camera intrinsic parameters from the image you provided
#         self.camera_matrix = np.array([[761.7297, 0, 640.3348],
#                                        [0, 762.2555, 362.3959],
#                                        [0, 0, 1.0000]])

#         # Distortion coefficients (radial distortion, padded with zeros for tangential distortion)
#         self.dist_coeffs = np.array([-0.0079, -6.2538e-04, 0, 0, 0])

#         # Extrinsic parameters (rotation and translation)
#         self.rotation_matrix = np.array([[-0.0021, -0.9988, 0.0486],
#                                          [-0.2391, -0.0467, -0.9699],
#                                          [0.9710, -0.0137, -0.2387]])

#         self.translation_vector = np.array([0.1116, -0.3260, -0.1386])

#         self.bridge = CvBridge()
#         self.image_undistorted = None  # Initialize the undistorted image
#         self.bounding_boxes = []  # Initialize bounding boxes to avoid access before detection
#         self.labels = []  # Initialize labels
#         self.confidences = []  # Initialize confidences

#         # Set up ROS subscribers
#         self.camera_sub = self.create_subscription(Image, '/wamv/sensors/cameras/front_left_camera_sensor/image_raw', self.image_callback, 10)
#         self.lidar_sub = self.create_subscription(PointCloud2, '/wamv/sensors/lidars/lidar_wamv_sensor/points', self.lidar_callback, 10)

#         self.image_frame = None  # Store the latest image frame for use with LiDAR data

#     def image_callback(self, msg):
#         try:
#             # Convert ROS image to OpenCV image
#             self.image_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

#             # Undistort the image using the camera intrinsic parameters (intrinsics)
#             self.image_undistorted = cv2.undistort(self.image_frame, self.camera_matrix, self.dist_coeffs)

#             # Run YOLOv8 object detection
#             results = self.model(self.image_undistorted)

#             # Ensure there are results and bounding boxes
#             if results and results[0].boxes is not None and len(results[0].boxes) > 0:
#                 # Filter results by confidence threshold (e.g., 0.5)
#                 valid_indices = results[0].boxes.conf.cpu().numpy() >= 0

#                 # Apply the confidence filter
#                 self.bounding_boxes = results[0].boxes.xyxy.cpu().numpy()[valid_indices]  # Get the bounding boxes (xmin, ymin, xmax, ymax)
#                 self.labels = [self.model.names[int(cls)] for cls in results[0].boxes.cls.cpu().numpy()[valid_indices]]
#                 self.confidences = results[0].boxes.conf.cpu().numpy()[valid_indices]

#         except Exception as e:
#             self.get_logger().error(f"Error in image callback: {e}")

#     def lidar_callback(self, msg):
#         try:
#             if self.image_frame is None or self.image_undistorted is None or len(self.bounding_boxes) == 0:
#                 return  # Wait for the first image frame to arrive and undistorted image to be set, and check if bounding boxes exist

#             # Convert ROS PointCloud2 to XYZ data
#             pc_data = []
#             for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
#                 pc_data.append([point[0], point[1], point[2]])

#             pc_data = np.array(pc_data)

#             # Filter the LiDAR point cloud (within 10 meters)
#             distances = np.linalg.norm(pc_data, axis=1)
#             valid_indices = np.logical_and(distances <= 10, np.isfinite(pc_data).all(axis=1))
#             pt_cloud_data_filtered = pc_data[valid_indices]

#             # Ensure that valid LiDAR data exists
#             if len(pt_cloud_data_filtered) == 0:
#                 return

#             # Project the filtered LiDAR points onto the camera image and get the position for objects
#             self.project_lidar_to_image(pt_cloud_data_filtered)

#         except Exception as e:
#             self.get_logger().error(f"Error in LiDAR callback: {e}")

#     def project_lidar_to_image(self, lidar_points):
#         # Convert LiDAR points to homogeneous coordinates (add 1 for the translation part)
#         lidar_points_homogeneous = np.hstack((lidar_points, np.ones((lidar_points.shape[0], 1))))

#         # Apply the extrinsic transformation (rotation and translation)
#         extrinsic_matrix = np.hstack((self.rotation_matrix, self.translation_vector.reshape(3, 1)))
#         camera_points = lidar_points_homogeneous @ extrinsic_matrix.T  # Transform LiDAR points to camera frame

#         # Keep only points in front of the camera (positive Z in camera coordinates)
#         valid_indices = camera_points[:, 2] > 0
#         camera_points = camera_points[valid_indices]

#         # Project the 3D points in the camera frame to the 2D image plane using the intrinsic matrix
#         image_points = (self.camera_matrix @ camera_points[:, :3].T).T

#         # Normalize homogeneous coordinates to get pixel coordinates
#         image_points[:, 0] /= image_points[:, 2]
#         image_points[:, 1] /= image_points[:, 2]

#         # Ensure points are within image bounds
#         height, width, _ = self.image_undistorted.shape
#         valid_image_points = np.logical_and.reduce((
#             image_points[:, 0] >= 0,
#             image_points[:, 0] < width,
#             image_points[:, 1] >= 0,
#             image_points[:, 1] < height
#         ))

#         # Draw LiDAR points on the image as small red circles
#         for i, valid in enumerate(valid_image_points):
#             if valid:
#                 x, y = int(image_points[i, 0]), int(image_points[i, 1])
#                 cv2.circle(self.image_undistorted, (x, y), 2, (0, 0, 255), -1)  # Red points for LiDAR projections

#         # Calculate object distances and positions based on bounding boxes
#         detected_objects_info = []
#         for i, bbox in enumerate(self.bounding_boxes):
#             xmin, ymin, xmax, ymax = map(int, bbox)
#             label = self.labels[i] if i < len(self.labels) else "Unknown"
#             confidence = self.confidences[i] if i < len(self.confidences) else 0

#             # Get LiDAR points within the bounding box in the image
#             in_bbox_indices = np.logical_and.reduce((
#                 image_points[:, 0] >= xmin,
#                 image_points[:, 0] <= xmax,
#                 image_points[:, 1] >= ymin,
#                 image_points[:, 1] <= ymax
#             ))

#             if np.any(in_bbox_indices):
#                 points_in_bbox = camera_points[in_bbox_indices]

#                 # Calculate the average distance and position for the object
#                 avg_distance = np.mean(np.linalg.norm(points_in_bbox[:, :3], axis=1))
#                 avg_position = np.mean(points_in_bbox[:, :3], axis=0)  # Average 3D position

#                 # Store object information
#                 detected_objects_info.append({
#                     "label": label,
#                     "bbox": [xmin, ymin, xmax, ymax],
#                     "distance": avg_distance,
#                     "position": avg_position,
#                     "confidence": confidence
#                 })

#                 # Draw bounding box, label, confidence, and distance on the image
#                 text = f'{label} ({confidence*100:.1f}%): {avg_distance:.2f}m'
#                 cv2.rectangle(self.image_undistorted, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
#                 cv2.putText(self.image_undistorted, text, (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

#                 # Log detected object with distance, position, and confidence
#                 self.get_logger().info(f'Detected {label}: Confidence = {confidence*100:.1f}%, Distance = {avg_distance:.2f}m, Position = {avg_position}')

#         # Display the result with YOLO, distances, bounding boxes, and LiDAR points
#         cv2.imshow('YOLOv8 Detection with LiDAR Points and 3D Position', self.image_undistorted)
#         cv2.waitKey(1)

#         # Slow down the output to 500ms
#         time.sleep(0.5)

#         # Print useful information to the console
#         self.print_detected_objects(detected_objects_info)

#     def print_detected_objects(self, detected_objects_info):
#         if detected_objects_info:
#             print(f"Detected {len(detected_objects_info)} objects:")
#             for obj in detected_objects_info:
#                 print(f"Object: {obj['label']}, Confidence: {obj['confidence']*100:.1f}%, Distance: {obj['distance']:.2f}m, Position: {obj['position']}, BBox: {obj['bbox']}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = CameraLidarFusion()
    
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()




#######################

# import numpy as np
# import cv2
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image, PointCloud2
# from cv_bridge import CvBridge
# import sensor_msgs_py.point_cloud2 as pc2
# from ultralytics import YOLO
# import torch
# import time
# import logging

# # Suppress YOLOv8 logging
# logging.getLogger("ultralytics").setLevel(logging.ERROR)

# class CameraLidarFusion(Node):
#     def __init__(self):
#         super().__init__('camera_lidar_fusion_node')

#         # Set up frame skipping to process every Nth frame
#         self.frame_counter = 0
#         self.frame_skip = 3  # Process every 3rd frame for faster detection

#         # Check for GPU availability and use GPU if available
#         self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
#         print(f"Using device: {self.device}")

#         # Load YOLOv8 model (nano version)
#         self.model = YOLO("yolov8n.pt")

#         # Move the model to the selected device (GPU or CPU)
#         self.model.to(self.device)

#         # Lower the confidence threshold to detect more objects
#         self.model.conf = 0.1  # Lower threshold for more detections

#         # Camera intrinsic parameters from the image you provided
#         self.camera_matrix = np.array([[761.7297, 0, 640.3348],
#                                        [0, 762.2555, 362.3959],
#                                        [0, 0, 1.0000]])

#         # Distortion coefficients (radial distortion, padded with zeros for tangential distortion)
#         self.dist_coeffs = np.array([-0.0079, -6.2538e-04, 0, 0, 0])

#         # Extrinsic parameters (rotation and translation)
#         self.rotation_matrix = np.array([[-0.0021, -0.9988, 0.0486],
#                                          [-0.2391, -0.0467, -0.9699],
#                                          [0.9710, -0.0137, -0.2387]])

#         self.translation_vector = np.array([0.1116, -0.3260, -0.1386])

#         self.bridge = CvBridge()
#         self.image_undistorted = None  # Initialize the undistorted image
#         self.bounding_boxes = []  # Initialize bounding boxes to avoid access before detection
#         self.labels = []  # Initialize labels
#         self.confidences = []  # Initialize confidences

#         # Set up ROS subscribers
#         self.camera_sub = self.create_subscription(Image, '/wamv/sensors/cameras/front_left_camera_sensor/image_raw', self.image_callback, 10)
#         self.lidar_sub = self.create_subscription(PointCloud2, '/wamv/sensors/lidars/lidar_wamv_sensor/points', self.lidar_callback, 10)

#         self.image_frame = None  # Store the latest image frame for use with LiDAR data

#     def image_callback(self, msg):
#         self.frame_counter += 1

#         # Skip frames to speed up detection (process every 3rd frame)
#         if self.frame_counter % self.frame_skip != 0:
#             return

#         try:
#             # Convert ROS image to OpenCV image
#             self.image_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

#             # Undistort the image using the camera intrinsic parameters (intrinsics)
#             self.image_undistorted = cv2.undistort(self.image_frame, self.camera_matrix, self.dist_coeffs)

#             # Resize the image to lower resolution (e.g., 320x320) for faster YOLO processing
#             resized_image = cv2.resize(self.image_undistorted, (320, 320))

#             # Run YOLOv8 object detection on the resized image
#             print("Running YOLO detection...")
#             results = self.model(resized_image)

#             # Rescale the bounding boxes back to the original resolution
#             scale_x = self.image_undistorted.shape[1] / resized_image.shape[1]
#             scale_y = self.image_undistorted.shape[0] / resized_image.shape[0]

#             # Clone the tensor before performing the in-place update
#             bbox_tensor = results[0].boxes.xyxy.clone()

#             bbox_tensor[:, [0, 2]] *= scale_x
#             bbox_tensor[:, [1, 3]] *= scale_y

#             # Ensure there are results and bounding boxes
#             if results and results[0].boxes is not None and len(results[0].boxes) > 0:
#                 # Filter results by confidence threshold
#                 valid_indices = results[0].boxes.conf.cpu().numpy() >= self.model.conf

#                 # Apply the confidence filter
#                 self.bounding_boxes = bbox_tensor.cpu().numpy()[valid_indices]  # Get the bounding boxes (xmin, ymin, xmax, ymax)
#                 self.labels = [self.model.names[int(cls)] for cls in results[0].boxes.cls.cpu().numpy()[valid_indices]]
#                 self.confidences = results[0].boxes.conf.cpu().numpy()[valid_indices]

#                 # Debugging output
#                 print(f"Detected {len(self.labels)} objects")

#                 for label, bbox, confidence in zip(self.labels, self.bounding_boxes, self.confidences):
#                     print(f"Label: {label}, Confidence: {confidence:.2f}, BBox: {bbox}")

#         except Exception as e:
#             self.get_logger().error(f"Error in image callback: {e}")

#     def lidar_callback(self, msg):
#         try:
#             if self.image_frame is None or self.image_undistorted is None or len(self.bounding_boxes) == 0:
#                 return  # Wait for the first image frame to arrive and undistorted image to be set, and check if bounding boxes exist

#             # Convert ROS PointCloud2 to XYZ data
#             pc_data = []
#             for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
#                 pc_data.append([point[0], point[1], point[2]])

#             pc_data = np.array(pc_data)

#             # Filter the LiDAR point cloud (within 10 meters)
#             distances = np.linalg.norm(pc_data, axis=1)
#             valid_indices = np.logical_and(distances <= 10, np.isfinite(pc_data).all(axis=1))
#             pt_cloud_data_filtered = pc_data[valid_indices]

#             # Ensure that valid LiDAR data exists
#             if len(pt_cloud_data_filtered) == 0:
#                 return

#             # Project the filtered LiDAR points onto the camera image and get the position for objects
#             self.project_lidar_to_image(pt_cloud_data_filtered)

#         except Exception as e:
#             self.get_logger().error(f"Error in LiDAR callback: {e}")

#     def project_lidar_to_image(self, lidar_points):
#         # Convert LiDAR points to homogeneous coordinates (add 1 for the translation part)
#         lidar_points_homogeneous = np.hstack((lidar_points, np.ones((lidar_points.shape[0], 1))))

#         # Apply the extrinsic transformation (rotation and translation)
#         extrinsic_matrix = np.hstack((self.rotation_matrix, self.translation_vector.reshape(3, 1)))
#         camera_points = lidar_points_homogeneous @ extrinsic_matrix.T  # Transform LiDAR points to camera frame

#         # Keep only points in front of the camera (positive Z in camera coordinates)
#         valid_indices = camera_points[:, 2] > 0
#         camera_points = camera_points[valid_indices]

#         # Project the 3D points in the camera frame to the 2D image plane using the intrinsic matrix
#         image_points = (self.camera_matrix @ camera_points[:, :3].T).T

#         # Normalize homogeneous coordinates to get pixel coordinates
#         image_points[:, 0] /= image_points[:, 2]
#         image_points[:, 1] /= image_points[:, 2]

#         # Ensure points are within image bounds
#         height, width, _ = self.image_undistorted.shape
#         valid_image_points = np.logical_and.reduce((
#             image_points[:, 0] >= 0,
#             image_points[:, 0] < width,
#             image_points[:, 1] >= 0,
#             image_points[:, 1] < height
#         ))

#         # Draw LiDAR points on the image as small red circles
#         for i, valid in enumerate(valid_image_points):
#             if valid:
#                 x, y = int(image_points[i, 0]), int(image_points[i, 1])
#                 cv2.circle(self.image_undistorted, (x, y), 2, (0, 0, 255), -1)  # Red points for LiDAR projections

#         # Calculate object distances based on bounding boxes
#         detected_objects_info = []
#         for i, bbox in enumerate(self.bounding_boxes):
#             xmin, ymin, xmax, ymax = map(int, bbox)
#             label = self.labels[i] if i < len(self.labels) else "Unknown"
#             confidence = self.confidences[i] if i < len(self.confidences) else 0

#             # Get LiDAR points within the bounding box in the image
#             in_bbox_indices = np.logical_and.reduce((
#                 image_points[:, 0] >= xmin,
#                 image_points[:, 0] <= xmax,
#                 image_points[:, 1] >= ymin,
#                 image_points[:, 1] <= ymax
#             ))

#             if np.any(in_bbox_indices):
#                 points_in_bbox = camera_points[in_bbox_indices]

#                 # Calculate the average distance for the object
#                 avg_distance = np.mean(np.linalg.norm(points_in_bbox[:, :3], axis=1))

#                 # Store object information
#                 detected_objects_info.append({
#                     "label": label,
#                     "bbox": [xmin, ymin, xmax, ymax],
#                     "distance": avg_distance,
#                     "confidence": confidence
#                 })

#                 # Draw bounding box, label, confidence, and distance on the image
#                 cv2.rectangle(self.image_undistorted, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
#                 text = f'{label} ({confidence*100:.1f}%): {avg_distance:.2f}m'
#                 cv2.putText(self.image_undistorted, text, (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

#                 # Log detected object with distance and confidence
#                 self.get_logger().info(f'Detected {label}: Confidence = {confidence*100:.1f}%, Distance = {avg_distance:.2f}m')

#         # Display the result with YOLO, distances, bounding boxes, and LiDAR points
#         cv2.imshow('YOLOv8 Detection with LiDAR Points and 3D Position', self.image_undistorted)
#         cv2.waitKey(1)

#         # Slow down the output to 500ms
#         time.sleep(0.5)

#         # Print useful information to the console
#         self.print_detected_objects(detected_objects_info)

#     def print_detected_objects(self, detected_objects_info):
#         if detected_objects_info:
#             print(f"Detected {len(detected_objects_info)} objects:")
#             for obj in detected_objects_info:
#                 print(f"Object: {obj['label']}, Confidence: {obj['confidence']*100:.1f}%, Distance: {obj['distance']:.2f}m, BBox: {obj['bbox']}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = CameraLidarFusion()
    
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()



########################################  works okay with yolo get average distance of bounding box

# import numpy as np
# import cv2
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image, PointCloud2
# from cv_bridge import CvBridge
# import sensor_msgs_py.point_cloud2 as pc2
# from ultralytics import YOLO
# import logging

# # Block YOLO logger
# logging.getLogger("ultralytics").setLevel(logging.CRITICAL)

# class CameraLidarFusion(Node):
#     def __init__(self):
#         super().__init__('camera_lidar_fusion_node')

#         # Camera intrinsic parameters from the image you provided
#         self.camera_matrix = np.array([[761.7297, 0, 640.3348],
#                                        [0, 762.2555, 362.3959],
#                                        [0, 0, 1.0000]])

#         # Distortion coefficients
#         self.dist_coeffs = np.array([-0.0079, -0.00062538, 0, 0, 0])

#         # Extrinsic parameters (rotation and translation)
#         self.rotation_matrix = np.array([[-0.0021, -0.9988, 0.0486],
#                                          [-0.2391, -0.0467, -0.9699],
#                                          [0.9710, -0.0137, -0.2387]])

#         self.translation_vector = np.array([0.1116, -0.3260, -0.1386])

#         self.bridge = CvBridge()
#         self.image_undistorted = None  # Initialize the undistorted image

#         # Load the YOLOv8 model
#         self.model = YOLO('best.pt')

#         # Set up ROS subscribers
#         self.camera_sub = self.create_subscription(
#             Image, '/wamv/sensors/cameras/front_left_camera_sensor/image_raw', self.image_callback, 10)
#         self.lidar_sub = self.create_subscription(
#             PointCloud2, '/wamv/sensors/lidars/lidar_wamv_sensor/points', self.lidar_callback, 10)

#         self.image_frame = None  # Store the latest image frame for use with LiDAR data
#         self.lidar_points = None  # Store LiDAR points for use in object distance calculation

#     def image_callback(self, msg):
#         try:
#             # Convert ROS image to OpenCV image
#             self.image_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

#             # Undistort the image using the camera intrinsic parameters (intrinsics)
#             self.image_undistorted = cv2.undistort(self.image_frame, self.camera_matrix, self.dist_coeffs)

#             # Perform object detection
#             results = self.model(self.image_undistorted, verbose=False)

#             # Extract bounding boxes and object names
#             for result in results:
#                 for box in result.boxes:
#                     x1, y1, x2, y2 = map(int, box.xyxy[0])  # Get bounding box coordinates
#                     obj_class = result.names[int(box.cls)]  # Get the class name of the detected object

#                     # Extract the LiDAR points corresponding to the bounding box
#                     if self.lidar_points is not None:
#                         self.extract_lidar_points_in_bbox(x1, y1, x2, y2, obj_class)

#                     # Draw bounding box on the image
#                     cv2.rectangle(self.image_undistorted, (x1, y1), (x2, y2), (0, 255, 0), 2)

#             # Show undistorted image with bounding boxes
#             cv2.imshow('Undistorted Image with Bounding Boxes', self.image_undistorted)
#             cv2.waitKey(1)
#         except Exception as e:
#             self.get_logger().error(f"Error in image callback: {e}")

#     def lidar_callback(self, msg):
#         try:
#             # Convert ROS PointCloud2 to XYZ data
#             pc_data = []
#             for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
#                 pc_data.append([point[0], point[1], point[2]])

#             pc_data = np.array(pc_data)

#             # Filter the LiDAR point cloud (within 10 meters)
#             distances = np.linalg.norm(pc_data, axis=1)
#             valid_indices = np.logical_and(distances <= 10, np.isfinite(pc_data).all(axis=1))
#             self.lidar_points = pc_data[valid_indices]

#         except Exception as e:
#             self.get_logger().error(f"Error in LiDAR callback: {e}")

#     def project_lidar_to_image(self, lidar_points):
#         # Convert LiDAR points to homogeneous coordinates (add 1 for the translation part)
#         lidar_points_homogeneous = np.hstack((lidar_points, np.ones((lidar_points.shape[0], 1))))

#         # Apply the extrinsic transformation (rotation and translation)
#         extrinsic_matrix = np.hstack(
#             (self.rotation_matrix, self.translation_vector.reshape(3, 1)))
#         camera_points = lidar_points_homogeneous @ extrinsic_matrix.T  # Transform LiDAR points to camera frame

#         # Keep only points in front of the camera (positive Z in camera coordinates)
#         valid_indices = camera_points[:, 2] > 0
#         camera_points = camera_points[valid_indices]
#         lidar_points = lidar_points[valid_indices]  # Filter the original LiDAR points

#         # Check if any LiDAR points are valid after filtering
#         if camera_points.shape[0] == 0:
#             self.get_logger().info('No valid LiDAR points to project.')
#             return np.array([]), np.array([])  # Return empty arrays

#         # Project the 3D points in the camera frame to the 2D image plane using the intrinsic matrix
#         image_points = (self.camera_matrix @ camera_points[:, :3].T).T

#         # Normalize homogeneous coordinates to get pixel coordinates
#         image_points[:, 0] /= image_points[:, 2]
#         image_points[:, 1] /= image_points[:, 2]

#         # Only keep the 2D (x, y) coordinates
#         image_points = image_points[:, :2]

#         # Return the 2D image points and the depths (Z-values in camera frame)
#         depths = camera_points[:, 2]

#         return image_points, depths

#     def extract_lidar_points_in_bbox(self, x1, y1, x2, y2, obj_class):
#         # Project LiDAR points to the image plane
#         if self.lidar_points is None:
#             return

#         image_points, depths = self.project_lidar_to_image(self.lidar_points)

#         # Check if there are any projected points
#         if image_points.size == 0:
#             self.get_logger().info('No projected LiDAR points available.')
#             return

#         # Find LiDAR points that fall within the bounding box
#         lidar_in_bbox_depths = []
#         for i, (x, y) in enumerate(image_points):
#             if x1 <= x <= x2 and y1 <= y <= y2:
#                 lidar_in_bbox_depths.append(depths[i])

#         if lidar_in_bbox_depths:
#             # Calculate the median distance to the object
#             average_distance = np.average(lidar_in_bbox_depths)
#             print(f"Detected {obj_class} at a median distance of {average_distance:.2f} meters")

#             # Draw the distance on the image
#             cv2.putText(self.image_undistorted, f'{obj_class}: {average_distance:.2f}m', (x1, y1 - 10),
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
#         # else:
#             # self.get_logger().info(f"No LiDAR points found within bounding box for {obj_class}.")

# def main(args=None):
#     rclpy.init(args=args)
#     node = CameraLidarFusion()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()



############################################ distance accuracy seems pretty good

# import numpy as np
# import cv2
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image, PointCloud2
# from cv_bridge import CvBridge
# import sensor_msgs_py.point_cloud2 as pc2
# from ultralytics import YOLO
# import logging
# from sklearn.cluster import DBSCAN

# # Block YOLO logger
# logging.getLogger("ultralytics").setLevel(logging.CRITICAL)

# class CameraLidarFusion(Node):
#     def __init__(self):
#         super().__init__('camera_lidar_fusion_node')

#         # Camera intrinsic parameters
#         self.camera_matrix = np.array([[761.7297, 0, 640.3348],
#                                        [0, 762.2555, 362.3959],
#                                        [0, 0, 1.0000]])

#         # Distortion coefficients
#         self.dist_coeffs = np.array([-0.0079, -0.00062538, 0, 0, 0])

#         # Extrinsic parameters (rotation and translation)
#         self.rotation_matrix = np.array([[-0.0021, -0.9988, 0.0486],
#                                          [-0.2391, -0.0467, -0.9699],
#                                          [0.9710, -0.0137, -0.2387]])

#         self.translation_vector = np.array([0.1116, -0.3260, -0.1386])

#         self.bridge = CvBridge()
#         self.image_undistorted = None  # Initialize the undistorted image

#         # Load the pre-trained YOLOv8 model (pre-trained on COCO dataset)
#         self.model = YOLO('best.pt')  # Pre-trained model on COCO dataset

#         # Set up ROS subscribers
#         self.camera_sub = self.create_subscription(
#             Image, '/wamv/sensors/cameras/front_left_camera_sensor/image_raw', self.image_callback, 10)
#         self.lidar_sub = self.create_subscription(
#             PointCloud2, '/wamv/sensors/lidars/lidar_wamv_sensor/points', self.lidar_callback, 10)

#         self.image_frame = None  # Store the latest image frame for use with LiDAR data
#         self.lidar_points = None  # Store LiDAR points for use in object distance calculation

#     def image_callback(self, msg):
#         try:
#             # Convert ROS image to OpenCV image
#             self.image_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

#             # Undistort the image using the camera intrinsic parameters (intrinsics)
#             self.image_undistorted = cv2.undistort(self.image_frame, self.camera_matrix, self.dist_coeffs)

#             # Perform object detection using the pre-trained YOLOv8 model
#             results = self.model(self.image_undistorted, verbose=False)

#             # Extract bounding boxes and object names
#             for result in results:
#                 for box in result.boxes:
#                     x1, y1, x2, y2 = map(int, box.xyxy[0])  # Get bounding box coordinates
#                     obj_class = result.names[int(box.cls)]  # Get the class name of the detected object

#                     # Extract the LiDAR points corresponding to the bounding box
#                     if self.lidar_points is not None:
#                         self.extract_lidar_points_in_bbox(x1, y1, x2, y2, obj_class)

#                     # Draw bounding box on the image
#                     cv2.rectangle(self.image_undistorted, (x1, y1), (x2, y2), (0, 255, 0), 2)

#             # Show undistorted image with bounding boxes
#             cv2.imshow('Undistorted Image with Bounding Boxes', self.image_undistorted)
#             cv2.waitKey(1)
#         except Exception as e:
#             self.get_logger().error(f"Error in image callback: {e}")

#     def lidar_callback(self, msg):
#         try:
#             # Convert ROS PointCloud2 to XYZ data
#             pc_data = []
#             for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
#                 pc_data.append([point[0], point[1], point[2]])

#             pc_data = np.array(pc_data)

#             # Filter the LiDAR point cloud (within 10 meters)
#             distances = np.linalg.norm(pc_data, axis=1)
#             valid_indices = np.logical_and(distances <= 10, np.isfinite(pc_data).all(axis=1))
#             self.lidar_points = pc_data[valid_indices]

#         except Exception as e:
#             self.get_logger().error(f"Error in LiDAR callback: {e}")

#     def project_lidar_to_image(self, lidar_points):
#         # Convert LiDAR points to homogeneous coordinates (add 1 for the translation part)
#         lidar_points_homogeneous = np.hstack((lidar_points, np.ones((lidar_points.shape[0], 1))))

#         # Apply the extrinsic transformation (rotation and translation)
#         extrinsic_matrix = np.hstack(
#             (self.rotation_matrix, self.translation_vector.reshape(3, 1)))
#         camera_points = lidar_points_homogeneous @ extrinsic_matrix.T  # Transform LiDAR points to camera frame

#         # Keep only points in front of the camera (positive Z in camera coordinates)
#         valid_indices = camera_points[:, 2] > 0
#         camera_points = camera_points[valid_indices]
#         lidar_points = lidar_points[valid_indices]  # Filter the original LiDAR points

#         # Check if any LiDAR points are valid after filtering
#         if camera_points.shape[0] == 0:
#             self.get_logger().info('No valid LiDAR points to project.')
#             return np.array([]), np.array([])  # Return empty arrays

#         # Project the 3D points in the camera frame to the 2D image plane using the intrinsic matrix
#         image_points = (self.camera_matrix @ camera_points[:, :3].T).T

#         # Normalize homogeneous coordinates to get pixel coordinates
#         image_points[:, 0] /= image_points[:, 2]
#         image_points[:, 1] /= image_points[:, 2]

#         # Only keep the 2D (x, y) coordinates
#         image_points = image_points[:, :2]

#         # Return the 2D image points and the depths (Z-values in camera frame)
#         depths = camera_points[:, 2]

#         return image_points, depths

#     def extract_lidar_points_in_bbox(self, x1, y1, x2, y2, obj_class):
#         # Project LiDAR points to the image plane
#         if self.lidar_points is None:
#             return

#         image_points, depths = self.project_lidar_to_image(self.lidar_points)

#         # Check if there are any projected points
#         if image_points.size == 0:
#             self.get_logger().info('No projected LiDAR points available.')
#             return

#         # Find LiDAR points that fall within the bounding box
#         lidar_in_bbox_depths = []
#         lidar_in_bbox_points = []
#         for i, (x, y) in enumerate(image_points):
#             if x1 <= x <= x2 and y1 <= y <= y2:
#                 lidar_in_bbox_depths.append(depths[i])
#                 lidar_in_bbox_points.append([x, y])

#         if len(lidar_in_bbox_depths) > 0:
#             # Cluster the points to remove noise using DBSCAN
#             lidar_in_bbox_points = np.array(lidar_in_bbox_points)
#             clustering = DBSCAN(eps=0.3, min_samples=10).fit(lidar_in_bbox_points)
#             labels = clustering.labels_

#             # Find the largest cluster and use its points
#             unique_labels, counts = np.unique(labels[labels != -1], return_counts=True)
#             if len(unique_labels) > 0:
#                 largest_cluster = unique_labels[np.argmax(counts)]
#                 lidar_in_bbox_depths = np.array(lidar_in_bbox_depths)[labels == largest_cluster]

#             # Calculate the weighted average distance to the object
#             distances = np.array(lidar_in_bbox_depths)
#             weights = 1 / distances  # Weight closer points more
#             weighted_avg_distance = np.average(distances, weights=weights)
#             print(f"Detected {obj_class} at a weighted average distance of {weighted_avg_distance:.2f} meters")

#             # Draw the distance on the image
#             cv2.putText(self.image_undistorted, f'{obj_class}: {weighted_avg_distance:.2f}m', (x1, y1 - 10),
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
#         # else:
#             # self.get_logger().info(f"No LiDAR points found within bounding box for {obj_class}.")

# def main(args=None):
#     rclpy.init(args=args)
#     node = CameraLidarFusion()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()



#########################

# import numpy as np
# import cv2
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image, PointCloud2
# from cv_bridge import CvBridge
# import sensor_msgs_py.point_cloud2 as pc2
# from ultralytics import YOLO
# import logging
# from sklearn.cluster import DBSCAN

# # Block YOLO logger
# logging.getLogger("ultralytics").setLevel(logging.CRITICAL)

# class CameraLidarFusion(Node):
#     def __init__(self):
#         super().__init__('camera_lidar_fusion_node')

#              # Camera intrinsic parameters
#         self.camera_matrix = np.array([[761.7297, 0, 640.3348],
#                                        [0, 762.2555, 362.3959],
#                                        [0, 0, 1.0000]])

#         # Distortion coefficients
#         self.dist_coeffs = np.array([-0.0079, -0.00062538, 0, 0, 0])

#         # Extrinsic parameters (rotation and translation)
#         self.rotation_matrix = np.array([[-0.0021, -0.9988, 0.0486],
#                                          [-0.2391, -0.0467, -0.9699],
#                                          [0.9710, -0.0137, -0.2387]])

#         self.translation_vector = np.array([0.1116, -0.3260, -0.1386])

#         self.bridge = CvBridge()
#         self.image_undistorted = None  # Initialize the undistorted image

#         # Load the pre-trained YOLOv8 model (pre-trained on COCO dataset)
#         self.model = YOLO('best.pt')  # Pre-trained model on COCO dataset

#         # Set up ROS subscribers VRX
#         self.camera_sub = self.create_subscription(
#             Image, '/wamv/sensors/cameras/front_left_camera_sensor/image_raw', self.image_callback, 10)
#         self.lidar_sub = self.create_subscription(
#             PointCloud2, '/wamv/sensors/lidars/lidar_wamv_sensor/points', self.lidar_callback, 10)
#         ### boat
#         # self.camera_sub = self.create_subscription(
#         #     Image, '/flir_camera/image_raw', self.image_callback, 10)
#         # self.lidar_sub = self.create_subscription(
#         #     PointCloud2, '/lidar_points', self.lidar_callback, 10)
        

#         self.image_frame = None  # Store the latest image frame for use with LiDAR data
#         self.lidar_points = None  # Store LiDAR points for use in object distance calculation

#         # Dictionary to track previously detected objects and distances
#         self.tracked_objects = {}

#         # Define a dictionary of color names and their RGB values
#         self.color_names = {
#             'black': (0, 0, 0),
#             'white': (255, 255, 255),
#             'red': (255, 0, 0),
#             'lime': (0, 255, 0),
#             'blue': (0, 0, 255),
#             'yellow': (255, 255, 0),
#             'cyan': (0, 255, 255),
#             'magenta': (255, 0, 255),
#             'silver': (192, 192, 192),
#             'gray': (128, 128, 128),
#             'maroon': (128, 0, 0),
#             'olive': (128, 128, 0),
#             'green': (0, 128, 0),
#             'purple': (128, 0, 128),
#             'teal': (0, 128, 128),
#             'navy': (0, 0, 128),
#             'orange': (255, 165, 0),
#             'pink': (255, 192, 203),
#             'brown': (165, 42, 42),
#             'gold': (255, 215, 0),
#             'beige': (245, 245, 220),
#             'coral': (255, 127, 80),
#             'turquoise': (64, 224, 208),
#             # Add more colors as needed
#         }

#     def image_callback(self, msg):
#         try:
#             # Convert ROS image to OpenCV image
#             self.image_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

#             # Undistort the image using the camera intrinsic parameters (intrinsics)
#             self.image_undistorted = cv2.undistort(self.image_frame, self.camera_matrix, self.dist_coeffs)

#             # Decrease the image resolution by 50% (scale factor 0.5)
#             scale_factor = 0.5
#             height, width = self.image_undistorted.shape[:2]
#             new_size = (int(width * scale_factor), int(height * scale_factor))
#             self.image_undistorted = cv2.resize(self.image_undistorted, new_size, interpolation=cv2.INTER_AREA)


#             # Perform object detection using the pre-trained YOLOv8 model
#             results = self.model(self.image_undistorted, verbose=False)

#             # Flag to check if a new object was detected
#             new_object_detected = False

#             # Extract bounding boxes and object names
#             for result in results:
#                 for box in result.boxes:
#                     x1, y1, x2, y2 = map(int, box.xyxy[0])  # Get bounding box coordinates
#                     obj_class = result.names[int(box.cls)]  # Get the class name of the detected object

#                     # Extract the LiDAR points corresponding to the bounding box
#                     if self.lidar_points is not None:
#                         new_object_detected = True
#                         self.extract_lidar_points_in_bbox(x1, y1, x2, y2, obj_class)

#                     # Draw bounding box on the image
#                     cv2.rectangle(self.image_undistorted, (x1, y1), (x2, y2), (0, 255, 0), 2)

#             # Show undistorted image with bounding boxes
#             cv2.imshow('Undistorted Image with Bounding Boxes', self.image_undistorted)
#             cv2.waitKey(1)

#             # If no new object detected, just update the distance of tracked objects
#             if not new_object_detected and self.tracked_objects:
#                 self.update_tracked_objects()

#         except Exception as e:
#             self.get_logger().error(f"Error in image callback: {e}")

#     def lidar_callback(self, msg):
#         try:
#             # Convert ROS PointCloud2 to XYZ data
#             pc_data = []
#             for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
#                 pc_data.append([point[0], point[1], point[2]])

#             pc_data = np.array(pc_data)

#             # Filter the LiDAR point cloud (within 10 meters)
#             distances = np.linalg.norm(pc_data, axis=1)
#             valid_indices = np.logical_and(distances <= 10, np.isfinite(pc_data).all(axis=1))
#             self.lidar_points = pc_data[valid_indices]

#         except Exception as e:
#             self.get_logger().error(f"Error in LiDAR callback: {e}")

#     def project_lidar_to_image(self, lidar_points):
#         # Convert LiDAR points to homogeneous coordinates (add 1 for the translation part)
#         lidar_points_homogeneous = np.hstack((lidar_points, np.ones((lidar_points.shape[0], 1))))

#         # Apply the extrinsic transformation (rotation and translation)
#         extrinsic_matrix = np.hstack(
#             (self.rotation_matrix, self.translation_vector.reshape(3, 1)))
#         camera_points = lidar_points_homogeneous @ extrinsic_matrix.T  # Transform LiDAR points to camera frame

#         # Keep only points in front of the camera (positive Z in camera coordinates)
#         valid_indices = camera_points[:, 2] > 0
#         camera_points = camera_points[valid_indices]
#         lidar_points = lidar_points[valid_indices]  # Filter the original LiDAR points

#         # Check if any LiDAR points are valid after filtering
#         if camera_points.shape[0] == 0:
#             self.get_logger().info('No valid LiDAR points to project.')
#             return np.array([]), np.array([])  # Return empty arrays

#         # Project the 3D points in the camera frame to the 2D image plane using the intrinsic matrix
#         image_points = (self.camera_matrix @ camera_points[:, :3].T).T

#         # Normalize homogeneous coordinates to get pixel coordinates
#         image_points[:, 0] /= image_points[:, 2]
#         image_points[:, 1] /= image_points[:, 2]

#         # Only keep the 2D (x, y) coordinates
#         image_points = image_points[:, :2]

#         # Return the 2D image points and the depths (Z-values in camera frame)
#         depths = camera_points[:, 2]

#         return image_points, depths

#     def extract_lidar_points_in_bbox(self, x1, y1, x2, y2, obj_class):
#         # Project LiDAR points to the image plane
#         if self.lidar_points is None:
#             return

#         image_points, depths = self.project_lidar_to_image(self.lidar_points)

#         # Check if there are any projected points
#         if image_points.size == 0:
#             self.get_logger().info('No projected LiDAR points available.')
#             return

#         # Find LiDAR points that fall within the bounding box
#         lidar_in_bbox_depths = []
#         lidar_in_bbox_points = []
#         for i, (x, y) in enumerate(image_points):
#             if x1 <= x <= x2 and y1 <= y <= y2:
#                 lidar_in_bbox_depths.append(depths[i])
#                 lidar_in_bbox_points.append([x, y])

#         if len(lidar_in_bbox_depths) > 0:
#             # Cluster the points to remove noise using DBSCAN
#             lidar_in_bbox_points = np.array(lidar_in_bbox_points)
#             clustering = DBSCAN(eps=0.3, min_samples=10).fit(lidar_in_bbox_points)
#             labels = clustering.labels_

#             # Find the largest cluster and use its points
#             unique_labels, counts = np.unique(labels[labels != -1], return_counts=True)
#             if len(unique_labels) > 0:
#                 largest_cluster = unique_labels[np.argmax(counts)]
#                 lidar_in_bbox_depths = np.array(lidar_in_bbox_depths)[labels == largest_cluster]

#             # Calculate the weighted average distance to the object
#             distances = np.array(lidar_in_bbox_depths)
#             weights = 1 / distances  # Weight closer points more
#             weighted_avg_distance = np.average(distances, weights=weights)

#             # Update tracked object info
#             self.tracked_objects[obj_class] = weighted_avg_distance
#             print(f"Detected {obj_class} at a weighted average distance of {weighted_avg_distance:.2f} meters")

#     def update_tracked_objects(self):
#         """Update and print the distance of tracked objects if no new object is detected."""
#         for obj_class, distance in self.tracked_objects.items():
#             print(f"Updated {obj_class} distance: {distance:.2f} meters")

#     def calculate_average_color(self, roi):
#         """
#         Calculate the average color of the region of interest (ROI).
#         ROI is expected to be in BGR format.
#         Returns the average color as a tuple of integers (B, G, R).
#         """
#         if roi.size == 0:
#             return (0, 0, 0)
#         avg_color_per_channel = np.mean(roi, axis=(0, 1))
#         avg_color_bgr = tuple(avg_color_per_channel.astype(int))
#         return avg_color_bgr

#     def get_color_name(self, avg_color_bgr):
#         """
#         Map the average BGR color to the closest color name.
#         """
#         # Convert BGR to RGB
#         avg_color_rgb = (avg_color_bgr[2], avg_color_bgr[1], avg_color_bgr[0])

#         min_distance = float('inf')
#         closest_color_name = "Unknown"

#         for color_name, color_rgb in self.color_names.items():
#             distance = self.euclidean_distance(avg_color_rgb, color_rgb)
#             if distance < min_distance:
#                 min_distance = distance
#                 closest_color_name = color_name

#         return closest_color_name

#     @staticmethod
#     def euclidean_distance(color1, color2):
#         """
#         Calculate the Euclidean distance between two RGB colors.
#         """
#         return np.sqrt(sum((e1 - e2) ** 2 for e1, e2 in zip(color1, color2)))

# def main(args=None):
#     rclpy.init(args=args)
#     node = CameraLidarFusion()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()



################################

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import sensor_msgs_py.point_cloud2 as pc2
from ultralytics import YOLO
import logging
from sklearn.cluster import DBSCAN

# Block YOLO logger
logging.getLogger("ultralytics").setLevel(logging.CRITICAL)

class CameraLidarFusion(Node):
    def __init__(self):
        super().__init__('camera_lidar_fusion_node')

        # Camera intrinsic parameters
        self.camera_matrix = np.array([[761.7297, 0, 640.3348],
                                       [0, 762.2555, 362.3959],
                                       [0, 0, 1.0000]])

        # Distortion coefficients
        self.dist_coeffs = np.array([-0.0079, -0.00062538, 0, 0, 0])

        # Extrinsic parameters (rotation and translation)
        self.rotation_matrix = np.array([[-0.0021, -0.9988, 0.0486],
                                         [-0.2391, -0.0467, -0.9699],
                                         [0.9710, -0.0137, -0.2387]])

        self.translation_vector = np.array([0.1116, -0.3260, -0.1386])

        self.bridge = CvBridge()
        self.image_undistorted = None  # Initialize the undistorted image

        # Load the pre-trained YOLOv8 model (pre-trained on COCO dataset)
        self.model = YOLO('best.pt')  # Ensure 'best.pt' is in the correct path

        # Dictionary to store the last position of detected objects
        self.last_positions = {}

        # Set up ROS subscribers
        self.camera_sub = self.create_subscription(
            Image, '/wamv/sensors/cameras/front_left_camera_sensor/image_raw', self.image_callback, 10)
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/wamv/sensors/lidars/lidar_wamv_sensor/points', self.lidar_callback, 10)

        self.image_frame = None  # Store the latest image frame for use with LiDAR data
        self.lidar_points = None  # Store LiDAR points for use in object distance calculation

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV image
            self.image_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Undistort the image using the camera intrinsic parameters (intrinsics)
            self.image_undistorted = cv2.undistort(self.image_frame, self.camera_matrix, self.dist_coeffs)

            # Perform object detection using the pre-trained YOLOv8 model
            results = self.model(self.image_undistorted, verbose=False)

            # Project all LiDAR points onto the image and visualize them
            if self.lidar_points is not None:
                self.visualize_lidar_on_image()

            # Extract bounding boxes and object names
            for result in results:
                for box in result.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])  # Get bounding box coordinates
                    obj_class = result.names[int(box.cls)]  # Get the class name of the detected object

                    # Check if the position has changed before logging
                    if self.has_position_changed(obj_class, x1, y1, x2, y2):
                        # Extract the LiDAR points corresponding to the bounding box
                        if self.lidar_points is not None:
                            self.extract_lidar_points_in_bbox(x1, y1, x2, y2, obj_class)

                    # Draw bounding box on the image
                    cv2.rectangle(self.image_undistorted, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Show undistorted image with bounding boxes and projected LiDAR points
            cv2.imshow('Undistorted Image with Bounding Boxes and LiDAR Points', self.image_undistorted)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error in image callback: {e}")

    def lidar_callback(self, msg):
        try:
            # Convert ROS PointCloud2 to XYZ data
            pc_data = []
            for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                pc_data.append([point[0], point[1], point[2]])

            pc_data = np.array(pc_data)

            # Filter the LiDAR point cloud (within 10 meters)
            distances = np.linalg.norm(pc_data, axis=1)
            valid_indices = np.logical_and(distances <= 10, np.isfinite(pc_data).all(axis=1))
            self.lidar_points = pc_data[valid_indices]

        except Exception as e:
            self.get_logger().error(f"Error in LiDAR callback: {e}")

    def project_lidar_to_image(self, lidar_points):
        # Convert LiDAR points to homogeneous coordinates (add 1 for the translation part)
        lidar_points_homogeneous = np.hstack((lidar_points, np.ones((lidar_points.shape[0], 1))))

        # Apply the extrinsic transformation (rotation and translation)
        extrinsic_matrix = np.hstack(
            (self.rotation_matrix, self.translation_vector.reshape(3, 1)))
        camera_points = lidar_points_homogeneous @ extrinsic_matrix.T  # Transform LiDAR points to camera frame

        # Keep only points in front of the camera (positive Z in camera coordinates)
        valid_indices = camera_points[:, 2] > 0
        camera_points = camera_points[valid_indices]
        lidar_points = lidar_points[valid_indices]  # Filter the original LiDAR points

        # Check if any LiDAR points are valid after filtering
        if camera_points.shape[0] == 0:
            self.get_logger().info('No valid LiDAR points to project.')
            return np.array([]), np.array([])  # Return empty arrays

        # Project the 3D points in the camera frame to the 2D image plane using the intrinsic matrix
        image_points = (self.camera_matrix @ camera_points[:, :3].T).T

        # Normalize homogeneous coordinates to get pixel coordinates
        image_points[:, 0] /= image_points[:, 2]
        image_points[:, 1] /= image_points[:, 2]

        # Only keep the 2D (x, y) coordinates
        image_points = image_points[:, :2]

        # Return the 2D image points and the depths (Z-values in camera frame)
        depths = camera_points[:, 2]

        return image_points, depths

    def extract_lidar_points_in_bbox(self, x1, y1, x2, y2, obj_class):
        # Project LiDAR points to the image plane
        if self.lidar_points is None:
            return

        image_points, depths = self.project_lidar_to_image(self.lidar_points)

        # Check if there are any projected points
        if image_points.size == 0:
            self.get_logger().info('No projected LiDAR points available.')
            return

        # Find LiDAR points that fall within the bounding box
        lidar_in_bbox_depths = []
        lidar_in_bbox_points = []
        for i, (x, y) in enumerate(image_points):
            if x1 <= x <= x2 and y1 <= y <= y2:
                lidar_in_bbox_depths.append(depths[i])
                lidar_in_bbox_points.append([x, y])

        if len(lidar_in_bbox_depths) > 0:
            # Cluster the points to remove noise using DBSCAN
            lidar_in_bbox_points = np.array(lidar_in_bbox_points)
            clustering = DBSCAN(eps=0.3, min_samples=10).fit(lidar_in_bbox_points)
            labels = clustering.labels_

            # Find the largest cluster and use its points
            unique_labels, counts = np.unique(labels[labels != -1], return_counts=True)
            if len(unique_labels) > 0:
                largest_cluster = unique_labels[np.argmax(counts)]
                lidar_in_bbox_depths = np.array(lidar_in_bbox_depths)[labels == largest_cluster]

            # Calculate the weighted average distance to the object
            distances = np.array(lidar_in_bbox_depths)
            weights = 1 / distances  # Weight closer points more
            weighted_avg_distance = np.average(distances, weights=weights)
            print(f"Detected {obj_class} at a weighted average distance of {weighted_avg_distance:.2f} meters")

            # Draw the distance on the image
            cv2.putText(self.image_undistorted, f'{obj_class}: {weighted_avg_distance:.2f}m', (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    def visualize_lidar_on_image(self):
        """
        Projects all valid LiDAR points onto the image and visualizes them.
        Points are color-coded based on their depth.
        """
        image_points, depths = self.project_lidar_to_image(self.lidar_points)

        if image_points.size == 0:
            self.get_logger().info('No LiDAR points to visualize.')
            return

        # Normalize depths for coloring (assuming max distance is 10 meters)
        max_distance = 10.0
        normalized_depths = np.clip(depths / max_distance, 0, 1)  # Normalize between 0 and 1

        # Convert normalized depths to colors (e.g., closer points in red, farther in blue)
        colors = cv2.applyColorMap((255 * (1 - normalized_depths)).astype(np.uint8), cv2.COLORMAP_JET)

        for i, (x, y) in enumerate(image_points):
            # Ensure the projected points are within image boundaries
            x_int = int(np.round(x))
            y_int = int(np.round(y))
            if 0 <= x_int < self.image_undistorted.shape[1] and 0 <= y_int < self.image_undistorted.shape[0]:
                # Convert the color to a tuple for cv2.circle (from [B, G, R] format)
                color = tuple(map(int, colors[i, 0]))  # colors[i, 0] is [B, G, R] in 1D, so convert to a tuple
                cv2.circle(self.image_undistorted, (x_int, y_int), 2, color, -1)

    def has_position_changed(self, obj_class, x1, y1, x2, y2):
        """
        Checks if the position of the detected object has changed.
        Returns True if the position has changed, otherwise False.
        """
        current_position = (x1, y1, x2, y2)
        
        if obj_class not in self.last_positions:
            # If no previous position, consider it changed
            self.last_positions[obj_class] = current_position
            return True

        # Compare current position with the last known position
        if self.last_positions[obj_class] != current_position:
            # Update the stored position if it has changed
            self.last_positions[obj_class] = current_position
            return True

        # Position hasn't changed
        return False

def main(args=None):
    rclpy.init(args=args)
    node = CameraLidarFusion()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
