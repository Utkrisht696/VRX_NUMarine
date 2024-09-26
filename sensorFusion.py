# # import rclpy
# # from rclpy.node import Node
# # from sensor_msgs.msg import Image, LaserScan
# # import numpy as np
# # import cv2
# # from cv_bridge import CvBridge
# # from time import time

# # class SensorFusionNode(Node):
# #     def __init__(self):
# #         super().__init__('sensor_fusion_node')
# #         self.camera_subscriber = self.create_subscription(
# #             Image,
# #             '/wamv/sensors/cameras/front_left_camera_sensor/image_raw',
# #             self.camera_callback,
# #             10)
# #         self.lidar_subscriber = self.create_subscription(
# #             LaserScan,
# #             '/wamv/sensors/lidars/lidar_wamv_sensor/scan',
# #             self.lidar_callback,
# #             10)
# #         self.bridge = CvBridge()
# #         self.camera_data = None
# #         self.lidar_data = None
# #         self.last_log_time = time()

#     # def camera_callback(self, msg):
#     #     self.camera_data = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#     #     self.fuse_data()

#     # def lidar_callback(self, msg):
#     #     self.lidar_data = msg
#     #     self.fuse_data()

#     # def fuse_data(self):
#     #     if self.camera_data is not None and self.lidar_data is not None:
#     #         angle_min = self.lidar_data.angle_min
#     #         angle_max = self.lidar_data.angle_max
#     #         angle_increment = self.lidar_data.angle_increment
#     #         ranges = np.array(self.lidar_data.ranges)
#     #         angles = np.linspace(angle_min, angle_max, len(ranges))

#     #         current_time = time()
#     #         for r, theta in zip(ranges, angles):
#     #             if 0.2 < r < 30.0:  # Valid range
#     #                 # Calculate position
#     #                 x = int(r * np.cos(theta) * 100 + self.camera_data.shape[1] / 2)
#     #                 y = int(r * np.sin(theta) * 100 + self.camera_data.shape[0] / 2)

#     #                 # Structured data
#     #                 point_data = {
#     #                     'range': r,
#     #                     'angle': theta,
#     #                     'cartesian': (r * np.cos(theta), r * np.sin(theta)),
#     #                     'image_coords': (x, y)
#     #                 }

#     #                 # Print debug information every 200ms
#     #                 if current_time - self.last_log_time >= 0.2:
#     #                     self.get_logger().info(f'LiDAR point data: {point_data}')
#     #                     self.last_log_time = current_time

#     #                 if 0 <= x < self.camera_data.shape[1] and 0 <= y < self.camera_data.shape[0]:
#     #                     cv2.circle(self.camera_data, (x, y), 3, (0, 255, 0), -1)
#     #                     cv2.putText(self.camera_data, f"{r:.2f}", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, cv2.LINE_AA)
#     #                 else:
#     #                     if current_time - self.last_log_time >= 0.2:
#     #                         self.get_logger().warn(f'Point {point_data["image_coords"]} out of image bounds')
#     #                         self.last_log_time = current_time

#     #         # Display the result
#     #         cv2.imshow('Sensor Fusion', self.camera_data)
#     #         cv2.waitKey(1)

# # def main(args=None):
# #     rclpy.init(args=args)
# #     node = SensorFusionNode()
# #     rclpy.spin(node)
# #     node.destroy_node()
# #     rclpy.shutdown()

# # if __name__ == '__main__':
# #     main()




# # ####################################################3D

# # import rclpy
# # from rclpy.node import Node
# # from sensor_msgs.msg import Image, PointCloud2
# # import numpy as np
# # import open3d as o3d
# # from cv_bridge import CvBridge
# # import sensor_msgs_py.point_cloud2 as pc2
# # from time import time
# # import threading

# # class SensorFusionNode(Node):
# #     def __init__(self):
# #         super().__init__('sensor_fusion_node')
# #         self.camera_subscriber = self.create_subscription(
# #             Image,
# #             '/wamv/sensors/cameras/front_left_camera_sensor/image_raw',
# #             self.camera_callback,
# #             10)
# #         self.lidar_subscriber = self.create_subscription(
# #             PointCloud2,
# #             '/wamv/sensors/lidars/lidar_wamv_sensor/points',
# #             self.lidar_callback,
# #             10)
# #         self.bridge = CvBridge()
# #         self.camera_image = None
# #         self.lidar_data = None
# #         self.last_log_time = time()
# #         self.lock = threading.Lock()

# #     def camera_callback(self, msg):
# #         self.get_logger().info("Received camera image data")
# #         with self.lock:
# #             self.camera_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
# #         self.fuse_data()

# #     def lidar_callback(self, msg):
# #         self.get_logger().info("Received LiDAR point cloud data")
# #         with self.lock:
# #             self.lidar_data = msg
# #         self.fuse_data()

# #     def fuse_data(self):
# #         with self.lock:
# #             if self.camera_image is None:
# #                 self.get_logger().warn("Camera image data not yet received")
# #                 return
# #             if self.lidar_data is None:
# #                 self.get_logger().warn("LiDAR point cloud data not yet received")
# #                 return

# #             self.get_logger().info("Fusing data")
# #             try:
# #                 lidar_points = list(pc2.read_points(self.lidar_data, field_names=("x", "y", "z"), skip_nans=True))
# #                 self.get_logger().info(f"Number of points in LiDAR point cloud: {len(lidar_points)}")

# #                 if not lidar_points:
# #                     self.get_logger().warn("No points in LiDAR point cloud")
# #                     return

# #                 # Filter out invalid points
# #                 lidar_points = [p for p in lidar_points if np.isfinite(p[0]) and np.isfinite(p[1]) and np.isfinite(p[2])]

# #                 if not lidar_points:
# #                     self.get_logger().warn("No valid points in LiDAR point cloud after filtering")
# #                     return

# #                 # Extract x, y, z coordinates and convert to numpy array
# #                 lidar_np = np.array([[p[0], p[1], p[2]] for p in lidar_points], dtype=np.float32)

# #                 # Convert camera image to point cloud
# #                 img_height, img_width, _ = self.camera_image.shape
# #                 fx, fy = 500, 500  # Focal lengths (adjust as necessary)
# #                 cx, cy = img_width / 2, img_height / 2  # Principal points (center of the image)
# #                 img_points = []
# #                 for v in range(img_height):
# #                     for u in range(img_width):
# #                         color = self.camera_image[v, u, :]
# #                         z = 0  # Assuming the image plane is at z=0
# #                         x = (u - cx) / fx
# #                         y = (v - cy) / fy
# #                         img_points.append([x, y, z, color[2] / 255.0, color[1] / 255.0, color[0] / 255.0])

# #                 img_pcd = o3d.geometry.PointCloud()
# #                 img_points_np = np.array(img_points, dtype=np.float32)
# #                 img_pcd.points = o3d.utility.Vector3dVector(img_points_np[:, :3])
# #                 img_pcd.colors = o3d.utility.Vector3dVector(img_points_np[:, 3:])

# #                 # Create point cloud from LiDAR data
# #                 lidar_pcd = o3d.geometry.PointCloud()
# #                 lidar_pcd.points = o3d.utility.Vector3dVector(lidar_np)

# #                 # Combine the point clouds
# #                 combined_pcd = img_pcd + lidar_pcd

# #                 # Visualize combined point cloud
# #                 o3d.visualization.draw_geometries([combined_pcd])

# #             except Exception as e:
# #                 self.get_logger().error(f"Error processing point cloud data: {e}")

# # def main(args=None):
# #     rclpy.init(args=args)
# #     node = SensorFusionNode()
# #     rclpy.spin(node)
# #     node.destroy_node()
# #     rclpy.shutdown()

# # if __name__ == '__main__':
# #     main()


# # import rclpy
# # from rclpy.node import Node
# # from sensor_msgs.msg import Image, PointCloud2
# # import numpy as np
# # import open3d as o3d
# # from cv_bridge import CvBridge
# # import sensor_msgs_py.point_cloud2 as pc2
# # from time import time
# # import threading

# # class SensorFusionNode(Node):
# #     def __init__(self):
# #         super().__init__('sensor_fusion_node')
# #         self.camera_subscriber = self.create_subscription(
# #             Image,
# #             '/wamv/sensors/cameras/front_left_camera_sensor/image_raw',
# #             self.camera_callback,
# #             10)
# #         self.lidar_subscriber = self.create_subscription(
# #             PointCloud2,
# #             '/wamv/sensors/lidars/lidar_wamv_sensor/points',
# #             self.lidar_callback,
# #             10)
# #         self.bridge = CvBridge()
# #         self.camera_image = None
# #         self.lidar_data = None
# #         self.last_log_time = time()
# #         self.lock = threading.Lock()

# #     def camera_callback(self, msg):
# #         self.get_logger().info("Received camera image data")
# #         with self.lock:
# #             self.camera_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
# #         self.fuse_data()

# #     def lidar_callback(self, msg):
# #         self.get_logger().info("Received LiDAR point cloud data")
# #         with self.lock:
# #             self.lidar_data = msg
# #         self.fuse_data()

# #     def fuse_data(self):
# #         with self.lock:
# #             if self.camera_image is None:
# #                 self.get_logger().warn("Camera image data not yet received")
# #                 return
# #             if self.lidar_data is None:
# #                 self.get_logger().warn("LiDAR point cloud data not yet received")
# #                 return

# #             self.get_logger().info("Fusing data")
# #             try:
# #                 lidar_points = list(pc2.read_points(self.lidar_data, field_names=("x", "y", "z"), skip_nans=True))
# #                 self.get_logger().info(f"Number of points in LiDAR point cloud: {len(lidar_points)}")

# #                 if not lidar_points:
# #                     self.get_logger().warn("No points in LiDAR point cloud")
# #                     return

# #                 # Filter out invalid points
# #                 lidar_points = [p for p in lidar_points if np.isfinite(p[0]) and np.isfinite(p[1]) and np.isfinite(p[2])]

# #                 if not lidar_points:
# #                     self.get_logger().warn("No valid points in LiDAR point cloud after filtering")
# #                     return

# #                 # Extract x, y, z coordinates and convert to numpy array
# #                 lidar_np = np.array([[p[0], p[1], p[2]] for p in lidar_points], dtype=np.float32)

# #                 # Camera intrinsics (example values, adjust as necessary)
# #                 fx, fy = 500, 500  # Focal lengths
# #                 cx, cy = self.camera_image.shape[1] / 2, self.camera_image.shape[0] / 2  # Principal points

# #                 # Convert camera image to point cloud
# #                 img_height, img_width, _ = self.camera_image.shape
# #                 img_points = []
# #                 for v in range(img_height):
# #                     for u in range(img_width):
# #                         color = self.camera_image[v, u, :]
# #                         z = 1  # Assume the image plane is at z=1 for better visualization
# #                         x = (u - cx) / fx
# #                         y = (v - cy) / fy
# #                         img_points.append([x * z, y * z, z, color[2] / 255.0, color[1] / 255.0, color[0] / 255.0])

# #                 img_pcd = o3d.geometry.PointCloud()
# #                 img_points_np = np.array(img_points, dtype=np.float32)
# #                 img_pcd.points = o3d.utility.Vector3dVector(img_points_np[:, :3])
# #                 img_pcd.colors = o3d.utility.Vector3dVector(img_points_np[:, 3:])

# #                 # Create point cloud from LiDAR data
# #                 lidar_pcd = o3d.geometry.PointCloud()
# #                 lidar_pcd.points = o3d.utility.Vector3dVector(lidar_np)

# #                 # Transform LiDAR points if necessary (example translation)
# #                 lidar_pcd.translate([0, 0, -1])

# #                 # Combine the point clouds
# #                 combined_pcd = img_pcd + lidar_pcd

# #                 # Visualize combined point cloud
# #                 o3d.visualization.draw_geometries([combined_pcd])

# #             except Exception as e:
# #                 self.get_logger().error(f"Error processing point cloud data: {e}")

# # def main(args=None):
# #     rclpy.init(args=args)
# #     node = SensorFusionNode()
# #     rclpy.spin(node)
# #     node.destroy_node()
# #     rclpy.shutdown()

# # if __name__ == '__main__':
# #     main()




# #########



# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image, PointCloud2
# import numpy as np
# from cv_bridge import CvBridge
# import sensor_msgs_py.point_cloud2 as pc2
# import cv2
# import message_filters
# import threading
# import time

# class SensorFusionNode(Node):
#     def __init__(self):
#         super().__init__('sensor_fusion_node')
        
#         # Subscribers for camera and LiDAR data
#         self.camera_subscriber = message_filters.Subscriber(self, Image, '/wamv/sensors/cameras/front_left_camera_sensor/image_raw')
#         self.lidar_subscriber = message_filters.Subscriber(self, PointCloud2, '/wamv/sensors/lidars/lidar_wamv_sensor/points')
        
#         # Synchronize camera and LiDAR data
#         self.ts = message_filters.ApproximateTimeSynchronizer([self.camera_subscriber, self.lidar_subscriber], 10, 0.1)
#         self.ts.registerCallback(self.data_callback)
        
#         # Initialize CvBridge
#         self.bridge = CvBridge()
        
#         # Placeholder for intrinsic and extrinsic calibration parameters
#         self.camera_intrinsics = np.array([[500, 0, 320], [0, 500, 240], [0, 0, 1]])  # Example values
#         self.dist_coeffs = np.zeros((5, 1))  # Assuming no distortion for simplicity
#         self.camera_extrinsics = np.eye(4)  # Identity matrix for simplicity, replace with actual extrinsics
        
#         # Initialize the lock
#         self.lock = threading.Lock()
        
#         # Initialize data attributes
#         self.camera_image = None
#         self.lidar_data = None

#         # Load YOLOv3 model
#         yolo_cfg_path = "./yolov3.cfg"  # Path to YOLOv3 config file
#         yolo_weights_path = "./yolov3.weights"  # Path to YOLOv3 weights file
#         yolo_names_path = "./coco.names"  # Path to COCO names file

#         self.net = cv2.dnn.readNet(yolo_weights_path, yolo_cfg_path)
#         self.layer_names = self.net.getLayerNames()
        
#         # Check if getUnconnectedOutLayers returns scalars or arrays
#         out_layers = self.net.getUnconnectedOutLayers()
#         if isinstance(out_layers, (list, tuple, np.ndarray)) and isinstance(out_layers[0], (list, tuple, np.ndarray)):
#             self.output_layers = [self.layer_names[i[0] - 1] for i in out_layers]
#         else:
#             self.output_layers = [self.layer_names[i - 1] for i in out_layers]
        
#         self.classes = open(yolo_names_path).read().strip().split("\n")

#         # Thread for visualization
#         self.visualizer_thread = threading.Thread(target=self.visualize_data)
#         self.visualizer_thread.start()

#     def data_callback(self, image_msg, lidar_msg):
#         with self.lock:
#             self.camera_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
#             self.lidar_data = lidar_msg
#         self.get_logger().info("Received synchronized camera and LiDAR data")
    
#     def preprocess_lidar(self, lidar_msg):
#         lidar_points = list(pc2.read_points(lidar_msg, field_names=("x", "y", "z"), skip_nans=True))
#         if not lidar_points:
#             return np.empty((0, 3), dtype=np.float32)
        
#         # Extract x, y, z coordinates and convert to numpy array
#         lidar_np = np.array([[p[0], p[1], p[2]] for p in lidar_points], dtype=np.float32)
        
#         # Apply voxel grid filter using numpy operations
#         voxel_size = 0.1
#         unique_voxels = np.unique((lidar_np / voxel_size).astype(int), axis=0)
#         filtered_lidar_points = unique_voxels * voxel_size + voxel_size / 2.0
        
#         return filtered_lidar_points
    
#     def detect_objects(self, image):
#         height, width = image.shape[:2]
#         blob = cv2.dnn.blobFromImage(image, 0.00392, (320, 320), (0, 0, 0), True, crop=False)
#         self.net.setInput(blob)
#         outs = self.net.forward(self.output_layers)

#         class_ids = []
#         confidences = []
#         boxes = []

#         for out in outs:
#             for detection in out:
#                 scores = detection[5:]
#                 class_id = np.argmax(scores)
#                 confidence = scores[class_id]
#                 if confidence > 0.5:
#                     center_x = int(detection[0] * width)
#                     center_y = int(detection[1] * height)
#                     w = int(detection[2] * width)
#                     h = int(detection[3] * height)
#                     x = int(center_x - w / 2)
#                     y = int(center_y - h / 2)
#                     boxes.append([x, y, w, h])
#                     confidences.append(float(confidence))
#                     class_ids.append(class_id)

#         indices = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
#         result_boxes = []
#         result_class_ids = []
#         for i in range(len(boxes)):
#             if i in indices:
#                 result_boxes.append(boxes[i])
#                 result_class_ids.append(class_ids[i])
        
#         return result_boxes, result_class_ids

#     def project_lidar_to_image(self, lidar_points, camera_intrinsics, camera_extrinsics):
#         # Transform LiDAR points to camera frame
#         lidar_homogeneous = np.hstack((lidar_points, np.ones((lidar_points.shape[0], 1))))
#         camera_points = lidar_homogeneous @ camera_extrinsics.T
        
#         # Project to 2D image plane
#         camera_points = camera_points[:, :3] / camera_points[:, 2][:, np.newaxis]
#         image_points = camera_points @ camera_intrinsics.T
#         return image_points
    
#     def get_color_and_distance(self, image_points, lidar_points, boxes):
#         colors = []
#         distances = []
#         for box in boxes:
#             x, y, w, h = box
#             x1, y1, x2, y2 = x, y, x + w, y + h

#             # Find the LiDAR points within the bounding box
#             mask = (image_points[:, 0] >= x1) & (image_points[:, 0] <= x2) & (image_points[:, 1] >= y1) & (image_points[:, 1] <= y2)
#             points_in_box = lidar_points[mask]
#             image_points_in_box = image_points[mask]

#             if points_in_box.size > 0:
#                 # Get the closest LiDAR point
#                 distances.append(np.min(np.linalg.norm(points_in_box, axis=1)))

#                 # Get the average color in the bounding box
#                 color = np.mean(self.camera_image[y1:y2, x1:x2], axis=(0, 1)).astype(int)
#                 colors.append(color)
#             else:
#                 distances.append(None)
#                 colors.append(None)
        
#         return colors, distances

#     def visualize_data(self):
#         last_update_time = time.time()
#         update_interval = 1.0 / 50.0  # Limit to 30 frames per second
#         while rclpy.ok():
#             current_time = time.time()
#             if current_time - last_update_time < update_interval:
#                 time.sleep(0.01)
#                 continue

#             with self.lock:
#                 if self.camera_image is None or self.lidar_data is None:
#                     continue
                
#                 # Preprocess data
#                 filtered_lidar_points = self.preprocess_lidar(self.lidar_data)
                
#                 # Object detection
#                 result_boxes, result_class_ids = self.detect_objects(self.camera_image)
                
#                 # Project LiDAR points onto camera image plane
#                 image_points = self.project_lidar_to_image(filtered_lidar_points, self.camera_intrinsics, self.camera_extrinsics)
                
#                 # Get colors and distances for detected objects
#                 colors, distances = self.get_color_and_distance(image_points, filtered_lidar_points, result_boxes)
                
#                 # Draw object detection results and distances
#                 for i, box in enumerate(result_boxes):
#                     x, y, w, h = box
#                     label = str(self.classes[result_class_ids[i]])
#                     color = colors[i] if colors[i] is not None else (0, 255, 0)
#                     distance = distances[i] if distances[i] is not None else 'N/A'
#                     cv2.rectangle(self.camera_image, (x, y), (x + w, y + h), color, 2)
#                     if distances[i] is not None:
#                         cv2.putText(self.camera_image, f'{label} {distance:.2f}m', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
#                     else:
#                         cv2.putText(self.camera_image, f'{label} {distance}', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                
#                 # Visualize
#                 visual_image = self.camera_image.copy()
#                 height, width, _ = visual_image.shape
#                 for point in image_points:
#                     u, v = int(point[0]), int(point[1])
#                     v = height - 1 - v  # Invert the y-coordinate
#                     if 0 <= u < width and 0 <= v < height:
#                         cv2.circle(visual_image, (u, v), 2, (0, 255, 0), -1)
                
#                 cv2.imshow("Camera Image with LiDAR Overlay", visual_image)
#                 cv2.waitKey(1)
            
#             last_update_time = current_time
    
#     def destroy_node(self):
#         cv2.destroyAllWindows()
#         super().destroy_node()

# def main(args=None):
#     rclpy.init(args=args)
#     node = SensorFusionNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
##############################