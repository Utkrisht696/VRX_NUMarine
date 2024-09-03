## SENSOR FUSION USING ROSBAG and SolvePNP 


# import numpy as np
# import cv2
# from cv_bridge import CvBridge
# import rosbag2_py
# from sensor_msgs.msg import Image, CameraInfo, PointCloud2
# from sensor_msgs_py import point_cloud2
# from rclpy.serialization import deserialize_message
# from rosidl_runtime_py.utilities import get_message

# # Camera Intrinsics (from your calibration)
# K = np.array([[2585.8849514873077, 0.0, 1495.8212207667002],
#               [0.0, 2594.1433908794033, 973.2642905461533],
#               [0.0, 0.0, 1.0]])

# D = np.array([-0.1950356958649444, 0.1383504196472886, 0.0002563441895758756, -0.0003855201794363196, 0.0])

# # Specify the ROS bag file path
# BAG_FILE_PATH = "/home/nuc1/LIDAR_CAMERA_DATA_WITH_CHESSBOARD/rosbag2_2024_09_01-17_50_07_0.db3"

# # Function to extract 2D points from the image
# def extract_image_points(image_msg):
#     bridge = CvBridge()
#     image = bridge.imgmsg_to_cv2(image_msg, "bgr8")
    
#     # Convert the image to grayscale
#     gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
#     # Find chessboard corners
#     pattern_size = (10, 7)  # 10x7 chessboard pattern
#     ret, corners = cv2.findChessboardCorners(gray, pattern_size)
    
#     if not ret:
#         raise ValueError("Chessboard corners not found")
    
#     # Refine the corners for more accurate detection
#     corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
#                                (cv2.TermCriteria_EPS + cv2.TermCriteria_COUNT, 30, 0.1))
#     return corners.squeeze()

# # Function to extract 3D points from the LiDAR point cloud
# def extract_lidar_points(lidar_msg):
#     # Convert PointCloud2 message to a list of points
#     points = list(point_cloud2.read_points(lidar_msg, field_names=("x", "y", "z"), skip_nans=True))
    
#     # Extract the x, y, z coordinates from the structured array into a simple NumPy array
#     points = np.array([(p[0], p[1], p[2]) for p in points], dtype=np.float32)
    
#     # Ensure the array has the correct shape
#     num_corners = 70  # 10x7 chessboard has 70 corners
#     if points.shape[0] < num_corners:
#         raise ValueError("Not enough points in LiDAR data")
    
#     return points[:num_corners]

# # Function to compute extrinsics using solvePnP
# def compute_extrinsics(image_points, object_points):
#     # Use solvePnP to compute the extrinsic parameters
#     success, rvec, tvec = cv2.solvePnP(object_points, image_points, K, D)
    
#     if not success:
#         raise ValueError("solvePnP failed")
    
#     # Convert rotation vector to rotation matrix
#     R, _ = cv2.Rodrigues(rvec)
    
#     return R, tvec

# def main():
#     # Open the ROS2 bag
#     storage_options = rosbag2_py.StorageOptions(uri=BAG_FILE_PATH, storage_id="sqlite3")
#     converter_options = rosbag2_py.ConverterOptions(
#         input_serialization_format="cdr", 
#         output_serialization_format="cdr"
#     )
#     reader = rosbag2_py.SequentialReader()
#     reader.open(storage_options, converter_options)

#     # Get message types
#     image_msg_type = get_message("sensor_msgs/msg/Image")
#     lidar_msg_type = get_message("sensor_msgs/msg/PointCloud2")

#     # Variables to store data
#     image_msg = None
#     lidar_msg = None

#     # Iterate through the messages
#     while reader.has_next():
#         topic, data, t = reader.read_next()

#         if topic == "/flir_camera/image_raw":
#             image_msg = deserialize_message(data, image_msg_type)
        
#         elif topic == "/lidar_points":
#             lidar_msg = deserialize_message(data, lidar_msg_type)
        
#         # Break once both messages have been obtained
#         if image_msg is not None and lidar_msg is not None:
#             break

#     # Check that we have the necessary data
#     if image_msg is None or lidar_msg is None:
#         raise ValueError("Required data not found in the bag")

#     # Extract points
#     image_points = extract_image_points(image_msg)
#     lidar_points = extract_lidar_points(lidar_msg)

#     # Compute extrinsic parameters
#     R, tvec = compute_extrinsics(image_points, lidar_points)
    
#     # Print the results
#     print("Rotation Matrix (R):\n", R)
#     print("Translation Vector (tvec):\n", tvec)

# if __name__ == "__main__":
#     main()



import numpy as np
import cv2
from cv_bridge import CvBridge
import rosbag2_py
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# Camera Intrinsics (from your calibration)
K = np.array([[2585.8849514873077, 0.0, 1495.8212207667002],
              [0.0, 2594.1433908794033, 973.2642905461533],
              [0.0, 0.0, 1.0]])

D = np.array([-0.1950356958649444, 0.1383504196472886, 0.0002563441895758756, -0.0003855201794363196, 0.0])

# Specify the ROS bag file path
BAG_FILE_PATH = "/home/nuc1/LIDAR_CAMERA_DATA_WITH_CHESSBOARD/rosbag2_2024_09_01-17_50_07_0.db3"

# Function to extract 2D points from the image
def extract_image_points(image_msg):
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(image_msg, "bgr8")
    
    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Find chessboard corners
    pattern_size = (10, 7)  # 10x7 chessboard pattern
    ret, corners = cv2.findChessboardCorners(gray, pattern_size)
    
    if not ret:
        raise ValueError("Chessboard corners not found")
    
    # Refine the corners for more accurate detection
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                               (cv2.TermCriteria_EPS + cv2.TermCriteria_COUNT, 30, 0.1))
    return corners.squeeze(), image

# Function to extract 3D points from the LiDAR point cloud
def extract_lidar_points(lidar_msg):
    # Convert PointCloud2 message to a list of points
    points = list(point_cloud2.read_points(lidar_msg, field_names=("x", "y", "z"), skip_nans=True))
    
    # Extract the x, y, z coordinates from the structured array into a simple NumPy array
    points = np.array([(p[0], p[1], p[2]) for p in points], dtype=np.float32)
    
    # Ensure the array has the correct shape
    num_corners = 70  # 10x7 chessboard has 70 corners
    if points.shape[0] < num_corners:
        raise ValueError("Not enough points in LiDAR data")
    
    return points[:num_corners]

# Function to compute extrinsics using solvePnP
def compute_extrinsics(image_points, object_points):
    # Use RANSAC to robustly estimate the extrinsics
    success, rvec, tvec, inliers = cv2.solvePnPRansac(object_points, image_points, K, D)
    
    if not success:
        raise ValueError("solvePnPRansac failed")
    
    # Convert rotation vector to rotation matrix
    R, _ = cv2.Rodrigues(rvec)
    
    return R, tvec, inliers

# Function to project LiDAR points onto the image plane
def project_lidar_to_image(lidar_points, R, tvec, K, D):
    # Project 3D points to 2D using the rotation, translation, and camera matrix
    image_points, _ = cv2.projectPoints(lidar_points, R, tvec, K, D)
    
    return image_points

def main():
    # Open the ROS2 bag
    storage_options = rosbag2_py.StorageOptions(uri=BAG_FILE_PATH, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", 
        output_serialization_format="cdr"
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # Get message types
    image_msg_type = get_message("sensor_msgs/msg/Image")
    lidar_msg_type = get_message("sensor_msgs/msg/PointCloud2")

    # Variables to store data
    image_msg = None
    lidar_msg = None

    # Iterate through the messages
    while reader.has_next():
        topic, data, t = reader.read_next()

        if topic == "/flir_camera/image_raw":
            image_msg = deserialize_message(data, image_msg_type)
        
        elif topic == "/lidar_points":
            lidar_msg = deserialize_message(data, lidar_msg_type)
        
        # Break once both messages have been obtained
        if image_msg is not None and lidar_msg is not None:
            break

    # Check that we have the necessary data
    if image_msg is None or lidar_msg is None:
        raise ValueError("Required data not found in the bag")

    # Extract points
    image_points, image = extract_image_points(image_msg)
    lidar_points = extract_lidar_points(lidar_msg)

    # Compute extrinsic parameters
    R, tvec, inliers = compute_extrinsics(image_points, lidar_points)
    
    # Project the LiDAR points to the image plane
    projected_points = project_lidar_to_image(lidar_points, R, tvec, K, D)
    
    # Draw the projected points on the image
    for pt in projected_points:
        # Convert the point to integer coordinates
        center = (int(pt[0][0]), int(pt[0][1]))
        cv2.circle(image, center, 5, (0, 0, 255), -1)
    
    # Save the image with projected points
    cv2.imwrite("projected_points.jpg", image)

    # Print the results
    print("Rotation Matrix (R):\n", R)
    print("Translation Vector (tvec):\n", tvec)

if __name__ == "__main__":
    main()
