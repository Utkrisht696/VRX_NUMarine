import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import cv2
import numpy as np
import struct

class ChessboardDistanceNode(Node):
    def __init__(self):
        super().__init__('chessboard_distance_node')
        
        self.bridge = CvBridge()
        
        # Camera intrinsic parameters (from calibration)
        self.K = np.array([[2585.8849514873077, 0.0, 1495.8212207667002],
                           [0.0, 2594.1433908794033, 973.2642905461533],
                           [0.0, 0.0, 1.0]])
        self.D = np.array([-0.1950356958649444, 0.1383504196472886, 0.0002563441895758756, -0.0003855201794363196, 0.0])
        
        # Transformation matrix from LiDAR frame to camera frame
        self.T = np.array([
            [1, 0, 0, -1.1],
            [0, 1, 0, 0.0],
            [0, 0, 1, 0.9],
            [0, 0, 0, 1]
        ])
        
        # Chessboard parameters
        self.chessboard_size = (10, 7)  # 10x7 chessboard
        self.square_size = 0.100  # Size of a square in meters

        # Subscribe to camera image and LiDAR point cloud
        self.image_sub = self.create_subscription(Image, '/flir_camera/image_raw', self.image_callback, 10)
        self.lidar_sub = self.create_subscription(PointCloud2, '/lidar_points', self.lidar_callback, 10)

        # Storage for latest image and point cloud data
        self.latest_corners = None
        self.latest_image = None
        self.latest_pointcloud = None

        # Create window for display
        cv2.namedWindow('Chessboard Detection', cv2.WINDOW_NORMAL)

    def image_callback(self, msg):
        self.get_logger().info('Received image')
        
        try:
            # Convert the ROS Image message to a raw OpenCV Bayer image
            bayer_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

            # Convert Bayer RGGB to RGB
            cv_image = cv2.cvtColor(bayer_image, cv2.COLOR_BayerBG2BGR)

            # Rectify the image
            h, w = cv_image.shape[:2]
            new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(self.K, self.D, (w, h), 1, (w, h))
            rectified_image = cv2.undistort(cv_image, self.K, self.D, None, new_camera_matrix)

            # Convert to grayscale for chessboard detection
            gray_image = cv2.cvtColor(rectified_image, cv2.COLOR_BGR2GRAY)

            # Detect chessboard corners
            ret, corners = cv2.findChessboardCorners(gray_image, self.chessboard_size, None)
            
            if ret:
                self.get_logger().info('Chessboard detected')
                # Refine the corner detection for better accuracy
                corners = cv2.cornerSubPix(gray_image, corners, (11, 11), (-1, -1), 
                                           criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
                
                # Draw the corners on the image
                cv2.drawChessboardCorners(rectified_image, self.chessboard_size, corners, ret)
                self.latest_corners = corners.reshape(-1, 2)
            else:
                self.get_logger().info('Chessboard not detected')
                self.latest_corners = None

            # Display the image regardless of chessboard detection
            self.latest_image = rectified_image.copy()
            cv2.imshow('Chessboard Detection', rectified_image)
            cv2.resizeWindow('Chessboard Detection', 800, 600)  # Resize window to 800x600
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Failed to process image: {e}')

    def lidar_callback(self, msg):
        self.get_logger().info('Received LiDAR point cloud')
        self.latest_pointcloud = msg

        if self.latest_corners is not None and self.latest_image is not None:
            try:
                # Convert PointCloud2 to numpy array manually
                points = self.pointcloud2_to_xyz_array(msg)

                # Transform LiDAR points to the camera frame
                points_camera_frame = self.transform_points_to_camera_frame(points)

                # Project points onto the image plane
                image_points = self.project_points_to_image_plane(points_camera_frame)

                # Debugging: Log the first few projected points to inspect
                self.get_logger().info(f'First few projected points: {image_points[:5]}')

                # Overlay points on the image
                for point in image_points:
                    point = tuple(int(coord) for coord in point)  # Ensure the point is a tuple of integers
                    self.get_logger().info(f'Drawing point at: {point}')  # Log the point being drawn
                    cv2.circle(self.latest_image, point, 3, (0, 0, 255), -1)

                # Display the image with projected LiDAR points
                cv2.imshow('Chessboard Detection', self.latest_image)
                cv2.waitKey(1)

            except Exception as e:
                self.get_logger().error(f'Failed to process LiDAR data: {e}')

    def pointcloud2_to_xyz_array(self, cloud_msg):
        """Convert a ROS PointCloud2 message to a numpy array of shape (n, 3) for x, y, z."""
        points = []
        for point in self.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([point[0], point[1], point[2]])
        return np.array(points)

    def read_points(self, cloud, field_names=None, skip_nans=False, uvs=[]):
        """Generator to yield points from a PointCloud2 message."""
        fmt = 'fff'  # Format for x, y, z float32
        width, height, point_step, row_step, data, is_bigendian = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data, cloud.is_bigendian
        unpack_from = struct.Struct(fmt).unpack_from

        for v in range(height):
            for u in range(width):
                point_offset = row_step * v + point_step * u
                x, y, z = unpack_from(data[point_offset:point_offset + 12])
                if skip_nans and (np.isnan(x) or np.isnan(y) or np.isnan(z)):
                    continue
                yield (x, y, z)

    def transform_points_to_camera_frame(self, points):
        """Transform LiDAR points to the camera frame."""
        points_homogeneous = np.hstack((points, np.ones((points.shape[0], 1))))
        points_camera_frame = np.dot(self.T, points_homogeneous.T).T[:, :3]
        return points_camera_frame

    def project_points_to_image_plane(self, points_camera_frame):
        """Project 3D points in the camera frame onto the 2D image plane."""
        image_points, _ = cv2.projectPoints(points_camera_frame, np.zeros((3, 1)), np.zeros((3, 1)), self.K, self.D)
        return image_points.reshape(-1, 2)

def main(args=None):
    rclpy.init(args=args)
    node = ChessboardDistanceNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
