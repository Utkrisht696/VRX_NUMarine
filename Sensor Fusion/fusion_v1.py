import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import cv2
import numpy as np
from sensor_msgs_py import point_cloud2
from scipy.spatial import KDTree
from sklearn.linear_model import RANSACRegressor
from sklearn.preprocessing import PolynomialFeatures

class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_node')

        # Initialize subscribers
        self.image_sub = self.create_subscription(Image, '/flir_camera/image_raw', self.image_callback, 10)
        self.lidar_sub = self.create_subscription(PointCloud2, '/lidar_points', self.lidar_callback, 10)

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Storage for the latest messages
        self.latest_image = None
        self.latest_lidar_points = None

        # Chessboard size (adjust this based on your chessboard)
        self.chessboard_size = (10, 7)  # (number_of_inner_corners_per_row, number_of_inner_corners_per_column)

    def image_callback(self, msg):
        # Convert ROS image message to OpenCV image
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.get_logger().info("Received image")

        # Display the image for visual confirmation
        cv2.imshow('Received Image', self.latest_image)
        cv2.waitKey(1)

        # Attempt to process calibration if we have both image and LiDAR data
        if self.latest_lidar_points is not None:
            self.process_calibration()

    def lidar_callback(self, msg):
        # Extract point cloud data from the LiDAR message
        self.latest_lidar_points = []
        for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            self.latest_lidar_points.append([point[0], point[1], point[2]])
        self.latest_lidar_points = np.array(self.latest_lidar_points)
        self.get_logger().info(f"Received LiDAR point cloud with {len(self.latest_lidar_points)} points")

        # Attempt to process calibration if we have both image and LiDAR data
        if self.latest_image is not None:
            self.process_calibration()

    def process_calibration(self):
        self.get_logger().info("Starting calibration process...")
        
        # Convert the image to grayscale
        gray_image = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2GRAY)

        # Find the chessboard corners in the camera image
        ret, corners = cv2.findChessboardCorners(gray_image, self.chessboard_size, None)
        if ret:
            self.get_logger().info("Chessboard detected in camera image")

            # Refine corner locations
            corners = cv2.cornerSubPix(gray_image, corners, (11, 11), (-1, -1),
                                       criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))

            # Draw corners on the image for visualization
            cv2.drawChessboardCorners(self.latest_image, self.chessboard_size, corners, ret)
            cv2.imshow('Chessboard Corners', self.latest_image)
            cv2.waitKey(1)

            # Match 2D image corners with corresponding 3D LiDAR points
            object_points = self.match_lidar_to_corners(corners)

            if object_points is not None:
                # Estimate the extrinsic parameters
                self.estimate_extrinsics(corners, object_points)
        else:
            self.get_logger().info("Chessboard not found in image")

    def match_lidar_to_corners(self, corners):
        """
        Match 2D corners to 3D points in the LiDAR point cloud.
        This method assumes that the chessboard lies on a flat plane and that the LiDAR points can be approximated to this plane.
        """
        if self.latest_lidar_points is None:
            self.get_logger().error("No LiDAR points available for matching.")
            return None

        # Step 1: Estimate the plane of the chessboard in the LiDAR point cloud
        plane_normal, plane_d = self.estimate_plane(self.latest_lidar_points)
        if plane_normal is None:
            self.get_logger().error("Failed to estimate plane from LiDAR point cloud.")
            return None

        # Step 2: Project each 2D corner into 3D space using the plane
        lidar_points_on_plane = []
        for corner in corners:
            # Convert the corner from image coordinates to 3D space on the plane
            corner_3d = self.project_corner_to_plane(corner, plane_normal, plane_d)
            lidar_points_on_plane.append(corner_3d)

        lidar_points_on_plane = np.array(lidar_points_on_plane)

        # Step 3: Use KDTree to find the closest LiDAR points to the projected points
        kd_tree = KDTree(self.latest_lidar_points)
        _, indices = kd_tree.query(lidar_points_on_plane)

        # Step 4: Extract the corresponding 3D points from the LiDAR point cloud
        corresponding_lidar_points = self.latest_lidar_points[indices]

        self.get_logger().info(f"Matched {len(corners)} 2D corners with 3D LiDAR points")

        return corresponding_lidar_points

    def estimate_plane(self, points):
        """
        Estimate the plane equation from a set of 3D points using RANSAC.
        Returns the normal vector and the plane's D coefficient in the plane equation: Ax + By + Cz + D = 0.
        """
        # Use RANSAC to fit a plane to the 3D points
        X = points[:, :2]  # Use only X and Y coordinates for plane fitting
        y = points[:, 2]   # Z coordinates

        # Polynomial features to fit a plane
        poly = PolynomialFeatures(degree=1)
        X_poly = poly.fit_transform(X)

        # Fit the RANSAC regressor to the points
        ransac = RANSACRegressor()
        ransac.fit(X_poly, y)

        # The plane equation coefficients
        coeff = ransac.estimator_.coef_
        intercept = ransac.estimator_.intercept_

        # Normal vector of the plane
        normal = np.array([-coeff[1], -coeff[2], 1.0])
        normal = normal / np.linalg.norm(normal)

        # Plane constant D
        d = -intercept

        self.get_logger().info(f"Estimated plane with normal {normal} and D = {d}")

        return normal, d

    def project_corner_to_plane(self, corner, plane_normal, plane_d):
        """
        Project a 2D corner point onto a 3D plane in the LiDAR point cloud.
        Returns the corresponding 3D point on the plane.
        """
        # Camera intrinsic parameters (use your actual calibration data)
        camera_matrix = np.array([[2.50806326e+03, 0.00000000e+00, 1.49810605e+03],
                                  [0.00000000e+00, 2.51360955e+03, 9.78662837e+02],
                                  [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

        # Convert the corner from pixel coordinates to normalized camera coordinates
        corner_normalized = np.linalg.inv(camera_matrix).dot(np.append(corner[0], 1))

        # Calculate the intersection of the ray from the camera to the corner with the plane
        t = -plane_d / np.dot(plane_normal, corner_normalized)
        point_on_plane = t * corner_normalized

        return point_on_plane

    def estimate_extrinsics(self, corners, object_points):
        # Camera intrinsic parameters (use your actual calibration data)
        camera_matrix = np.array([[2.50806326e+03, 0.00000000e+00, 1.49810605e+03],
                                  [0.00000000e+00, 2.51360955e+03, 9.78662837e+02],
                                  [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

        # Distortion coefficients (use your actual calibration data)
        dist_coeffs = np.array([-0.15637766, 0.01454216, 0.00044572, 0.00074238, 0.13338084])

        # SolvePnP to get the rotation and translation vectors
        success, rotation_vector, translation_vector = cv2.solvePnP(object_points, corners, camera_matrix, dist_coeffs)
        if success:
            # Convert the rotation vector to a rotation matrix
            rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
            self.get_logger().info(f"Rotation Matrix:\n{rotation_matrix}")
            self.get_logger().info(f"Translation Vector:\n{translation_vector}")

            # Optionally, you can project the LiDAR points back onto the image to validate the calibration
            self.project_lidar_to_image(rotation_matrix, translation_vector)
        else:
            self.get_logger().info("Failed to estimate the pose")

    def project_lidar_to_image(self, rotation_matrix, translation_vector):
        # Project the 3D LiDAR points onto the 2D image plane using the estimated extrinsics
        camera_matrix = np.array([[2.50806326e+03, 0.00000000e+00, 1.49810605e+03],
                                  [0.00000000e+00, 2.51360955e+03, 9.78662837e+02],
                                  [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
        dist_coeffs = np.array([-0.15637766, 0.01454216, 0.00044572, 0.00074238, 0.13338084])

        # Project points using the rotation and translation matrices
        projected_points, _ = cv2.projectPoints(self.latest_lidar_points, rotation_matrix, translation_vector,
                                                camera_matrix, dist_coeffs)

        # Convert projected points to integers and filter out invalid points
        valid_points = []
        for point in projected_points:
            x, y = point[0][0], point[0][1]
            if np.isfinite(x) and np.isfinite(y):
                x, y = int(x), int(y)
                if 0 <= x < self.latest_image.shape[1] and 0 <= y < self.latest_image.shape[0]:
                    valid_points.append((x, y))

        # Draw the valid projected points on the image
        for x, y in valid_points:
            cv2.circle(self.latest_image, (x, y), 5, (0, 255, 0), -1)

        # Create and resize the window
        window_name = 'Projected LiDAR Points'
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 500, 400)

        # Show the image in the resized window
        cv2.imshow(window_name, self.latest_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
