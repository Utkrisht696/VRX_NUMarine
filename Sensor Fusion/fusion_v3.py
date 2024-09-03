import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge
import numpy as np
import cv2
import sensor_msgs_py.point_cloud2 as pc2
from scipy.spatial import KDTree

class SensorFusionNode(Node):

    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Subscriptions
        self.lidar_sub = self.create_subscription(
            PointCloud2, 
            '/lidar_points', 
            self.lidar_callback, 
            10
        )
        
        self.image_sub = self.create_subscription(
            Image, 
            '/flir_camera/image_raw', 
            self.image_callback, 
            10
        )

        self.bridge = CvBridge()  # Bridge to convert ROS Image message to OpenCV format
        self.lidar_points = None
        self.image = None
        self.image_points = None  # Initialize image_points to hold extracted 2D points

        # Intrinsic camera matrix (K) and distortion coefficients (D)
        self.camera_matrix = np.array([[2585.8849514873077, 0.0, 1495.8212207667002],
              [0.0, 2594.1433908794033, 973.2642905461533],
              [0.0, 0.0, 1.0]])
        
        self.dist_coeffs = np.array([-0.1950356958649444, 0.1383504196472886, 0.0002563441895758756, -0.0003855201794363196, 0.0])

    def lidar_callback(self, msg):
        # Process the PointCloud2 message to extract LiDAR points
        self.lidar_points = self.process_point_cloud(msg)
        if self.lidar_points is not None and self.image is not None:
            self.extract_image_points()  # Extract points from image before computing extrinsics
            self.compute_extrinsic_matrix()

    def image_callback(self, msg):
        # Convert the ROS Image message to a format usable by OpenCV
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.lidar_points is not None and self.image is not None:
            self.extract_image_points()  # Extract points from image before computing extrinsics
            self.compute_extrinsic_matrix()

    def process_point_cloud(self, msg):
        # Convert the PointCloud2 message to a numpy array
        points_list = []
        for point in pc2.read_points(msg, skip_nans=True):
            points_list.append([point[0], point[1], point[2]])  # Append x, y, z coordinates

        point_cloud = np.array(points_list, dtype=np.float32)
        self.get_logger().info(f"Processed {len(point_cloud)} points from PointCloud2 message.")
        return point_cloud

    def extract_image_points(self):
        # This function should extract 2D feature points from the image
        gray_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        self.image_points = cv2.goodFeaturesToTrack(gray_image, maxCorners=100, qualityLevel=0.01, minDistance=10)

        if self.image_points is not None:
            self.image_points = self.image_points.reshape(-1, 2)  # Reshape if needed

        # Log the number of points detected in the image
        if self.image_points is not None:
            self.get_logger().info(f"Detected {len(self.image_points)} image points.")
        else:
            self.get_logger().error("No image points detected.")

    def compute_extrinsic_matrix(self):
        # Ensure that image_points have been extracted and there are at least 4 points
        if self.image_points is None or self.lidar_points is None:
            self.get_logger().error("Missing image points or LiDAR points.")
            return

        # Ensure that both lidar_points and image_points have sufficient number of points
        if len(self.lidar_points) < 4 or len(self.image_points) < 4:
            self.get_logger().error(f"Insufficient points for solvePnP: {len(self.lidar_points)} 3D points, {len(self.image_points)} 2D points")
            return

        # Convert points to correct format
        lidar_points_np = np.array(self.lidar_points, dtype=np.float32)
        image_points_np = np.array(self.image_points, dtype=np.float32)

        # Project 3D LiDAR points into the 2D image plane
        projected_points_2d = np.array([self.project_3d_point(pt) for pt in lidar_points_np])

        # KD-Tree for fast nearest neighbor search in the 2D image space
        image_kd_tree = KDTree(projected_points_2d)

        selected_lidar_points = []
        selected_image_points = []

        for image_point in image_points_np:
            # Find the nearest 2D projected point for each image point
            dist, index = image_kd_tree.query(image_point)
            selected_lidar_points.append(lidar_points_np[index])
            selected_image_points.append(image_point)

        selected_lidar_points = np.array(selected_lidar_points, dtype=np.float32)
        selected_image_points = np.array(selected_image_points, dtype=np.float32)

        # Check that the number of selected points is sufficient
        if len(selected_lidar_points) < 4 or len(selected_image_points) < 4:
            self.get_logger().error("Not enough corresponding points found.")
            return

        try:
            # Solve PnP using the matched points
            retval, rvec, tvec = cv2.solvePnP(
                selected_lidar_points, 
                selected_image_points, 
                self.camera_matrix, 
                self.dist_coeffs
            )
            rotation_matrix, _ = cv2.Rodrigues(rvec)
            extrinsic_matrix = np.hstack((rotation_matrix, tvec))

            self.get_logger().info(f"Extrinsic Matrix:\n{extrinsic_matrix}")
        except cv2.error as e:
            self.get_logger().error(f"solvePnP failed: {e}")

    def project_3d_point(self, point_3d):
        # Project a 3D point to 2D using the camera intrinsic matrix
        point_3d_hom = point_3d  # No need to append 1 for homogeneous coordinate
        projected_2d_hom = np.dot(self.camera_matrix, point_3d_hom)
        projected_2d = projected_2d_hom[:2] / projected_2d_hom[2]  # Normalize to get (x, y)
        return projected_2d

def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()