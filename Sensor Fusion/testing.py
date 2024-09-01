import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge
from sensor_msgs_py import point_cloud2 as pc2  # Correct import for ROS 2

def pointcloud2_to_xyz_array(cloud_msg):
    # Convert the point cloud message to a numpy array
    points = []
    for point in pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
        points.append([point[0], point[1], point[2]])
    return np.array(points, dtype=np.float32)

class PnPNode(Node):
    def __init__(self):
        super().__init__('pnp_node')
        self.bridge = CvBridge()

        # Subscribing to the LiDAR and Camera topics
        self.lidar_sub = self.create_subscription(PointCloud2, '/lidar_points', self.lidar_callback, 10)
        self.image_sub = self.create_subscription(Image, '/flir_camera/image_raw', self.image_callback, 10)

        # Placeholders for incoming data
        self.lidar_points_3D = None
        self.image_points_2D = None

    def lidar_callback(self, msg):
        # Convert PointCloud2 to numpy array
        self.lidar_points_3D = pointcloud2_to_xyz_array(msg)
        self.get_logger().info(f"Received LiDAR data with {len(self.lidar_points_3D)} points.")
        self.try_run_pnp()

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # Extract 2D points from the image using ORB feature detection
        self.image_points_2D = self.extract_2D_points(image)
        self.get_logger().info(f"Extracted {len(self.image_points_2D)} image points.")
        self.try_run_pnp()

    def extract_2D_points(self, image):
        # Initialize ORB detector
        orb = cv2.ORB_create()

        # Detect keypoints and compute descriptors
        keypoints, descriptors = orb.detectAndCompute(image, None)

        # Convert keypoints to a numpy array of 2D points
        image_points_2D = np.array([kp.pt for kp in keypoints], dtype=np.float32)

        # Optional: Draw keypoints on the image for visualization
        image_with_keypoints = cv2.drawKeypoints(image, keypoints, None, color=(0, 255, 0))

        # Create and resize the window
        window_name = 'Keypoints'
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 1366, 768)

        cv2.imshow(window_name, image_with_keypoints)
        cv2.waitKey(1)  # Display the image with keypoints

        return image_points_2D

    def try_run_pnp(self):
        # Ensure both LiDAR and Image data are available and have sufficient points
        if self.lidar_points_3D is not None and self.image_points_2D is not None:
            if len(self.lidar_points_3D) >= 4 and len(self.image_points_2D) >= 4:
                self.get_logger().info("Sufficient points found, running PnP...")
                self.run_pnp()
            else:
                self.get_logger().warning(f"Not enough points to run solvePnP. "
                                          f"3D points: {len(self.lidar_points_3D)}, "
                                          f"2D points: {len(self.image_points_2D)}")
        else:
            self.get_logger().warning("Waiting for LiDAR and image data to be available.")

    def run_pnp(self):
        # Ensure the data is in the correct shape
        if len(self.lidar_points_3D) != len(self.image_points_2D):
            self.get_logger().error(f"Mismatch in number of 3D and 2D points: "
                                    f"3D points: {len(self.lidar_points_3D)}, "
                                    f"2D points: {len(self.image_points_2D)}")
            return

        camera_matrix = np.array([
            [2508.06326, 0, 1498.10605],
            [0, 2513.60955, 978.662837],
            [0, 0, 1]
        ])
        dist_coeffs = np.array([-0.15637766, 0.01454216, 0.00044572, 0.00074238, 0.13338084])

        # Solve PnP to find the rotation and translation vectors
        success, rotation_vector, translation_vector = cv2.solvePnP(
            self.lidar_points_3D,
            self.image_points_2D,
            camera_matrix,
            dist_coeffs
        )

        if success:
            # Convert the rotation vector to a rotation matrix
            rotation_matrix, _ = cv2.Rodrigues(rotation_vector)

            # Construct the extrinsic matrix [R|t]
            extrinsic_matrix = np.hstack((rotation_matrix, translation_vector))

            self.get_logger().info(f'Rotation Matrix:\n{rotation_matrix}')
            self.get_logger().info(f'Translation Vector:\n{translation_vector}')
            self.get_logger().info(f'Extrinsic Matrix [R|t]:\n{extrinsic_matrix}')

        # Reset the placeholders after running PnP to avoid redundant processing
        self.lidar_points_3D = None
        self.image_points_2D = None

def main(args=None):
    rclpy.init(args=args)
    node = PnPNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()