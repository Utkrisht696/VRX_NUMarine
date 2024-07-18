import rclpy
from rclpy.node import Node
import numpy as np
from scipy.interpolate import CubicSpline
from std_msgs.msg import Float64MultiArray

class GuidanceService(Node):
    def __init__(self):
        super().__init__('guidance_service')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/guidance_service/output', 10)
        self.set_waypoints()

    def set_waypoints(self):
        times = np.array([0.0, 15.0, 30.0])  # Example times
        north_positions = np.array([0.0, 5.0, 10.0])  # Example north positions in meters
        east_positions = np.array([0.0, 0.0, 0.0])  # Example east positions in meters
        north_velocities = np.array([0.0, 1/3, 0.0])  # Example north velocities in meters per second
        east_velocities = np.array([0.0, 0.0, 0.0])  # Example east velocities in meters per second

        # Fit cubic splines with specified derivatives
        spline_north = CubicSpline(times, north_positions, bc_type=((1, north_velocities[0]), (1, north_velocities[-1])))
        spline_east = CubicSpline(times, east_positions, bc_type=((1, east_velocities[0]), (1, east_velocities[-1])))

        # Evaluate splines at desired times for debugging
        eval_times = np.linspace(times.min(), times.max(), 100)
        eval_north_positions = spline_north(eval_times)
        eval_north_velocities = spline_north(eval_times, 1)
        eval_east_positions = spline_east(eval_times)
        eval_east_velocities = spline_east(eval_times, 1)

        # Log evaluated points
        for t, n, e, nv, ev in zip(eval_times, eval_north_positions, eval_east_positions, eval_north_velocities, eval_east_velocities):
            self.get_logger().info(f'Time: {t:.2f}, N: {n:.2f}, E: {e:.2f}, N Vel: {nv:.2f}, E Vel: {ev:.2f}')

        # Publish spline coefficients
        spline_msg = Float64MultiArray()
        spline_msg.data = np.concatenate([spline_north.c.ravel(), spline_east.c.ravel()]).tolist()
        self.publisher_.publish(spline_msg)

def main(args=None):
    rclpy.init(args=args)
    guidance_service = GuidanceService()
    rclpy.spin(guidance_service)
    guidance_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
