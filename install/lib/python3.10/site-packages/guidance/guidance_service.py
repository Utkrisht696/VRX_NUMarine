import rclpy
from rclpy.node import Node
from guidance.srv import Trajectory
import numpy as np
from scipy.interpolate import CubicSpline

# Constants for converting meters to latitude and longitude
METERS_PER_DEGREE_LATITUDE = 111320  # Approximate meters per degree of latitude
METERS_PER_DEGREE_LONGITUDE = 111320 * np.cos(np.radians(-33.72276913511836))  # Adjust for the given latitude

class GuidanceService(Node):
    def __init__(self):
        super().__init__('guidance_service')
        self.srv = self.create_service(Trajectory, 'trajectory', self.trajectory_callback)

    def trajectory_callback(self, request, response):
        times = np.array(request.times)
        north_positions = np.array(request.north_positions)
        east_positions = np.array(request.east_positions)

        n_points = len(times)
        assert len(north_positions) == n_points, "Number of north positions must match number of times"
        assert len(east_positions) == n_points, "Number of east positions must match number of times"

        # Convert north and east positions to latitude and longitude
        origin_lat = -33.72276913511836
        origin_lon = 150.67398721748103
        origin_alt = 1.1486131474375725

        latitudes = origin_lat + north_positions / METERS_PER_DEGREE_LATITUDE
        longitudes = origin_lon + east_positions / METERS_PER_DEGREE_LONGITUDE
        altitudes = np.full_like(latitudes, origin_alt)  # Assuming constant altitude for simplicity

        waypoints = np.vstack((latitudes, longitudes, altitudes))

        # Create splines for latitude, longitude, and altitude
        cs_lat = CubicSpline(times, waypoints[0, :])
        cs_lon = CubicSpline(times, waypoints[1, :])
        cs_alt = CubicSpline(times, waypoints[2, :])

        # Define time steps for the evaluation
        eval_times = np.linspace(times[0], times[-1], num=100)  # 100 points for the trajectory

        trajectory = []
        lat_velocities = []
        lon_velocities = []
        alt_velocities = []

        for t in eval_times:
            lat_pos = cs_lat(t)
            lon_pos = cs_lon(t)
            alt_pos = cs_alt(t)
            lat_vel = cs_lat(t, 1)
            lon_vel = cs_lon(t, 1)
            alt_vel = cs_alt(t, 1)
            trajectory.append([lat_pos, lon_pos, alt_pos])
            lat_velocities.append(lat_vel)
            lon_velocities.append(lon_vel)
            alt_velocities.append(alt_vel)

        response.latitudes = [point[0] for point in trajectory]
        response.longitudes = [point[1] for point in trajectory]
        response.altitudes = [point[2] for point in trajectory]
        response.lat_velocities = lat_velocities
        response.lon_velocities = lon_velocities
        response.alt_velocities = alt_velocities

        return response

def main(args=None):
    rclpy.init(args=args)
    guidance_service = GuidanceService()
    rclpy.spin(guidance_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
