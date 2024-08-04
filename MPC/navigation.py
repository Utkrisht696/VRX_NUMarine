import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import NavSatFix, Imu
import numpy as np
from scipy.spatial.transform import Rotation
from pyproj import Proj, Transformer

class StateEstimator(Node):

    def __init__(self):
        super().__init__('state_estimator')
        
        # Initialize state and other variables
        self.current_state = np.zeros((6, 1))
        self.P = np.diag([1000.0, 1000.0, 10.0, 0.0, 0.0, 0.0])
        self.Q = np.array([
            [ 6.1856e-02, -9.4391e-04,  1.4843e-04,  4.3312e-04, -1.2616e-04, -1.0697e-04],
            [-9.4391e-04,  6.5208e-02, -2.3349e-04,  3.8015e-04, -4.1236e-04,  3.9740e-04],
            [ 1.4843e-04, -2.3349e-04,  2.0787e-04,  5.3975e-06,  8.4772e-06,  6.3032e-05],
            [ 4.3312e-04,  3.8015e-04,  5.3975e-06,  7.9621e-06, -3.0529e-06,  3.4317e-06],
            [-1.2616e-04, -4.1236e-04,  8.4772e-06, -3.0529e-06,  5.8290e-06, -1.3472e-07],
            [-1.0697e-04,  3.9740e-04,  6.3032e-05,  3.4317e-06, -1.3472e-07,  2.4441e-05]
        ])
        self.R_gps = np.diag([0.1, 0.1])
        self.R_imu = np.diag([0.1, 0.1])

        self.proj_wgs84 = Proj(proj='latlong', datum='WGS84')
        self.proj_utm = Proj(proj='utm', zone=56, datum='WGS84', south=True)
        self.transformer = Transformer.from_proj(self.proj_wgs84, self.proj_utm)

        self.datum_x, self.datum_y = self.transformer.transform(150.6789, -33.7285)
        self.prev_yaw = None
        self.yaw_offset = 0.0

        self.last_update_time = self.get_clock().now().nanoseconds / 1e9

        self.state_pub = self.create_publisher(Float64MultiArray, '/wamv/current_state', 10)
        self.gps_sub = self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self.imu_callback, 10)
        
    def gps_to_local_xy(self, lon, lat):
        x, y = self.transformer.transform(lon, lat)
        local_x = x - self.datum_x
        local_y = y - self.datum_y
        return local_x, local_y

    def update_state(self, dt, gps_measurement=None, imu_measurement=None):
        # Prediction step (state transition)
        self.current_state, F = self.state_transition_A(self.current_state, dt, np.zeros((4, 1)))  # assuming last inputs are zeros for now
        self.P = F @ self.P @ F.T + self.Q * dt  # Scale process noise with dt

        # Update step (GPS)
        if gps_measurement is not None:
            H_gps = np.array([[1, 0, 0, 0, 0, 0],
                              [0, 1, 0, 0, 0, 0]])
            y = gps_measurement.reshape(-1,1,order='f') - H_gps @ self.current_state
            S = H_gps @ self.P @ H_gps.T + self.R_gps
            K = self.P @ H_gps.T @ np.linalg.inv(S)
            self.current_state = self.current_state + K @ y
            self.P = (np.eye(6) - K @ H_gps) @ self.P

        # Update step (IMU)
        if imu_measurement is not None:
            H_imu = np.array([[0, 0, 1, 0, 0, 0],
                              [0, 0, 0, 0, 0, 1]])
            y = imu_measurement.reshape(-1,1,order='f') - H_imu @ self.current_state
            S = H_imu @ self.P @ H_imu.T + self.R_imu
            K = self.P @ H_imu.T @ np.linalg.inv(S)
            self.current_state = self.current_state + K @ y
            self.P = (np.eye(6) - K @ H_imu) @ self.P

        # Publish the updated state
        self.publish_state()

    def state_transition_A(self, x, dt, u):
        k1 = self.CTStateModel(x, u)
        k2 = self.CTStateModel(x + (dt / 2.0) * k1, u)
        k3 = self.CTStateModel(x + (dt / 2.0) * k2, u)
        k4 = self.CTStateModel(x + dt * k3, u)

        xn = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        xn = xn.reshape(-1, 1, order='f')

        dk1dx = self.CTStateModelDx(x)
        dk2dx = self.CTStateModelDx(x + (dt / 2.0) * k1) @ (np.eye(6) + (dt / 2.0) * dk1dx)
        dk3dx = self.CTStateModelDx(x + (dt / 2.0) * k2) @ (np.eye(6) + (dt / 2.0) * dk2dx)
        dk4dx = self.CTStateModelDx(x + dt * k3) @ (np.eye(6) + dt * dk3dx)
        A = np.eye(6) + (dt / 6.0) * (dk1dx + 2.0 * dk2dx + 2.0 * dk3dx + dk4dx)

        return xn, A

    def CTStateModel(self, x, u):
        N, E, psi, u_vel, v, r = x[:,0]
        F_l  = 4*u[0,0]
        F_r  = 4*u[1,0]
        F_bl = 4*u[2,0]
        F_br = 4*u[3,0]
        
        tau_u = np.array([[0],
                          [0],
                          [0],
                          [(F_l+F_r)/252.5551],
                          [(F_br-F_bl)/251.7846],
                          [(1.2*(-F_bl+F_br) + 0.8*(F_r-F_l))/516.2194]])

        dN = u_vel * np.cos(psi) - v * np.sin(psi)
        dE = u_vel * np.sin(psi) + v * np.cos(psi)
        dpsi = r
        du = (251.7846 * v * r - (98.9777 + 150.1745*np.fabs(u_vel)) * u_vel) / 252.5551
        dv = (-252.5551 * u_vel * r - (100.4924 + 99.7235*np.fabs(v)) * v) / 251.7846
        dr = (- (806.6199 + 809.7870*np.fabs(r)) * r) / 516.2194

        tau_x = np.array([[dN],[dE],[dpsi],[du],[dv],[dr]])

        return tau_x + tau_u

    def CTStateModelDx(self, x):
        N, E, psi, u_vel, v, r = x[:,0]

        J = np.array([
            [0, 0, (-u_vel * np.sin(psi) - v * np.cos(psi)) , np.cos(psi) , -np.sin(psi) , 0],
            [0, 0, (u_vel * np.cos(psi) - v * np.sin(psi)) , np.sin(psi) , np.cos(psi) , 0],
            [0, 0, 0, 0, 0, 1.0],
            [0, 0, 0, 0 - ((98.9777 + 2.0*150.1745*np.fabs(u_vel)) / 252.5551) , (251.7846 / 252.5551) * r , (251.7846 / 252.5551) * v ],
            [0, 0, 0, -(252.5551 / 251.7846) * r , - ((100.4924 + 2.0*99.7235*np.fabs(v)) / 251.7846) , -(252.5551 / 251.7846) * u_vel ],
            [0, 0, 0, 0, 0, - ((806.6199 + 2.0*809.7870*np.fabs(r)) / 516.2194)]
        ])
        
        return J

    def gps_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_update_time
        self.last_update_time = current_time

        x, y = self.gps_to_local_xy(msg.longitude, msg.latitude)
        self.update_state(dt, gps_measurement=np.array([x, y]))

    def imu_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_update_time
        self.last_update_time = current_time

        quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        euler = Rotation.from_quat(quat).as_euler('xyz')
        yaw = euler[2]
        angular_velocity = msg.angular_velocity.z

        if self.prev_yaw is not None:
            diff = yaw - self.prev_yaw
            if diff > np.pi:
                self.yaw_offset -= 2.0 * np.pi
            elif diff < -np.pi:
                self.yaw_offset += 2.0 * np.pi

        unwrapped_yaw = yaw + self.yaw_offset
        self.prev_yaw = yaw

        self.update_state(dt, imu_measurement=np.array([unwrapped_yaw, angular_velocity]))

    def publish_state(self):
        msg = Float64MultiArray()
        msg.data = self.current_state.flatten().tolist()
        self.state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    estimator = StateEstimator()
    rclpy.spin(estimator)
    estimator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
