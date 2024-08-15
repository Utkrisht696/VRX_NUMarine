import sys
import os
import time
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray, Twist
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float64, Float64MultiArray
import casadi as ca
import numpy as np
from scipy.spatial.transform import Rotation
from pyproj import Proj, Transformer
from qpsolvers import Problem, solve_problem
from scipy.linalg import block_diag
class WAMV_Navigation(Node):

    def __init__(self, config):
        super().__init__('wamv_navigation')

        # Load model parameters
        self.m11 = config['model_parameters']['m11']
        self.m22 = config['model_parameters']['m22']
        self.m33 = config['model_parameters']['m33']
        self.d11 = config['model_parameters']['d11']
        self.d22 = config['model_parameters']['d22']
        self.d33 = config['model_parameters']['d33']
        self.d11_2 = config['model_parameters']['d11_2']
        self.d22_2 = config['model_parameters']['d22_2']
        self.d33_2 = config['model_parameters']['d33_2']
        self.d_m = config['model_parameters']['d_m']
        self.L_m = config['model_parameters']['L_m']
        self.d_b = config['model_parameters']['d_b']
        self.L_b = config['model_parameters']['L_b']
        
        # Load NMPC parameters
        self.Np = config['nmpc_parameters']['Np']
        self.Nu = config['nmpc_parameters']['Nu']
        self.dt = config['nmpc_parameters']['dt']
        self.nu = config['nmpc_parameters']['nu']
        self.nx = config['nmpc_parameters']['nx']
        
        # Initialize state and reference
        self.current_state = np.zeros((self.nx,1))
       
        # Last known actuator inputs
        self.last_inputs = np.zeros((self.nu,1))

        # Load EKF parameters
        self.P = np.diag(config['ekf_parameters']['initial_state_covariance'])
        self.Q = np.array(config['ekf_parameters']['process_noise_covariance'])
        self.R_gps = np.diag(config['ekf_parameters']['gps_measurement_noise_covariance'])
        self.R_imu = np.diag(config['ekf_parameters']['imu_measurement_noise_covariance'])

        self.beta = config['ekf_parameters']['beta']
        self.G = np.vstack((np.eye(self.nu),-np.eye(self.nu)))
        self.h = self.beta * np.ones((2*self.nu*self.Nu,1))
        I = np.eye(self.nu)
        Z = np.zeros(self.nu)
        mat = np.block([[-I, I],[I, -I]])
        for i in range(self.Nu-1):
            self.G = np.block([[self.G,np.zeros((self.G.shape[0],self.nu))],[np.zeros((2*self.nu,self.G.shape[1]-self.nu)),mat]])


        # Load datum coordinates
        self.datum_lat = config['datum_coordinates']['lat']
        self.datum_lon = config['datum_coordinates']['lon']

        # Initialize projections
        self.proj_wgs84 = Proj(proj='latlong', datum='WGS84')
        self.proj_utm = Proj(proj=config['projections']['proj'], zone=config['projections']['zone'], datum=config['projections']['datum'], south=config['projections']['south'])
        self.transformer = Transformer.from_proj(self.proj_wgs84, self.proj_utm)

        # Calculate datum in UTM coordinates
        self.datum_x, self.datum_y = self.transformer.transform(self.datum_lon, self.datum_lat)
        
        # Counters for number of gps and imu updates
        self.gpsUpdates = 0
        self.imuUpdates = 0
        self.prev_yaw = None
        self.yaw_offset = 0.0
        self.headingOld = 0.0
        self.currentwaypoint = 0

        # Timestamp of last update
        self.last_update_time = self.get_clock().now().nanoseconds / 1e9
        
        # ROS2 publishers and subscribers
        self.state_pub = self.create_publisher(Float64MultiArray, '/wamv/current_state', 10)
        self.gps_sub = self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self.imu_callback, 10)


    def publish_current_state(self):
        msg = Float64MultiArray()
        msg.data = self.current_state.flatten().tolist()  # Convert the state to a flat list
        self.state_pub.publish(msg)
        

    def CTStateModel(self, x, u):
        # Unpack state and inputs
        N, E, psi, u_vel, v, r = x[:,0]
        F_l  = 4*u[0,0]
        F_r  = 4*u[1,0]
        F_bl = 4*u[2,0]
        F_br = 4*u[3,0]
        
        tau_u = np.array([[0],
                          [0],
                          [0],
                          [(F_l+F_r)/self.m11],
                          [(F_br-F_bl)/self.m22],
                          [(self.L_b*(-F_bl+F_br) + self.d_m*(F_r-F_l))/self.m33]])

        # Compute state derivatives
        dN = u_vel * np.cos(psi) - v * np.sin(psi)
        dE = u_vel * np.sin(psi) + v * np.cos(psi)
        dpsi = r
        du = (self.m22 * v * r - (self.d11 + self.d11_2*np.fabs(u_vel)) * u_vel) / self.m11
        dv = (-self.m11 * u_vel * r - (self.d22 + self.d22_2*np.fabs(v)) * v) / self.m22
        dr = (- (self.d33 + self.d33_2*np.fabs(r)) * r) / self.m33

        tau_x = np.array([[dN],[dE],[dpsi],[du],[dv],[dr]])

        return tau_x + tau_u

    def CTStateModelDx(self, x):
        # Unpack state
        N, E, psi, u_vel, v, r = x[:,0]

        # Compute Jacobian
        J = np.array([
            [0, 0, (-u_vel * np.sin(psi) - v * np.cos(psi)) , np.cos(psi) , -np.sin(psi) , 0],
            [0, 0, (u_vel * np.cos(psi) - v * np.sin(psi)) , np.sin(psi) , np.cos(psi) , 0],
            [0, 0, 0, 0, 0, 1.0],
            [0, 0, 0, 0 - ((self.d11 + 2.0*self.d11_2*np.fabs(u_vel)) / self.m11) , (self.m22 / self.m11) * r , (self.m22 / self.m11) * v ],
            [0, 0, 0, -(self.m11 / self.m22) * r , - ((self.d22 + 2.0*self.d22_2*np.fabs(v)) / self.m22) , -(self.m11 / self.m22) * u_vel ],
            [0, 0, 0, 0, 0, - ((self.d33 + 2.0*self.d33_2*np.fabs(r)) / self.m33)]
        ])
        
        return J

    def state_transition_A(self, x, dt, u):
        k1 = self.CTStateModel(x,u)
        k2 = self.CTStateModel(x+(dt/2.0)*k1,u)
        k3 = self.CTStateModel(x+(dt/2.0)*k2,u)
        k4 = self.CTStateModel(x+dt*k3,u)

        xn = x + (dt/6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4)
        xn = xn.reshape(-1,1,order='f')

        dk1dx = self.CTStateModelDx(x)
        dk2dx = self.CTStateModelDx(x+(dt/2.0)*k1) @ (np.eye(6)+(dt/2.0)*dk1dx)
        dk3dx = self.CTStateModelDx(x+(dt/2.0)*k2) @ (np.eye(6)+(dt/2.0)*dk2dx)
        dk4dx = self.CTStateModelDx(x+(dt/1.0)*k3) @ (np.eye(6)+(dt/1.0)*dk3dx)
        A = np.eye(6) + (dt/6.0) * (dk1dx + 2.0*dk2dx + 2.0*dk3dx + dk4dx)
        
        return xn, A
    
    def state_transition_xonly(self, x, u, dt):
        k1 = self.CTStateModel(x,u)
        k2 = self.CTStateModel(x+(dt/2.0)*k1,u)
        k3 = self.CTStateModel(x+(dt/2.0)*k2,u)
        k4 = self.CTStateModel(x+dt*k3,u)

        xn = x + (dt/6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4)
        return xn.reshape(-1,1,order='f')

    def state_transition_U(self, x, u, dt, dxdu,k):
        k1 = self.CTStateModel(x,u)
        k2 = self.CTStateModel(x+(dt/2.0)*k1,u)
        k3 = self.CTStateModel(x+(dt/2.0)*k2,u)
        k4 = self.CTStateModel(x+dt*k3,u)

        xn = x + (dt/6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4)
        xn = xn.reshape(-1,1,order='f')

        # tau_u = np.array([[0],
        #                   [0],
        #                   [0],
        #                   [(F_l+F_r)/self.m11],
        #                   [(F_br-F_bl)/self.m22],
        #                   [(self.L_b*(-F_bl+F_br) + self.d_m*(F_r-F_l))/self.m33]])

        dFdu = 4 * np.eye(4)

        dxdF = np.array([
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [1/self.m11, 1/self.m11, 0, 0],
            [0, 0, -1/self.m22, 1/self.m22],
            [-self.d_m/self.m33, self.d_m/self.m33, -self.L_b/self.m33, self.L_b/self.m33]
        ])

        B = dxdF @ dFdu

        dk1du = self.CTStateModelDx(x) @ dxdu
        if k<self.Nu:
            dk1du[:,k*self.nu:(k+1)*self.nu] += B
        # else:
        #     dk1du[:,(self.Nu-1)*self.nu:(self.Nu)*self.nu] += B
        dk2du = self.CTStateModelDx(x+(dt/2.0)*k1) @ (dxdu+(dt/2.0)*dk1du)
        if k<self.Nu:
            dk2du[:,k*self.nu:(k+1)*self.nu] += B
        # else:
        #     dk2du[:,(self.Nu-1)*self.nu:(self.Nu)*self.nu] += B
        dk3du = self.CTStateModelDx(x+(dt/2.0)*k2) @ (dxdu+(dt/2.0)*dk2du)
        if k<self.Nu:
            dk3du[:,k*self.nu:(k+1)*self.nu] += B
        # else:
        #     dk3du[:,(self.Nu-1)*self.nu:(self.Nu)*self.nu] += B
        dk4du = self.CTStateModelDx(x+(dt/1.0)*k3) @ (dxdu+(dt/1.0)*dk3du)
        if k<self.Nu:
            dk4du[:,k*self.nu:(k+1)*self.nu] += B
        # else:
        #     dk4du[:,(self.Nu-1)*self.nu:(self.Nu)*self.nu] += B
        dxdun = dxdu + (dt/6.0) * (dk1du + 2.0*dk2du + 2.0*dk3du + dk4du)
        
        return xn, dxdun

    def gps_to_local_xy(self, lon, lat):
        # Convert GPS coordinates to UTM
        x, y = self.transformer.transform(lon, lat)
        
        # Calculate local x-y relative to the datum
        local_x = x - self.datum_x
        local_y = y - self.datum_y
        
        return local_x, local_y

    def gps_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_update_time
        self.last_update_time = current_time

        # Convert GPS to local x-y coordinates
        x, y = self.gps_to_local_xy(msg.longitude, msg.latitude)
        
        self.update_ekf(dt, gps_measurement=np.array([x, y]))

        if self.gpsUpdates < 300:
            self.gpsUpdates += 1

    def imu_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_update_time
        self.last_update_time = current_time

        quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        euler = Rotation.from_quat(quat).as_euler('xyz')
        yaw = euler[2]
        angular_velocity = msg.angular_velocity.z
        
        if self.prev_yaw is not None:
            # Check for wrap-around
            diff = yaw - self.prev_yaw
            if diff > np.pi:
                self.yaw_offset -= 2.0 * np.pi
            elif diff < -np.pi:
                self.yaw_offset += 2.0 * np.pi

        unwrapped_yaw = yaw + self.yaw_offset
        self.prev_yaw = yaw


        self.update_ekf(dt, imu_measurement=np.array([unwrapped_yaw, angular_velocity]))

        if self.imuUpdates < 300:
            self.imuUpdates += 1

    def update_ekf(self, dt, gps_measurement=None, imu_measurement=None):
        # Prediction step
        self.current_state, F = self.state_transition_A(self.current_state, dt, self.last_inputs)
        self.P = F @ self.P @ F.T + self.Q * dt  # Scale process noise with dt

        # Update step
        if gps_measurement is not None:
            H_gps = np.array([[1, 0, 0, 0, 0, 0],
                              [0, 1, 0, 0, 0, 0]])
            y = gps_measurement.reshape(-1,1,order='f') - H_gps @ self.current_state
            S = H_gps @ self.P @ H_gps.T + self.R_gps
            K = self.P @ H_gps.T @ np.linalg.inv(S)
            self.current_state = self.current_state + K @ y
            self.P = (np.eye(6) - K @ H_gps) @ self.P

        if imu_measurement is not None:
            H_imu = np.array([[0, 0, 1, 0, 0, 0],
                              [0, 0, 0, 0, 0, 1]])
            y = imu_measurement.reshape(-1,1,order='f') - H_imu @ self.current_state
            S = H_imu @ self.P @ H_imu.T + self.R_imu
            K = self.P @ H_imu.T @ np.linalg.inv(S)
            self.current_state = self.current_state + K @ y
            self.P = (np.eye(6) - K @ H_imu) @ self.P
        self.publish_current_state()

def main(args=None):

    # Set the working directory to the script's directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)

    rclpy.init(args=args)
    with open('config.yaml', 'r') as file:
        config = yaml.safe_load(file)
    navigation = WAMV_Navigation(config)
    rclpy.spin(navigation)
    navigation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
