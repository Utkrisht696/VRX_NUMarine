import sys
import os
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray, Twist
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float64, Float64MultiArray
import casadi as ca
import numpy as np
from scipy.spatial.transform import Rotation
from pyproj import Proj
from pyproj import Proj, Transformer
from qpsolvers import Problem, solve_problem

class WAMV_NMPC_Controller(Node):

    def __init__(self):
        super().__init__('wamv_nmpc_controller')
        
        # Model parameters
        self.m11, self.m22, self.m33 = 345.0, 256.0, 944.0
        self.d11, self.d22, self.d33 = 137.0, 79.0, 934.0
        self.d11_2, self.d22_2, self.d33_2 = 225.0, 55.0, 1057.0
        self.d_m, self.L_m = 0.8, 1.05
        self.d_b, self.L_b = 0.5, 1.2
        
        # NMPC parameters
        self.Np = 25  # Prediction horizon
        self.Nu = 15  # Control horizon        
        self.dt = 0.5  # Time step
        self.nu = 4
        self.nx = 6
        
        # Initialize state and reference
        self.current_state = np.zeros((self.nx,1))
       
        # Last known actuator inputs
        self.last_inputs = np.zeros((self.nu,1))

        # EKF parameters
        self.P = np.diag([1000.0, 1000.0, 10.0, 0.0, 0.0, 0.0]) # Initial state covariance
        self.Q = np.diag([0.1, 0.1, 0.01, 0.01, 0.01, 0.01])  # Process noise covariance
        self.R_gps = np.diag([1.0, 1.0])  # GPS measurement noise covariance
        self.R_imu = np.diag([0.01, 0.01])  # IMU measurement noise covariance
        
        # Control parameters
        self.Qctrl = np.diag([100, 100, 200, 0.00001, 0.00001, 0.1]) # Original
        self.Rctrl = np.diag([0.1, 0.1, 100, 100])
        self.Rctrl_array = np.array([0.1,
                                     0.1,
                                     100,
                                     100])
        #self.Qctrl = np.diag([100, 100, 200, 0.00001, 0.00001, 0.001])
        #self.Rctrl = 2 # Original
        self.thrust_lower_bound = -100
        self.thrust_upper_bound =  100

        self.U     = np.zeros((self.nu, self.Nu))
        self.Xref  = np.zeros((self.nx, self.Np+1))
        self.waypoints = np.array([[-400],[700],[np.pi/2]])

        # Sydney Regatta Centre coordinates (approximate center)
        self.datum_lat = -33.7285
        self.datum_lon = 150.6789

        # Initialize projections
        self.proj_wgs84 = Proj(proj='latlong', datum='WGS84')
        self.proj_utm = Proj(proj='utm', zone=56, datum='WGS84', south=True)
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
        self.cmd_L_pub = self.create_publisher(Float64, '/wamv/thrusters/stern_port/thrust', 10)
        self.cmd_R_pub = self.create_publisher(Float64, '/wamv/thrusters/stern_star/thrust', 10)
        self.cmd_bl_pub = self.create_publisher(Float64, '/wamv/thrusters/bow_port/thrust', 10)
        self.cmd_br_pub = self.create_publisher(Float64, '/wamv/thrusters/bow_star/thrust', 10)
        self.gps_sub = self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self.imu_callback, 10)
        # self.ref_sub = self.create_subscription(Float64MultiArray, '/wamv/reference', self.reference_callback, 10)
        self.ref_sub = self.create_subscription(PoseArray, '/vrx/wayfinding/waypoints', self.reference_callback, 10)
        
        # Timer for control loop
        self.create_timer(self.dt, self.control_loop)

        self.G = self.create_slew_rate_constraints_G(self.Nu, self.nu)
        self.h = self.create_slew_rate_constraints_h(50, self.U, self.Nu, self.nu)
        self.Ju = self.ju_matrix()
        self.e_u = self.e_matrix_section()

    
    def ju_matrix(self):
        Ju = np.zeros((self.Nu * self.nu,self.Nu * self.nu))
        for i in range(self.Nu):
            Ju[i * self.nu:(i + 1) * self.nu,i * self.nu:(i + 1) * self.nu] = self.Rctrl
        return Ju
    
    def e_matrix_section(self):
        e_u = np.zeros((self.Nu * self.nu, 1))
        for i in range(self.Nu):
            e_u[i*self.nu:i*self.nu + 4,0] = self.Rctrl_array
        return e_u
               

    def error_and_Jacobian(self, x, U, Xref):
        dXdU = np.zeros((self.nx , self.nu * self.Nu))
        ex = np.zeros((self.nx, self.Np+1))
        Jx = np.zeros((self.nx * (self.Np+1), self.nu * self.Nu))

        for k in range(self.Np+1):
            if k<self.Nu:
                ucur = U[:,[k]]
            else:
                ucur = U[:,[self.Nu-1]]

            ex[:,[k]] = self.Qctrl @ (x - Xref[:,[k]])
            Jx[k*self.nx:(k+1)*self.nx,:] = self.Qctrl @ dXdU
            x, dXdU = self.state_transition_U(x, ucur, self.dt, dXdU, k)

        #Ju = np.eye(self.Nu * self.nu) # Original
        Ju = self.Ju
        #e  = np.vstack((ex.reshape(-1,1,order='f'), self.Rctrl * U.reshape(-1,1,order='f'))) # Original
        e = np.vstack((ex.reshape(-1,1,order='f'), self.e_u * U.reshape(-1,1,order='f')))
        J  = np.vstack((Jx, Ju))

        return e, J
    
    def error_only(self, x, U, Xref):
        ex = np.zeros((self.nx, self.Np+1))
        for k in range(self.Np+1):
            if k<self.Nu:
                ucur = U[:,[k]]
            else:
                ucur = U[:,[self.Nu-1]]
            
            ex[:,[k]] = self.Qctrl @ (x - Xref[:,[k]])
            x = self.state_transition_xonly(x, ucur, self.dt)
        
        #e  = np.vstack((ex.reshape(-1,1,order='f'), self.Rctrl * U.reshape(-1,1,order='f'))) # Original
        e = np.vstack((ex.reshape(-1,1,order='f'), self.e_u * U.reshape(-1,1,order='f')))
        return e


    def computeTrajectory(self, x):
        #Compute unit direction vector from x to next waypoint
        uv = self.waypoint[0:2,:] - x[0:2,:]
        distance_to_waypoint = np.linalg.norm(uv)
        uv = uv / distance_to_waypoint

        maxSpeed = 2.0 #m/s
        maxDistance = self.Np * self.dt * maxSpeed
        if maxDistance < distance_to_waypoint:
            wpX = x[0,0] + maxDistance* uv[0,0]
            wpY = x[1,0] + maxDistance* uv[1,0]
        else:
            wpX = self.waypoint[0,0]
            wpY = self.waypoint[1,0]

        self.Xref[0,:] = self.waypoint[0,0] # np.linspace(x[0,0], wpX, self.Np+1)
        self.Xref[1,:] = self.waypoint[1,0] # np.linspace(x[1,0], wpY, self.Np+1)
        
        maxAngSpeed = 0.5 #rads/s
        if distance_to_waypoint < 10.0:
            heading = self.waypoint[2,0] #self.headingOld
        else:
            heading = np.arctan2(uv[1,0],uv[0,0])
        self.Xref[2,:] = self.waypoint[2,0] # np.linspace(x[2,0], heading, self.Np+1)
        self.headingOld = heading

        # print(self.Xref[0:3,:])

    def create_slew_rate_constraints_G(self, Nu, nu):
        
        I = np.eye(nu)

        # Initialize G and h
        G = np.zeros((2* Nu * nu, Nu * nu))
        

        for i in range(Nu):
            if i == 0:
                G[2 * i * nu:(2 * i + 1) * nu, i * nu:(i + 1) * nu] = I
                G[(2 * i + 1) * nu:(2 * i + 2) * nu, i * nu:(i + 1) * nu] = -I
            else:
                G[2 * i * nu:(2 * i + 1) * nu, (i-1) * nu:(i) * nu] = -I
                G[(2 * i + 1) * nu:(2 * i + 2) * nu, (i-1) * nu:(i) * nu] = I
                G[2 * i * nu:(2 * i + 1) * nu, (i) * nu:(i+1) * nu] = I
                G[(2 * i + 1) * nu:(2 * i + 2) * nu, (i) * nu:(i+1) * nu] = -I
            
        return G
    
    def create_slew_rate_constraints_h(self, slew_rate_limit, U, Nu, nu):
        beta = slew_rate_limit * np.ones((nu,1))
        h = np.zeros((2*nu*Nu,1))
        for i in range(Nu):
            if i == 0:
                h[i:nu] = beta + U[:,0:1].reshape(-1, 1)  # β + U_k
                h[nu:2*nu] = beta - U[:,0:1].reshape(-1, 1)  # β - U_k
            else:
                h[2*nu*i:(2 * i + 1) * nu] = beta
                h[(2 * i + 1) * nu:(2 * i + 2) * nu] = beta
        return h
    def update_slew_rate_constraints_h(self, slew_rate_limit, U, nu, h):
        beta = slew_rate_limit * np.ones((nu,1))
        
        
        h[:nu] = beta + U[:,0:1].reshape(-1, 1)  # β + U_k
        h[nu:2*nu] = beta - U[:,0:1].reshape(-1, 1)  # β - U_k
            
        return h
    def control_loop(self):
        print(self.current_state)
        if self.waypoints is not None and self.gpsUpdates > 100 and self.imuUpdates > 100:
            # Apply previous control action and compute predicted state at k+1
            self.last_inputs = self.U[:,0:1]
            x = self.state_transition_xonly(self.current_state, self.U, self.dt)

            print(self.last_inputs)
               
            # Publish control commands
            msg = Float64()
            msg.data = float(4 * self.last_inputs[0])
            self.cmd_L_pub.publish(msg)
            msg.data = float(4 * self.last_inputs[1])
            self.cmd_R_pub.publish(msg)
            msg.data = float(4 * self.last_inputs[2])
            self.cmd_bl_pub.publish(msg)
            msg.data = float(4 * self.last_inputs[3])
            self.cmd_br_pub.publish(msg)


            # Start timer
            start_time = time.time()

            # Solve NMPC problem
            U = np.zeros((self.nu, self.Nu))
            U[:,:-1] = self.U[:,1:]
            U[:,-1]  = self.U[:,-1]

            #Check if we are close to waypoint and move to the next
            if np.linalg.norm(self.waypoints[:,[self.currentwaypoint]] - self.current_state[:3,[0]]) < 0.5:
                self.currentwaypoint += 1
                if self.currentwaypoint >= self.waypoints.shape[1]:
                    self.currentwaypoint = self.waypoints.shape[1]-1

            #Pick waypoint from the list
            self.waypoint = self.waypoints[:,[self.currentwaypoint]]

            #Determine trajectory
            self.computeTrajectory(x)

            
            #self.h = self.update_slew_rate_constraints_h(50, self.U, self.nu, self.h)

            # Run main optimisation loop
            for i in range(10):
                #compute error and jacobian
                e, J = self.error_and_Jacobian(x, U, self.Xref)

                # Compute search direction
                # p, res, tmp, sv = np.linalg.lstsq(J,e, rcond=None)
                H = J.T @ J
                f = J.T @ e
                lb = -U.reshape(-1,1,order='f') + self.thrust_lower_bound*np.ones((self.Nu * self.nu,1))
                ub = -U.reshape(-1,1,order='f') + self.thrust_upper_bound*np.ones((self.Nu * self.nu,1))
                
                #Slew rates as G and h s.t. GU <= h
                #prob = Problem(H,f,self.G,self.h,None,None,lb,ub)
                prob = Problem(H,f,None,None,None,None,lb,ub)
                sol  = solve_problem(prob,solver='proxqp')

                p = sol.x
                lam = sol.z_box
                maxv = np.max((np.max(np.fabs(f)),np.max(np.fabs(lam))))

                #Compute the Newton Decrement and return if less than some threshold
                if np.linalg.norm(f+lam.reshape(-1,1,order='f')) < np.max((1e-3, 1e-3*maxv)):
                    break

                #Compute cost (sum of squared errors)
                co = e.T @ e

                # print("Cost = {:f}, FONC = {:f}".format(co[0,0], np.linalg.norm(f+lam.reshape(-1,1,order='f'))))

                #Use backtracking line search
                alp = 1.0
                for j in range(30):
                    #Try out new input sequence 
                    Un = U + alp*p.reshape(self.nu,-1,order='f')
                    e = self.error_only(x, Un, self.Xref)
                    cn = e.T @ e
                    #If we have reduced the cost then accept the new input sequence and return
                    if np.isfinite(cn) and cn < co:
                        U = Un
                        break
                    #Otherwise halve the step length
                    alp = alp / 2.0
                
                if j==29:
                    Jn = 0*J
                    for j in range(self.Nu * self.nu):
                        Ut = U.copy()
                        Ut = Ut.reshape(-1,1,order='f')
                        Ut[j] += 1e-8
                        Ut = Ut.reshape(self.nu,-1,order='f')
                        ep, Jp = self.error_and_Jacobian(x, Ut, self.Xref)
                        Ut = U.copy()
                        Ut = Ut.reshape(-1,1,order='f')
                        Ut[j] -= 1e-8
                        Ut = Ut.reshape(self.nu,-1,order='f')
                        em, Jm = self.error_and_Jacobian(x, Ut, self.Xref)
                        Jn[:,[j]] = (ep-em)/2e-8

                    print('hello')


            #record the optimal input sequence
            self.U = U

            # End timer
            end_time = time.time()

            # Calculate elapsed time
            elapsed_time = end_time - start_time
            print("Elapsed time: ", elapsed_time)
            # print('end')

    def reference_callback(self, msg):
        # Extract yaw from quaternion for the reference pose
        self.waypoints = np.zeros((3,len(msg.poses)))
        for i in range(len(msg.poses)):
            quat = [msg.poses[i].orientation.x, msg.poses[i].orientation.y, msg.poses[i].orientation.z, msg.poses[i].orientation.w]
            euler = Rotation.from_quat(quat).as_euler('xyz')
            x, y = self.gps_to_local_xy(msg.poses[i].position.y, msg.poses[i].position.x)
            self.waypoints[:3,i] = [x, y, euler[2]]  # yaw is euler[2]

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
                          [(self.L_b*(F_bl+F_br) + self.d_m*(F_r-F_l))/self.m33]])

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
        #                   [(self.L_b*(F_bl+F_br) + self.d_m*(F_r-F_l))/self.m33]])

        dFdu = 4 * np.eye(4)

        dxdF = np.array([
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [1/self.m11, 1/self.m11, 0, 0],
            [0, 0, -1/self.m22, 1/self.m22],
            [-self.d_m/self.m33, self.d_m/self.m33, self.L_b/self.m33, self.L_b/self.m33]
        ])

        B = dxdF @ dFdu

        dk1du = self.CTStateModelDx(x) @ dxdu
        if k<self.Nu:
            dk1du[:,k*self.nu:(k+1)*self.nu] += B
        else:
            dk1du[:,(self.Nu-1)*self.nu:(self.Nu)*self.nu] += B
        dk2du = self.CTStateModelDx(x+(dt/2.0)*k1) @ (dxdu+(dt/2.0)*dk1du)
        if k<self.Nu:
            dk2du[:,k*self.nu:(k+1)*self.nu] += B
        else:
            dk2du[:,(self.Nu-1)*self.nu:(self.Nu)*self.nu] += B
        dk3du = self.CTStateModelDx(x+(dt/2.0)*k2) @ (dxdu+(dt/2.0)*dk2du)
        if k<self.Nu:
            dk3du[:,k*self.nu:(k+1)*self.nu] += B
        else:
            dk3du[:,(self.Nu-1)*self.nu:(self.Nu)*self.nu] += B
        dk4du = self.CTStateModelDx(x+(dt/1.0)*k3) @ (dxdu+(dt/1.0)*dk3du)
        if k<self.Nu:
            dk4du[:,k*self.nu:(k+1)*self.nu] += B
        else:
            dk4du[:,(self.Nu-1)*self.nu:(self.Nu)*self.nu] += B
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
        self.current_state, F = self.state_transition_A(self.current_state, dt, 0*self.last_inputs)
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

def main(args=None):
    rclpy.init(args=args)
    controller = WAMV_NMPC_Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


