import sys
import os
import time
from can_msgs.msg import Frame

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
from scipy.linalg import block_diag
import pymap3d as pm
class WAMV_NMPC_Controller(Node):

    def __init__(self):
        super().__init__('wamv_nmpc_controller')
        
        # Model parameters 252.5551, 251.7846, 516.2194,  98.9777, 100.4924, 806.6199, 150.1745, 99.7235, 809.7870
        self.m11, self.m22, self.m33 = 395, 395, 1101 #410.89900114, 525.73311676, 544.59887753#250.0, 250.0, 500.0
        self.d11, self.d22, self.d33 = 47, 206, 548 #115.90932599,  76.23187308, 812.11345731#100.0, 100.0, 800.0
        self.d11_2, self.d22_2, self.d33_2 = 166, 50, 583 #142.36120428, 206.60423149, 902.23070692#150.0, 100.0, 800.0
        self.d_m, self.L_m = 0.63, 0.98
        self.d_b, self.L_b = 0.63, 1.62
        
        # NMPC parameters
        self.Np = 20  # Prediction horizon
        self.Nu = 10      
        self.dt = 0.2  # Time step
        self.nu = 4
        self.nx = 6
        
        # Initialize state and reference
        self.current_state = np.zeros((self.nx,1))
       
        # Last known actuator inputs
        self.last_inputs = np.zeros((self.nu,1))

        # EKF parameters
        self.P = np.diag([1000.0, 1000.0, 10.0, 0.0, 0.0, 0.0]) # Initial state covariance
        self.Q = 1e-3*np.eye(6)#np.diag([0.01, 0.01, 0.001, 0.001, 0.001, 0.001])  # Process noise covariance
        # self.Q = np.array([
        #     [ 6.1856e-02, -9.4391e-04,  1.4843e-04,  4.3312e-04, -1.2616e-04, -1.0697e-04],
        #     [-9.4391e-04,  6.5208e-02, -2.3349e-04,  3.8015e-04, -4.1236e-04,  3.9740e-04],
        #     [ 1.4843e-04, -2.3349e-04,  2.0787e-04,  5.3975e-06,  8.4772e-06,  6.3032e-05],
        #     [ 4.3312e-04,  3.8015e-04,  5.3975e-06,  7.9621e-06, -3.0529e-06,  3.4317e-06],
        #     [-1.2616e-04, -4.1236e-04,  8.4772e-06, -3.0529e-06,  5.8290e-06, -1.3472e-07],
        #     [-1.0697e-04,  3.9740e-04,  6.3032e-05,  3.4317e-06, -1.3472e-07,  2.4441e-05]
        #     ])
        self.R_gps = np.diag([0.1, 0.1])  # GPS measurement noise covariance
        self.R_imu = np.diag([0.1, 0.1])  # IMU measurement noise covariance

        self.beta = 20
        self.G = np.vstack((np.eye(self.nu),-np.eye(self.nu)))
        self.h = self.beta * np.ones((2*self.nu*self.Nu,1))
        I = np.eye(self.nu)
        Z = np.zeros(self.nu)
        mat = np.block([[-I, I],[I, -I]])
        for i in range(self.Nu-1):
            self.G = np.block([[self.G,np.zeros((self.G.shape[0],self.nu))],[np.zeros((2*self.nu,self.G.shape[1]-self.nu)),mat]])
        
        #Control parameters
        self.Qctrl = np.diag([100, 100, 200, 0.00001, 0.00001, 0.1])
        self.Rctrl = 4
        self.Rdiff = 10
        self.thrust_lower_bound = -80
        self.thrust_upper_bound =  80

        self.U     = np.zeros((self.nu, self.Nu))
        self.Xref  = np.zeros((self.nx, self.Np+1))
        self.waypoints = np.array([[20.0],[-20.0],[0.0]])

        # Sydney Regatta Centre coordinates (approximate center)
        self.datum_lat = -32.91649273201699
        self.datum_lon = 151.76075877703477

        # # Initialize projections
        # self.proj_wgs84 = Proj(proj='latlong', datum='WGS84')
        # self.proj_utm = Proj(proj='utm', zone=56, datum='WGS84', south=True)
        # self.transformer = Transformer.from_proj(self.proj_wgs84, self.proj_utm)

        # Calculate datum in UTM coordinates
        # self.datum_x, self.datum_y = self.transformer.transform(self.datum_lon, self.datum_lat)
        
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
        self.cmd_L_pub = self.create_publisher(Float64, '/wamv/thrusters/stern_port/thrust', 10)
        self.cmd_R_pub = self.create_publisher(Float64, '/wamv/thrusters/stern_star/thrust', 10)
        self.cmd_bl_pub = self.create_publisher(Float64, '/wamv/thrusters/bow_port/thrust', 10)
        self.cmd_br_pub = self.create_publisher(Float64, '/wamv/thrusters/bow_star/thrust', 10)
        self.gps_sub = self.create_subscription(NavSatFix, '/NavSatFix', self.gps_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/Imu', self.imu_callback, 10)
        # self.ref_sub = self.create_subscription(Float64MultiArray, '/wamv/reference', self.reference_callback, 10)
        self.ref_sub = self.create_subscription(PoseArray, '/vrx/wayfinding/waypoints', self.reference_callback, 10)
        self.waypoint_pub = self.create_publisher(Float64MultiArray, '/wamv/computed_trajectory', 10)

        self.can_sub = self.create_subscription(Frame, 'can_tx', self.can_callback, 10)  # Subscribe to CAN messages
        
        # Timer for control loop
        self.create_timer(self.dt, self.control_loop)
    
    def error_and_Jacobian(self, x, U, Xref, grad=False):
        dXdU = np.zeros((self.nx , self.nu * self.Nu))
        ex = np.zeros((self.nx, self.Np+1))

        if grad:
            Jx = np.zeros((self.nx * (self.Np+1), self.nu * self.Nu))

        Uo = U.copy()
        Uo[:,1:] = Uo[:,:-1]
        Uo[:,[0]] = self.last_inputs

        for k in range(self.Np+1):
            if k<self.Nu:
                ucur = U[:,[k]]
            else:
                ucur = 0*U[:,[self.Nu-1]]

            ex[:,[k]] = self.Qctrl @ (x - Xref[:,[k]])
            if grad:
                Jx[k*self.nx:(k+1)*self.nx,:] = self.Qctrl @ dXdU
                x, dXdU = self.state_transition_U(x, ucur, self.dt, dXdU, k)
            else:
                x = self.state_transition_xonly(x, ucur, self.dt)

        e  = np.vstack((ex.reshape(-1,1,order='f'), self.Rctrl * U.reshape(-1,1,order='f'), self.Rdiff * (U-Uo).reshape(-1,1,order='f')))
        if grad:
            Ju = self.Rctrl * np.eye(self.nu * self.Nu)
            Jdu = self.Rdiff * (np.eye(self.nu * self.Nu) - np.diag(np.ones((self.Nu-1)*self.nu),-self.nu))
            J  = np.vstack((Jx, Ju, Jdu))
            return e, J
        else:
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
        self.Xref[2,:] = heading#self.waypoint[2,0] # np.linspace(x[2,0], heading, self.Np+1)
        self.headingOld = heading

        # Publish computed trajectory
        msg = Float64MultiArray()
        msg.data = self.Xref.flatten().tolist()
        self.waypoint_pub.publish(msg)

        # print(self.Xref[0:3,:])


    def publish_current_state(self,x):
        msg = Float64MultiArray()
        msg.data = x.flatten().tolist()  # Convert the state to a flat list
        self.state_pub.publish(msg)

    def can_callback(self, msg):
        """Callback function to handle incoming CAN messages."""
        if msg.id == 0x0:  # Check for the specific CAN ID
            if len(msg.data) >= 5 and any(b > 0 for b in msg.data[:5]):  # Check if any of the first 5 bytes are > 0
                self.enabled = True
                # self.get_logger().info("Control enabled based on CAN message.")
            else:
                self.enabled = False
                # self.get_logger().info("Control disabled based on CAN message.")
    
    def control_loop(self):

        if not self.enabled:
            self.get_logger().info("Control loop skipped because control is disabled.")
            return
        
        # print(self.current_state[0:3,[0]].T)
        if self.waypoints is not None and self.gpsUpdates > 100 and self.imuUpdates > 100:
            # Apply previous control action and compute predicted state at k+1
            self.last_inputs = self.U[:,[0]]
            # Publish control commands
            msg = Float64()
            msg.data = float(self.last_inputs[0])
            self.cmd_L_pub.publish(msg)
            msg.data = float(self.last_inputs[1])
            self.cmd_R_pub.publish(msg)
            msg.data = float(self.last_inputs[2])
            self.cmd_bl_pub.publish(msg)
            msg.data = float(self.last_inputs[3])
            self.cmd_br_pub.publish(msg)

            x = self.state_transition_xonly(self.current_state, self.last_inputs, self.dt)
            #x = self.current_state

            self.publish_current_state(x)
            # print(self.last_inputs.T)


            # Start timer
            start_time = time.time()

            # Solve NMPC problem
            U = np.zeros((self.nu, self.Nu))
            U[:,:-1] = self.U[:,1:]
            U[:,-1]  = self.U[:,-1]

            #Check if we are close to waypoint and move to the next
            if np.linalg.norm(self.waypoints[:,[self.currentwaypoint]] - self.current_state[:3,[0]]) < 2:
                self.currentwaypoint += 1
                if self.currentwaypoint >= self.waypoints.shape[1]:
                    self.currentwaypoint = self.waypoints.shape[1]-1

            #Pick waypoint from the list
            self.waypoint = self.waypoints[:,[self.currentwaypoint]]

            #Determine trajectory
            self.computeTrajectory(x)

            h = self.h.copy()
            h[0:self.nu,[0]] += self.last_inputs
            h[self.nu:2*self.nu,[0]] -= self.last_inputs

            # Run main optimisation loop
            for i in range(10):
                #compute error and jacobian
                e, J = self.error_and_Jacobian(x, U, self.Xref, True)

                # Compute search direction
                # p, res, tmp, sv = np.linalg.lstsq(J,e, rcond=None)
                H = J.T @ J
                f = J.T @ e
                lb = -U.reshape(-1,1,order='f') + self.thrust_lower_bound*np.ones((self.Nu * self.nu,1))
                ub = -U.reshape(-1,1,order='f') + self.thrust_upper_bound*np.ones((self.Nu * self.nu,1))
                
                #Slew rates as G and h s.t. GU <= h
                hh = h - self.G @ U.reshape(-1,1,order='f')
                
                prob = Problem(H,f,self.G,hh,None,None,lb,ub)
                sol  = solve_problem(prob,solver='daqp')

                if sol.found == False:
                    print('QP not solved')

                p = sol.x
                lam = sol.z_box
                lamg = sol.z
                maxv = np.max((np.max(np.fabs(f)),np.max(np.fabs(lam)),np.max(np.fabs(lamg))))
                # maxv = np.max((np.max(np.fabs(f)),np.max(np.fabs(lam))))


                #Compute the Newton Decrement and return if less than some threshold
                fonc_vec = f+lam.reshape(-1,1,order='f')+(self.G.T @ lamg).reshape(-1,1,order='f')
                # fonc_vec = f+lam.reshape(-1,1,order='f')
                if np.linalg.norm(fonc_vec) < np.max((1e-1, 1e-3*maxv)):
                    break

                #Compute cost (sum of squared errors)
                co = e.T @ e

                # print("Cost = {:f}, FONC = {:f}".format(co[0,0], np.linalg.norm(fonc_vec)))

                #Use backtracking line search
                alp = 1.0
                for j in range(30):
                    #Try out new input sequence 
                    Un = U + alp*p.reshape(self.nu,-1,order='f')
                    e = self.error_and_Jacobian(x, Un, self.Xref)
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
                        Ut[j] += 1e-6
                        Ut = Ut.reshape(self.nu,-1,order='f')
                        # ep, Jp = self.error_and_Jacobian(x, Ut, self.Xref)
                        ep = self.error_and_Jacobian(x, Ut, self.Xref)
                        Ut = U.copy()
                        Ut = Ut.reshape(-1,1,order='f')
                        Ut[j] -= 1e-6
                        Ut = Ut.reshape(self.nu,-1,order='f')
                        # em, Jm = self.error_and_Jacobian(x, Ut, self.Xref)
                        em = self.error_and_Jacobian(x, Ut, self.Xref)
                        Jn[:,[j]] = (ep-em)/2e-6

                    print('hello')

            # if any(self.G@U.reshape(-1,1,order='f') > h):
                # print('asdf')

            #record the optimal input sequence
            self.U = U

            # End timer
            end_time = time.time()

            # Calculate elapsed time
            elapsed_time = end_time - start_time
            xc = self.current_state[:,0]
            uc = self.last_inputs[:,0]
            print("X: {:8.2f}, Y: {:8.2f}, P: {:8.2f}, U1: {:8.2f}, U2: {:8.2f}, U3: {:8.2f}, U4: {:8.2f}, ET: {:8.2f}".format(xc[0], xc[1], xc[2], uc[0], uc[1], uc[2], uc[3], elapsed_time))
            # print('end')

    def reference_callback(self, msg):
        # Extract yaw from quaternion for the reference pose
        self.waypoints = np.zeros((3,len(msg.poses)))
        for i in range(len(msg.poses)):
            quat = [msg.poses[i].orientation.x, msg.poses[i].orientation.y, msg.poses[i].orientation.z, msg.poses[i].orientation.w]
            euler = Rotation.from_quat(quat).as_euler('xyz')
            x, y = self.gps_to_local_ned(msg.poses[i].position.y, msg.poses[i].position.x)
            self.waypoints[:3,i] = [x, y, euler[2]]  # yaw is euler[2]

    def CTStateModel(self, x, u):
        # Unpack state and inputs
        N, E, psi, u_vel, v, r = x[:,0]
        F_l  = 2*1.7*u[0,0]
        F_r  = 2*1.7*u[1,0]
        F_bl = 1.7*u[2,0]
        F_br = 1.7*u[3,0]
        
        tau_u = np.array([[0],
                          [0],
                          [0],
                          [(F_l+F_r)/self.m11],
                          [(-F_br+F_bl)/self.m22],
                          [(self.L_b*(F_bl-F_br) + self.d_m*(-F_r+F_l))/self.m33]])

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
        #                   [(-F_br+F_bl)/self.m22],
        #                   [(self.L_b*(F_bl-F_br) + self.d_m*(-F_r+F_l))/self.m33]])
        dFdu = np.diag([2*1.7, 2*1.7, 1.7, 1.7])

        dxdF = np.array([
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [1/self.m11, 1/self.m11, 0, 0],
            [0, 0, 1/self.m22, -1/self.m22],
            [self.d_m/self.m33, -self.d_m/self.m33, self.L_b/self.m33, -self.L_b/self.m33]
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

    def gps_to_local_ned(self, lon, lat):
        # Convert GPS coordinates to UTM
        # x, y = self.transformer.transform(lon, lat)
        
        # Calculate local x-y relative to the datum
        # local_x = x - self.datum_x
        # local_y = y - self.datum_y
        
        north, east, down = pm.geodetic2ned(lat, lon, 0, self.datum_lat, self.datum_lon,0)

        return north, east

    def gps_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_update_time
        self.last_update_time = current_time

        # Convert GPS to local x-y coordinates
        north, east = self.gps_to_local_ned(msg.longitude, msg.latitude)
        
        self.update_ekf(dt, gps_measurement=np.array([north, east]))

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

def main(args=None):
    rclpy.init(args=args)
    controller = WAMV_NMPC_Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

