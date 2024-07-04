import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import NavSatFix, Imu
#from some_package.srv import Trajectory  # Replace with actual service types
from scipy.optimize import least_squares
import math

# Kinematics helper function for skew-symmetric matrix
def skew(vector):
    return np.array([
        [0, -vector[2], vector[1]],
        [vector[2], 0, -vector[0]],
        [-vector[1], vector[0], 0]
    ])

# Initialize constants
l = -1.9  # Longitudinal distance from B to wheels [m]
d = 1.63  # Distance between wheels [m]
thruster_depth = -0.05

rLBb = np.array([l, -d/2, thruster_depth])
rRBb = np.array([l, d/2, thruster_depth])
rFLBb = np.array([-l, -d/2, thruster_depth])
rFRBb = np.array([-l, d/2, thruster_depth])

SrLBb = skew(rLBb)
SrRBb = skew(rRBb)
SrFLBb = skew(rFLBb)
SrFRBb = skew(rFRBb)

Ba_top = np.array([
    [1, 1, 0, 0],
    [0, 0, -1, 1],
    [0, 0, 0, 0]
])

Ba_bottom = np.hstack([
    SrLBb @ np.array([[1, 0, 0]]).T,
    SrRBb @ np.array([[1, 0, 0]]).T,
    SrFRBb @ np.array([[1, 0, 0]]).T,
    SrFLBb @ np.array([[1, 0, 0]]).T
])

Ba = np.vstack([Ba_top, Ba_bottom, np.zeros((6, 4))])

def dynamics(t, x, u):
    q3_bar = 0
    q4_bar = 0
    q5_bar = 0

    f = np.array([
        - (8*x[0])/25 - np.sin(x[10])*(24525*q3_bar - 24525*x[8]) - x[1]*(x[1]/1975 - (2*x[5])/395) - x[2]*(x[2]/4475 + (2*x[4])/895),
        (12*x[5])/79 - (96*x[1])/79 + (x[2]*x[3])/400 + x[0]*(x[1]/1975 - (2*x[5])/395) + np.cos(x[10])*np.sin(x[9])*(24525*q3_bar - 24525*x[8]),
        x[0]*(x[2]/4475 + (2*x[4])/895) - (40*x[4])/179 - (x[1]*x[3])/400 - (720*x[2])/179 + np.cos(x[9])*np.cos(x[10])*(24525*q3_bar - 24525*x[8]),
        450*q4_bar - (5*x[3])/2 - 450*x[9] - x[4]*(x[1]/1975 - (2*x[5])/395) - x[2]*((8*x[1])/1975 - x[5]/1975) - x[5]*(x[2]/4475 + (2*x[4])/895) + x[1]*((18*x[2])/4475 + x[4]/4475),
        np.cos(x[9])*(400*q5_bar - 400*x[10]) - (800*x[4])/179 - (80*x[2])/179 + (x[0]*x[2])/250 + (x[3]*x[5])/400 + x[3]*(x[1]/1975 - (2*x[5])/395) - x[0]*((18*x[2])/4475 + x[4]/4475) + np.sin(x[9])*np.tan(x[10])*(450*q4_bar - 450*x[9]),
        (8*x[1])/79 - (80*x[5])/79 - (x[0]*x[1])/250 - (x[3]*x[4])/400 - np.sin(x[9])*(400*q5_bar - 400*x[10]) + x[0]*((8*x[1])/1975 - x[5]/1975) + x[3]*(x[2]/4475 + (2*x[4])/895) + np.cos(x[9])*np.tan(x[10])*(450*q4_bar - 450*x[9]),
        ((18*x[2])/4475 + x[4]/4475)*(np.sin(x[9])*np.sin(x[11]) + np.cos(x[9])*np.cos(x[11])*np.sin(x[10])) - ((8*x[1])/1975 - x[5]/1975)*(np.cos(x[9])*np.sin(x[11]) - np.cos(x[11])*np.sin(x[9])*np.sin(x[10])) + (x[0]*np.cos(x[10])*np.cos(x[11]))/250,
        ((8*x[1])/1975 - x[5]/1975)*(np.cos(x[9])*np.cos(x[11]) + np.sin(x[9])*np.sin(x[10])*np.sin(x[11])) - ((18*x[2])/4475 + x[4]/4475)*(np.cos(x[11])*np.sin(x[9]) - np.cos(x[9])*np.sin(x[10])*np.sin(x[11])) + (x[0]*np.cos(x[10])*np.sin(x[11]))/250,
        np.cos(x[9])*np.cos(x[10])*((18*x[2])/4475 + x[4]/4475) - (x[0]*np.sin(x[10]))/250 + np.cos(x[10])*np.sin(x[9])*((8*x[1])/1975 - x[5]/1975),
        x[3]/400 - np.cos(x[9])*np.tan(x[10])*(x[1]/1975 - (2*x[5])/395) + np.sin(x[9])*np.tan(x[10])*(x[2]/4475 + (2*x[4])/895),
        np.cos(x[9])*(x[2]/4475 + (2*x[4])/895) + np.sin(x[9])*(x[1]/1975 - (2*x[5])/395),
        (np.sin(x[9])*(x[2]/4475 + (2*x[4])/895))/np.cos(x[10]) - (np.cos(x[9])*(x[1]/1975 - (2*x[5])/395))/np.cos(x[10])
    ])

    G = Ba @ u
    return f + G

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/optimized_control', 10)
        self.create_timer(1.0, self.timer_callback)  # 1 Hz

        # Add subscribers for GPS and IMU
        self.subscription_gps = self.create_subscription(
            NavSatFix,
            '/wamv/sensors/gps/gps/fix',
            self.gps_callback,
            10
        )
        self.subscription_imu = self.create_subscription(
            Imu,
            '/wamv/sensors/imu/imu/data',
            self.imu_callback,
            10
        )

        # Add subscribers for thrust inputs
        self.thrust_topics = [
            '/wamv/thrusters/bow_port/thrust',
            '/wamv/thrusters/bow_star/thrust',
            '/wamv/thrusters/stern_port1/thrust',
            '/wamv/thrusters/stern_port2/thrust',
            '/wamv/thrusters/stern_star1/thrust',
            '/wamv/thrusters/stern_star2/thrust'
        ]
        self.thrust_values = [0.0] * len(self.thrust_topics)
        self.combined_thrust_values = [0.0] * 4  # Combined thrust values for bow port, bow star, stern port, stern star
        for i, topic in enumerate(self.thrust_topics):
            self.create_subscription(Float64, topic, lambda msg, i=i: self.thrust_callback(msg, i), 10)

        self.latest_gps = None
        self.latest_imu = None
        self.current_state = None  # Store the current state directly

    def thrust_callback(self, msg, index):
        self.thrust_values[index] = msg.data

        # Combine stern port and stern star thrust values
        self.combined_thrust_values[0] = self.thrust_values[0]  # Bow port thrust
        self.combined_thrust_values[1] = self.thrust_values[1]  # Bow star thrust
        self.combined_thrust_values[2] = self.thrust_values[2] + self.thrust_values[3]  # Combined stern port thrust
        self.combined_thrust_values[3] = self.thrust_values[4] + self.thrust_values[5]  # Combined stern star thrust

        self.update_control_state()

    def gps_callback(self, msg):
        self.latest_gps = msg
        self.update_control_state()

    def imu_callback(self, msg):
        self.latest_imu = msg
        self.update_control_state()

    def update_control_state(self):
        if self.latest_gps is None or self.latest_imu is None:
            return

        # Extract GPS data
        latitude = self.latest_gps.latitude
        longitude = self.latest_gps.longitude
        altitude = self.latest_gps.altitude

        # Extract IMU data
        orientation = self.latest_imu.orientation
        angular_velocity = self.latest_imu.angular_velocity
        linear_acceleration = self.latest_imu.linear_acceleration

        # Convert orientation from quaternion to Euler angles (phi, theta, psi)
        q = orientation
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        phi = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (q.w * q.y - q.z * q.x)
        theta = math.asin(sinp)

        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        psi = math.atan2(siny_cosp, cosy_cosp)

        # Construct displacement vector (N, E, D, phi, theta, psi)
        displacement = np.array([latitude, longitude, altitude, phi, theta, psi])

        # Construct momentum vector from angular velocity and linear acceleration
        momentum = np.array([
            linear_acceleration.x, linear_acceleration.y, linear_acceleration.z,
            angular_velocity.x, angular_velocity.y, angular_velocity.z
        ])

        # Combine into a single state vector
        self.current_state = np.concatenate((momentum, displacement))
        self.get_logger().info(f'State vector updated: {self.current_state}')

    def control_error(self, U):
        dt_ctrl = 5  # Control horizon time (s)
        nSubsteps = 10  # Number of Runge-Kutta integration steps per control horizon
        Np = 10  # Prediction horizon steps
        Nc = 10  # Control horizon steps

        # Penalties
        Q_r = 1e4  # Position error penalty
        Q_psi = 1e2  # Heading error penalty
        R = 1e-3  # Actuator penalty

        sqrtqr = np.sqrt(Q_r)
        sqrtqpsi = np.sqrt(Q_psi)
        sqrtru = np.sqrt(R)

        # Initial values from ROS topics
        t0 = self.get_clock().now().seconds_nanoseconds()[0]  # Current time
        if self.current_state is None:
            return np.zeros((3 + len(U)))  # No control state available yet

        x0 = self.current_state  # Current state
        u0 = self.combined_thrust_values  # Current input from combined thrust values

        dt = dt_ctrl / nSubsteps
        nx = len(x0)
        nu = len(u0)

        x_pred = np.zeros((nx, Np + 1))
        u_pred = np.zeros((nu, Np))

        # Predict forward one time step from (t0, x0) to (t1, x1) with input u0
        t = t0
        x = np.array(x0)

        for _ in range(nSubsteps):
            # Runge-Kutta integration using 3/8-rule
            f1 = dynamics(t, x, u0)
            f2 = dynamics(t + dt/3, x + f1*dt/3, u0)
            f3 = dynamics(t + dt*2/3, x - f1*dt/3 + f2*dt, u0)
            f4 = dynamics(t + dt, x + f1*dt - f2*dt + f3*dt, u0)
            x = x + (f1 + 3*f2 + 3*f3 + f4) * dt / 8
            t = t + dt

        x_pred[:, 0] = x

        for k in range(Np):
            if k < Nc:
                u = U[nu * k:nu * (k + 1)]
            else:
                u = U[nu * (Nc - 1):nu * Nc]

            for _ in range(nSubsteps):
                # Runge-Kutta integration using 3/8-rule
                f1 = dynamics(t, x, u)
                f2 = dynamics(t + dt/3, x + f1*dt/3, u)
                f3 = dynamics(t + dt*2/3, x - f1*dt/3 + f2*dt, u)
                f4 = dynamics(t + dt, x + f1*dt - f2*dt + f3*dt, u)
                x = x + (f1 + 3*f2 + 3*f3 + f4) * dt / 8
                t = t + dt

            u_pred[:, k] = u
            x_pred[:, k + 1] = x

        # Control Error
        e = np.zeros((3 + nu) * Np)

        for k in range(Np):
            t = self.get_clock().now().seconds_nanoseconds()[0] + (k + 1) * dt_ctrl
            x = x_pred[:, k + 1]
            u = u_pred[:, k]

            client = self.create_client(Trajectory, '/trajectory')
            client.wait_for_service()
            request = Trajectory.Request()
            request.time = t

            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                X = future.result().trajectory
            else:
                self.get_logger().error("Service call failed")

            rPNn = np.array([X[0, 0], X[1, 0], 0])
            vPNn = np.array([X[0, 1], X[1, 1], 0])

            if np.linalg.norm(vPNn) < 1e-10:
                psistar = x[12]
                vPNn_norm = 0
            else:
                psistar = np.arctan2(vPNn[1], vPNn[0])
                psistar = psistar + 2 * np.pi * round((x[12] - psistar) / (2 * np.pi))
                vPNn_norm = 1

            rBNn = np.array([x[7], x[8], x[12]])
            rCNn = rBNn  # Boat CoG (C) coinciding with boat reference point (B)

            e[(k * (3 + nu)):(k + 1) * (3 + nu)] = np.concatenate([
                sqrtqr * (rCNn[:2] - rPNn[:2]),
                [sqrtqpsi * (rCNn[2] - psistar)],
                sqrtru * u
            ])

        return e  # Return the error vector for least_squares

    def control_bound_constraints(self):
        uLowerLimit = -np.ones(4)*100  # -100% PWM
        uUpperLimit = np.ones(4)*100  # 100% PWM
        nControlHorizon = 10       # Control horizon steps
        lb = np.tile(uLowerLimit, nControlHorizon)
        ub = np.tile(uUpperLimit, nControlHorizon)
        return lb, ub

    def optimization_solver(self):
        U0 = np.zeros((40,))  # Initial guess for U
        lb, ub = self.control_bound_constraints()

        result = least_squares(self.control_error, U0, bounds=(lb, ub))

        assert result.success, result.message
        return result.x, result.cost, result.status, result.message

    def timer_callback(self):
        U_opt, resnorm, exitflag, message = self.optimization_solver()
        if exitflag > 0:
            self.get_logger().info("Optimization succeeded. Control cost: %f" % resnorm)
            print("Optimal control inputs:", U_opt)

            # Create the message
            msg = Float64MultiArray()
            msg.data = list(map(float, U_opt))  # Ensure the data is a list of floats

            # Publish the message
            self.publisher_.publish(msg)
        else:
            self.get_logger().error("Optimization failed: %s" % message)

def main(args=None):
    rclpy.init(args=args)
    control_node = ControlNode()
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
