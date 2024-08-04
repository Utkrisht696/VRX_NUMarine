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
from scipy.linalg import block_diag

class GuidanceNode(Node):

    def __init__(self):
        super().__init__('guidance_node')
        self.Np = 30  # Prediction horizon
        self.dt = 0.1  # Time step
        self.maxSpeed = 2.0  # m/s
        self.maxDistance = self.Np * self.dt * self.maxSpeed
        self.headingOld = 0.0
        self.waypoints = None

        # Publishers and Subscribers
        self.state_sub = self.create_subscription(Float64MultiArray, '/wamv/current_state', self.state_callback, 10)
        self.waypoint_pub = self.create_publisher(Float64MultiArray, '/wamv/computed_trajectory', 10)
        self.ref_sub = self.create_subscription(Float64MultiArray, '/vrx/wayfinding/waypoints', self.reference_callback, 10)

    def state_callback(self, msg):
        current_state = np.array(msg.data).reshape(-1, 1)
        if self.waypoints is not None:
            self.compute_trajectory(current_state)

    def reference_callback(self, msg):
        # Extract yaw from quaternion for the reference pose
        self.waypoints = np.zeros((3,len(msg.poses)))
        for i in range(len(msg.poses)):
            quat = [msg.poses[i].orientation.x, msg.poses[i].orientation.y, msg.poses[i].orientation.z, msg.poses[i].orientation.w]
            euler = Rotation.from_quat(quat).as_euler('xyz')
            x, y = self.gps_to_local_xy(msg.poses[i].position.y, msg.poses[i].position.x)
            self.waypoints[:3,i] = [x, y, euler[2]]  # yaw is euler[2]

    def compute_trajectory(self, x):
        uv = self.waypoints[0:2, :] - x[0:2, :]
        distance_to_waypoint = np.linalg.norm(uv)
        uv = uv / distance_to_waypoint

        if self.maxDistance < distance_to_waypoint:
            wpX = x[0, 0] + self.maxDistance * uv[0, 0]
            wpY = x[1, 0] + self.maxDistance * uv[1, 0]
        else:
            wpX = self.waypoints[0, 0]
            wpY = self.waypoints[1, 0]

        Xref = np.zeros((6, self.Np+1))
        Xref[0, :] = self.waypoints[0, 0]
        Xref[1, :] = self.waypoints[1, 0]
        
        if distance_to_waypoint < 10.0:
            heading = self.waypoints[2, 0]
        else:
            heading = np.arctan2(uv[1, 0], uv[0, 0])
        Xref[2, :] = heading
        self.headingOld = heading

        # Publish computed trajectory
        msg = Float64MultiArray()
        msg.data = Xref.flatten().tolist()
        self.waypoint_pub.publish(msg)




def main(args=None):
    rclpy.init(args=args)
    guidance = GuidanceNode()
    rclpy.spin(guidance)
    guidance.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
