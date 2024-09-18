import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Pose, PoseArray
import yaml
import os
import sys
import tkinter as tk
from tkinter import messagebox
from gps_waypoint_logger.utils.gps_utils import euler_from_quaternion, quaternion_from_euler
from ament_index_python.packages import get_package_share_directory


class GpsGuiLogger(tk.Tk, Node):
    """
    ROS2 node to log GPS waypoints to a file in the format of PoseArray
    """

    def __init__(self, logging_file_path):
        tk.Tk.__init__(self)
        Node.__init__(self, 'gps_waypoint_logger')
        self.title("GPS Waypoint Logger")

        self.logging_file_path = logging_file_path

        self.gps_pose_label = tk.Label(self, text="Current Coordinates:")
        self.gps_pose_label.pack()
        self.gps_pose_textbox = tk.Label(self, text="", width=45)
        self.gps_pose_textbox.pack()

        self.log_gps_wp_button = tk.Button(self, text="Log GPS Waypoint",
                                           command=self.log_waypoint)
        self.log_gps_wp_button.pack()

        # Subscribe to the specific IMU and GPS topics
        self.gps_subscription = self.create_subscription(
            NavSatFix,
            '/wamv/sensors/gps/gps/fix',
            self.gps_callback,
            1
        )
        self.last_gps_position = NavSatFix()

        self.imu_subscription = self.create_subscription(
            Imu,
            '/wamv/sensors/imu/imu/data',
            self.imu_callback,
            1
        )
        self.last_heading = 0.0

        # Clear the YAML file at the start
        self.clear_yaml_file()

    def clear_yaml_file(self):
        """
        Clear the contents of the YAML file by resetting it to an empty poses list.
        """
        initial_data = {
            "poses": []
        }
        with open(self.logging_file_path, 'w') as yaml_file:
            yaml.dump(initial_data, yaml_file, default_flow_style=False)

    def gps_callback(self, msg: NavSatFix):
        """
        Callback to store the last GPS pose
        """
        self.last_gps_position = msg
        self.updateTextBox()

    def imu_callback(self, msg: Imu):
        """
        Callback to store the last heading
        """
        _, _, self.last_heading = euler_from_quaternion(msg.orientation)
        self.updateTextBox()

    def updateTextBox(self):
        """
        Function to update the GUI with the last coordinates
        """
        self.gps_pose_textbox.config(
            text=f"Lat: {self.last_gps_position.latitude:.6f}, Lon: {self.last_gps_position.longitude:.6f}, yaw: {self.last_heading:.2f} rad")

    def log_waypoint(self):
        """
        Function to save a new waypoint to a file in PoseArray format
        """
        # Convert GPS coordinates to a Pose (x = latitude, y = longitude, z = 0)
        new_pose = {
            "position": {
                "x": self.last_gps_position.latitude,
                "y": self.last_gps_position.longitude,
                "z": 0.0
            },
            "orientation": {}
        }

        # Convert yaw to quaternion and set orientation
        quaternion = quaternion_from_euler(0.0, 0.0, self.last_heading)
        new_pose["orientation"]["x"] = quaternion.x
        new_pose["orientation"]["y"] = quaternion.y
        new_pose["orientation"]["z"] = quaternion.z
        new_pose["orientation"]["w"] = quaternion.w

        # Read the current contents of the YAML file
        with open(self.logging_file_path, 'r') as yaml_file:
            existing_data = yaml.safe_load(yaml_file)

        # Add the new pose to the list of poses
        existing_data["poses"].append(new_pose)

        # Write the updated contents back to the YAML file
        with open(self.logging_file_path, 'w') as yaml_file:
            yaml.dump(existing_data, yaml_file, default_flow_style=False)

        messagebox.showinfo("Info", "Waypoint logged successfully")


def main(args=None):
    rclpy.init(args=args)

    # Allow passing the logging path as an argument
    default_yaml_file_path = get_package_share_directory("gps_waypoint_logger")
    if len(sys.argv) > 1:
        yaml_file_path = sys.argv[1]
    else:
        yaml_file_path = default_yaml_file_path

    gps_gui_logger = GpsGuiLogger(yaml_file_path)

    while rclpy.ok():
        # Spin both the ROS system and the interface
        rclpy.spin_once(gps_gui_logger, timeout_sec=0.1)  # Run ros2 callbacks
        gps_gui_logger.update()  # Update the tkinter interface

    rclpy.shutdown()


if __name__ == '__main__':
    main()
