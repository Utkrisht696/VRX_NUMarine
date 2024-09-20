import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Pose, PoseArray
import yaml
import os
import sys
import datetime
from std_msgs.msg import Header
import tkinter as tk
from tkinter import ttk
from gps_waypoint_logger.utils.gps_utils import euler_from_quaternion, quaternion_from_euler
from ament_index_python.packages import get_package_share_directory


class GpsGuiLogger(Node):
	"""
	ROS2 node to log GPS waypoints to a file in the format of PoseArray
	"""

	def __init__(self, logging_file_path):
		Node.__init__(self, 'gps_waypoint_logger')

		self.publisher_ = self.create_publisher(PoseArray, '/vrx/wayfinding/waypoints', 10)
		self.timer = self.create_timer(2.0, self.publish_waypoints)
		self.timer.cancel()
		self.pose_array_msg = None

		self.logging_file_path = logging_file_path
		self.waypoint_counter = 0
		self.new_yaml_waypoint_list_filename = ''
		# print("init started")
		# TKINTER INTERFACE
		self.tkroot = tk.Tk()

		self.tkroot.title("Course and Waypoint Management")
		self.tkroot.geometry("800x200")  # Adjusted the size to fit everything

		# Create a frame for the dropdowns and buttons side by side
		self.main_frame = ttk.Frame(self.tkroot)
		self.main_frame.pack(side=tk.TOP, padx=20, pady=20)

		# Create a sub-frame for the dropdowns and action buttons on the left
		self.dropdown_frame = ttk.Frame(self.main_frame)
		self.dropdown_frame.grid(row=0, column=0, padx=20, sticky="nw")

		# Create a sub-frame for the buttons and label on the right
		self.button_frame = ttk.Frame(self.main_frame)
		self.button_frame.grid(row=0, column=1, padx=20, sticky="ne")

		# Course dropdown
		self.course_label = ttk.Label(self.dropdown_frame, text="Course:")
		self.course_label.grid(row=0, column=0, padx=10, pady=5)

		self.course_options = ["Testing", "C1", "C2", "C3"]
		self.course_var = tk.StringVar()
		self.course_dropdown = ttk.Combobox(self.dropdown_frame, textvariable=self.course_var, values=self.course_options)
		self.course_dropdown.grid(row=0, column=1, padx=10, pady=5)
		self.course_dropdown.current(0)

		# Task dropdown
		self.task_label = ttk.Label(self.dropdown_frame, text="Task:")
		self.task_label.grid(row=1, column=0, padx=10, pady=5)

		self.task_options = ["Testing", "FTP", "Wildlife"]
		self.task_var = tk.StringVar()
		self.task_dropdown = ttk.Combobox(self.dropdown_frame, textvariable=self.task_var, values=self.task_options)
		self.task_dropdown.grid(row=1, column=1, padx=10, pady=5)
		self.task_dropdown.current(0)

		# Waypoint list dropdown
		self.waypoint_label_dropdown = ttk.Label(self.dropdown_frame, text="Waypoint List:")
		self.waypoint_label_dropdown.grid(row=2, column=0, padx=10, pady=5)

		self.waypoint_var = tk.StringVar()
		self.waypoint_dropdown = ttk.Combobox(self.dropdown_frame, textvariable=self.waypoint_var)
		self.waypoint_dropdown.grid(row=2, column=1, padx=10, pady=5)

		# Bind the selection event to update the selected waypoint file
		self.waypoint_dropdown.bind("<<ComboboxSelected>>", self.on_waypoint_selection)

		# Action buttons (Run and Refresh) underneath the dropdowns
		self.run_button = ttk.Button(self.dropdown_frame, text="Run", command=self.run_action)
		self.run_button.grid(row=3, column=0, padx=10, pady=10)

		self.refresh_button = ttk.Button(self.dropdown_frame, text="Refresh", command=self.refresh_action)
		self.refresh_button.grid(row=3, column=1, padx=10, pady=10)

		# New list button (aligned to the right)
		self.new_list_button = ttk.Button(self.button_frame, text="New List", command=self.new_list_button_handler)
		self.new_list_button.grid(row=0, column=0, padx=10, pady=5)

		# New waypoint button (aligned to the right)
		self.new_waypoint_button = ttk.Button(self.button_frame, text="New Waypoint",
											  command=self.new_waypoint_button_handler)
		self.new_waypoint_button.grid(row=1, column=0, padx=10, pady=5)

		# Label to display the selected course and task when "New List" is pressed
		self.update_label = ttk.Label(self.button_frame, text="No List Created Yet.", width=30, anchor="w")
		self.update_label.grid(row=0, column=1, padx=10, pady=5)

		# Label to display the current waypoint number when "New Waypoint" is pressed
		self.waypoint_label = ttk.Label(self.button_frame, text="No List Created Yet.", width=30, anchor="w")
		self.waypoint_label.grid(row=1, column=1, padx=10, pady=5)

		# Populate the waypoint list dropdown on startup
		self.refresh_waypoint_list()



		# END TKINTER INTERFACE

		# Subscribe to the specific IMU and GPS topics
		self.gps_subscription = self.create_subscription(
			NavSatFix,
			'/wamv/sensors/gps/gps/fix',
			self.gps_callback,  ## TKINTER INTERFACE
			1)

		self.last_gps_position = NavSatFix()

		self.imu_subscription = self.create_subscription(
			Imu,
			'/wamv/sensors/imu/imu/data',
			self.imu_callback,
			1
		)
		self.last_heading = 0.0

		# print("gps pos: ", self.last_gps_position)

		# print("Finished init")

		# Start the GUI event loop
		# self.tkroot.mainloop()

	def gps_callback(self, msg: NavSatFix):
		"""
        Callback to store the last GPS pose
        """
		self.last_gps_position = msg

	def imu_callback(self, msg: Imu):
		"""
        Callback to store the last heading
        """
		_, _, self.last_heading = euler_from_quaternion(msg.orientation)

	def new_list_button_handler(self):
		# Reset the waypoint counter when a new list is created
		self.waypoint_counter = 0

		# Get the selected course and task
		self.selected_course = self.course_var.get()
		self.selected_task = self.task_var.get()

		self.new_waypoint_button.config(state="normal")  # Enable butto

		yaml_date_tyme_string = self.format_date_time()

		# Create the YAML filename
		self.new_yaml_waypoint_list_filename = '{}_{}_{}.yaml'.format(self.selected_course.lower(), self.selected_task.lower(),
																 yaml_date_tyme_string)

		# Update the label with fixed-width text to avoid shifting
		self.update_label.config(text=f"Creating list: {self.new_yaml_waypoint_list_filename}", anchor="w")

		# Create the YAML file structure if it doesn't exist
		yaml_filepath = os.path.join(self.logging_file_path, self.new_yaml_waypoint_list_filename)
		if not os.path.exists(yaml_filepath):
			initial_yaml_data = {"poses": []}
			with open(yaml_filepath, 'w') as yaml_file:
				yaml.dump(initial_yaml_data, yaml_file, default_flow_style=False)

		# Reset the waypoint label to "Starting Waypoint"
		self.waypoint_label.config(text="Create Starting Waypoint", anchor="w")

		# Refresh the waypoint list dropdown
		self.refresh_waypoint_list()


	def refresh_waypoint_list(self):
		print("Refreshing waypoint list")
		"""Populate the waypoint dropdown with existing YAML files in the folder."""
		yaml_files = [f for f in os.listdir(self.logging_file_path) if f.endswith('.yaml')]
		self.waypoint_dropdown['values'] = yaml_files  # Update the dropdown values
		if yaml_files:
			self.waypoint_var.set(yaml_files[0])  # Set default selection to the first file


	def new_waypoint_button_handler(self):
		self.waypoint_counter += 1

		# Append waypoint data to the YAML file
		yaml_filepath = os.path.join(self.logging_file_path, self.new_yaml_waypoint_list_filename)
		self.append_waypoint_yaml(yaml_filepath)

		# Update the label to reflect the current waypoint with fixed width
		if self.waypoint_counter == 0:
			self.waypoint_label.config(text="Starting Waypoint", anchor="w")
		else:
			self.waypoint_label.config(text=f"{self.waypoint_counter}th Waypoint", anchor="w")

	def load_waypoints(self, filename):
		"""
        Load the PoseArray from the YAML file.
        """

		waypoints_file = os.path.join(self.logging_file_path, filename)
		print(waypoints_file)
		with open(waypoints_file, 'r') as file:
			pose_array_data = yaml.safe_load(file)

		# Create a PoseArray message and populate it with the loaded poses
		pose_array_msg = PoseArray()
		pose_array_msg.header = Header()
		pose_array_msg.header.frame_id = ''  # Adjust frame_id as needed

		for pose_data in pose_array_data['poses']:
			pose = self.pose_from_yaml(pose_data)
			pose_array_msg.poses.append(pose)

		return pose_array_msg

	def pose_from_yaml(self, pose_data):
		"""
        Convert the pose data from YAML format to a Pose message.
        """

		pose = Pose()
		pose.position.x = pose_data['position']['x']
		pose.position.y = pose_data['position']['y']
		pose.position.z = pose_data['position']['z']
		pose.orientation.x = pose_data['orientation']['x']
		pose.orientation.y = pose_data['orientation']['y']
		pose.orientation.z = pose_data['orientation']['z']
		pose.orientation.w = pose_data['orientation']['w']

		return pose

	def publish_waypoints(self):
		"""
        Publish the PoseArray loaded from the YAML file.
        """
		# Update the timestamp before publishing

		self.pose_array_msg.header.stamp = self.get_clock().now().to_msg()

		# Publish the pose array message
		self.publisher_.publish(self.pose_array_msg)
		self.timer.reset()
		self.get_logger().info('Published pose array with {} waypoints'.format(len(self.pose_array_msg.poses)))

	def format_date_time(self):
		# Get the current date and time
		now = datetime.datetime.now()

		# Format components
		day = now.strftime('%d')  # Numerical day
		day_of_week = now.strftime('%a')  # Abbreviated weekday name (Mon, Tue, etc.)
		hour = now.strftime('%I')  # 12-hour format (01, 02, ..., 12)
		minute = now.strftime('%M')  # Minutes (00, 01, ..., 59)
		period = now.strftime('%p').lower()[0]  # AM or PM in lowercase

		# Remove leading zero from hour if present
		hour = hour.lstrip('0')

		# Combine the components in the desired format
		formatted_time = f"{day}{day_of_week}_{hour}{period}_{minute}m"

		return formatted_time

	def append_waypoint_yaml(self, yaml_filename):
		new_pose = self.generate_current_posearray()

		try:
			with open(yaml_filename, 'r') as yaml_file:
				existing_data = yaml.safe_load(yaml_file)

			# Add the new pose to the list of poses
			existing_data["poses"].append(new_pose)

			# Write the updated contents back to the YAML file
			with open(yaml_filename, 'w') as yaml_file:
				yaml.dump(existing_data, yaml_file, default_flow_style=False)

			print(f"Waypoint {self.waypoint_counter} added to {yaml_filename}")
		except FileNotFoundError:
			print(f"File {yaml_filename} not found!")
		except Exception as e:
			print(f"An error occurred: {e}")

	def on_waypoint_selection(self, event):
		"""Update the filename when a waypoint file is selected."""

		selected_waypoint_file = self.waypoint_var.get()
		self.new_yaml_waypoint_list_filename = os.path.join(self.logging_file_path, selected_waypoint_file)
		self.update_label.config(text=f"Selected list: {self.new_yaml_waypoint_list_filename}", anchor="w")

		# Grey out the add waypoint button when selecting a list
		self.new_waypoint_button.config(state="disabled")  # Disable butto

	def run_action(self):
		print(self.new_yaml_waypoint_list_filename)
		self.pose_array_msg = self.load_waypoints(self.new_yaml_waypoint_list_filename)
		self.publish_waypoints()
		print(f"TODO Run action triggered for: {self.new_yaml_waypoint_list_filename}")

	def refresh_action(self):
		self.refresh_waypoint_list()


	def generate_current_posearray(self):
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

		return new_pose


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
		gps_gui_logger.tkroot.update()  # Update the tkinter interface

	rclpy.shutdown()


if __name__ == '__main__':
	main()
