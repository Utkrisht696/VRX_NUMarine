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
	
	## TKINTER INTERFACE
	self.tkroot = = tk.Tk()
	
	self.tkroot.title("Course and Waypoint Management")
	self.tkroot.geometry("800x200")  # Adjusted the size to fit everything

	# Create a frame for the dropdowns and buttons side by side
	self.main_frame = ttk.Frame(self.tkroot)
	main_frame.pack(side=tk.TOP, padx=20, pady=20)

	# Create a sub-frame for the dropdowns and action buttons on the left
	dropdown_frame = ttk.Frame(self.main_frame)
	dropdown_frame.grid(row=0, column=0, padx=20, sticky="nw")

	# Create a sub-frame for the buttons and label on the right
	button_frame = ttk.Frame(self.main_frame)
	button_frame.grid(row=0, column=1, padx=20, sticky="ne")

	# Course dropdown
	course_label = ttk.Label(dropdown_frame, text="Course:")
	course_label.grid(row=0, column=0, padx=10, pady=5)

	course_options = ["Testing", "C1", "C2", "C3"]
	course_var = tk.StringVar()
	course_dropdown = ttk.Combobox(dropdown_frame, textvariable=course_var, values=course_options)
	course_dropdown.grid(row=0, column=1, padx=10, pady=5)
	course_dropdown.current(0)

	# Task dropdown
	task_label = ttk.Label(dropdown_frame, text="Task:")
	task_label.grid(row=1, column=0, padx=10, pady=5)

	task_options = ["Testing", "FTP", "Wildlife"]
	task_var = tk.StringVar()
	task_dropdown = ttk.Combobox(dropdown_frame, textvariable=task_var, values=task_options)
	task_dropdown.grid(row=1, column=1, padx=10, pady=5)
	task_dropdown.current(0)

	# Waypoint list dropdown
	waypoint_label_dropdown = ttk.Label(dropdown_frame, text="Waypoint List:")
	waypoint_label_dropdown.grid(row=2, column=0, padx=10, pady=5)

	waypoint_var = tk.StringVar()
	waypoint_dropdown = ttk.Combobox(dropdown_frame, textvariable=waypoint_var)
	waypoint_dropdown.grid(row=2, column=1, padx=10, pady=5)

	# Bind the selection event to update the selected waypoint file
	waypoint_dropdown.bind("<<ComboboxSelected>>", on_waypoint_selection)

	# Action buttons (Run and Refresh) underneath the dropdowns
	run_button = ttk.Button(dropdown_frame, text="Run", command=run_action)
	run_button.grid(row=3, column=0, padx=10, pady=10)

	refresh_button = ttk.Button(dropdown_frame, text="Refresh", command=refresh_action)
	refresh_button.grid(row=3, column=1, padx=10, pady=10)

	# New list button (aligned to the right)
	new_list_button = ttk.Button(button_frame, text="New List", command=new_list_button_handler)
	new_list_button.grid(row=0, column=0, padx=10, pady=5)

	# New waypoint button (aligned to the right)
	new_waypoint_button = ttk.Button(button_frame, text="New Waypoint", command=new_waypoint_button_handler)
	new_waypoint_button.grid(row=1, column=0, padx=10, pady=5)

	# Label to display the selected course and task when "New List" is pressed
	update_label = ttk.Label(button_frame, text="No List Created Yet.", width=30, anchor="w")
	update_label.grid(row=0, column=1, padx=10, pady=5)

	# Label to display the current waypoint number when "New Waypoint" is pressed
	waypoint_label = ttk.Label(button_frame, text="No List Created Yet.", width=30, anchor="w")
	waypoint_label.grid(row=1, column=1, padx=10, pady=5)

	# Populate the waypoint list dropdown on startup
	refresh_waypoint_list()

	# Start the GUI event loop
	self.tkroot.mainloop()

	## END TKINTER INTERFACE


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
    
    def new_list_button_handler(self):
	    global waypoint_counter, new_yaml_waypoint_list_filename
	    # Reset the waypoint counter when a new list is created
	    waypoint_counter = 0
	    
	    # Get the selected course and task
	    selected_course = course_var.get()
	    selected_task = task_var.get()
		
	    new_waypoint_button.config(state="normal")  # Enable butto
	    
	    yaml_date_tyme_string = format_date_time()
	    
	    # Create the YAML filename
	    new_yaml_waypoint_list_filename = '{}_{}_{}.yaml'.format(selected_course.lower(), selected_task.lower(),yaml_date_tyme_string)
	    
	    # Update the label with fixed-width text to avoid shifting
	    update_label.config(text=f"Creating list: {new_yaml_waypoint_list_filename}", anchor="w")
	    
	    # Create the YAML file structure if it doesn't exist
	    yaml_filepath = os.path.join(yaml_folder_structure, new_yaml_waypoint_list_filename)
	    if not os.path.exists(yaml_filepath):
		initial_yaml_data = {"poses": []}
		with open(yaml_filepath, 'w') as yaml_file:
		    yaml.dump(initial_yaml_data, yaml_file, default_flow_style=False)
	    
	    # Reset the waypoint label to "Starting Waypoint"
	    waypoint_label.config(text="Create Starting Waypoint", anchor="w")
	    
	    # Refresh the waypoint list dropdown
	    refresh_waypoint_list()

	def refresh_waypoint_list(self):
		print("Refreshing waypoint list")
		"""Populate the waypoint dropdown with existing YAML files in the folder."""
		self.yaml_files = [f for f in os.listdir(yaml_folder_structure) if f.endswith('.yaml')]
		waypoint_dropdown['values'] = yaml_files  # Update the dropdown values
		if yaml_files:
		    waypoint_var.set(yaml_files[0])  # Set default selection to the first file

	def new_waypoint_button_handler(self):
		global waypoint_counter, new_yaml_waypoint_list_filename
		waypoint_counter += 1
		
		# Append waypoint data to the YAML file
		yaml_filepath = os.path.join(yaml_folder_structure, new_yaml_waypoint_list_filename)
		append_waypoint_yaml(yaml_filepath)

		# Update the label to reflect the current waypoint with fixed width
		if waypoint_counter == 0:
		    waypoint_label.config(text="Starting Waypoint", anchor="w")
		else:
		    waypoint_label.config(text=f"{waypoint_counter}th Waypoint", anchor="w")

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
        gps_gui_logger.update()  # Update the tkinter interface

    rclpy.shutdown()


if __name__ == '__main__':
    main()
