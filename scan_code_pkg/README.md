## Launching ROS Package

Open terminal and run the following:

```shell

. install/setup.bash
ros2 launch vrx_gz competition.launch.py world:=scan_dock_deliver_task
```

In a seperate terminal launch the scan_code_pkg by running:

```shell
. install/setup.bash
ros2 run scan_code_pkg read_gazebo_frame
```

