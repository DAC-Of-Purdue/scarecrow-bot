# Turtle Bot 4

The turtle bot 4 we have is a lite version.
The main different between standard and lite version is mainly the camera.
Check the full spec and comparison [here](https://turtlebot.github.io/turtlebot4-user-manual/overview/features.html).

After power-on, it will take a few second to boot up.
You will hear the chime once booting up sequence is completed.
The robot will connect to `DigitalAGClub` WiFi and obtain IP Address: `192.168.0.111`.
You can SSH to it by `ssh ubuntu@192.168.0.111` (your PC also needs to connect to same WiFi).
The login password is `turtlebot4`.
You can also use the web interface by browsing to `192.168.0.111:8000`.

LED status light description is [here](https://iroboteducation.github.io/create3_docs/hw/face/).
And for more detail on setting up is [here.](https://turtlebot.github.io/turtlebot4-user-manual/setup/basic.html)

## ROS

| Environment       | Duo-boot | Virtualization | Docker | Subsystem |
| ----------------- | -------- | -------------- | ------ | --------- |
| Windows x86       |          |                |        |           |
| Windows on Arm    |          |                |        |           |
| osX Intel         |          |                |        |           |
| osX Apple Silicon |          |                |        |           |
| Chromebook        |          |                |        |           |

## Controlling the robot

The robot is operating ROS2 Humble.
Therefore, your desktop must have ROS2 Humble installed.
Then, open the terminal on your PC and run [setup.bash](../setup.bash).

```bash
source setup.bash
```

### Actions

There are several [actions](https://turtlebot.github.io/turtlebot4-user-manual/software/create3.html) available.
To use an actions, it must following this syntax

```bash
ros2 action send_goal [action] [message_type] [value]
```

For example,

```bash
ros2 action send_goal /dock irobot_create_msgs/action/Dock {}
ros2 action send_goal /drive_distance irobot_create_msgs/action/DriveDistance "{distance: 1.0}"
ros2 action send_goal /wall_follow irobot_create_msgs/action/WallFollow "{follow_side: 1, max_runtime: {sec: 100, nanosec: 0}}"
```

You can use following commands to examine actions or message types

```bash
ros2 action list # list all available actions
ros2 action show [action] -t # show action's detail
ros2 interface show [message_type] # show message structure
ros2 interface proto [message_type] # show the message prototype
```

## SLAM

[RViz](https://github.com/ros-visualization/rviz) is a ROS visualization tool.
There are two built-in RViz setups from TurtleBot.

```bash
ros2 launch turtlebot4_viz view_model.launch.py
```

This command will launch RViz with a view from robot or in another word, what the robot sees.
It will show, for example, front-view from the camera or LiDar data.

```bash
ros2 launch turtlebot4_navigation slam.launch.py
# in another terminal
ros2 launch turtlebot4_viz view_robot.launch.py
```

These command will open the top-view 2D coordinate.
The starting point will be the reference point.
Other objects and wall will show in the view according to their coordinates.

### Create a map

In SLAM mode, the NAV2 stack will keep updating map from sensors (odometer & LiDar).
You can manually drive the robot around to area until the map shown in RViz is completed.
Then run the following command to save the map.

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name:
    data: 'map_name'"
```

You will get two files (.pgm and .yaml) at the location where `ros2 launch turtlebot4_navigation slam.launch.py` is running.

## Navigation

With pre-defined map, we can use built-in navigation stack from NAV2.

```bash
# first run NAV2 navigation stack
ros2 launch turtlebot4_navigation nav2.launch.py
# run localization stack with our pre-defined map
ros2 launch turtlebot4_navigation localization.launch.py map:=map_name.yaml
# use RViz to visualize
ros2 launch turtlebot4_viz view_robot.launch.py
```

After the map is loaded into RViz, use `2D Pose Estimate` (top toolbar) to indicate the robot's current position.
Note that you don't have to be precise.
But you should try to give the best estimate of both position and direction.
The robot will calibrate its location with information from the predefined map.
At this point, you can use `Nav2 goal` to automatically navigate the robot into destination.

## Camera

To launch USB camera node with the [parameters](camera_params.yaml), run the command below.

```bash
ros2 run usb_cam usb_cam_node_exe --ros-args --params-file camera_params.yaml
```

It is a good practice to calibrate camera following this [tutorial](https://docs.ros.org/en/rolling/p/camera_calibration/tutorial_mono.html).
