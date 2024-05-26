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
```
