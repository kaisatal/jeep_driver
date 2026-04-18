# Jeep Raptor Driver

ROS 2 Humble Python package for controlling the Jeep Raptor vehicle through Jetson Nano.
Includes manual teleoperation through keyboard and lidar-based autonomous driving using Velodyne VLP-16.

---

## Overview

The system consists of five ROS 2 nodes:

- `jeep_driver_node` — low-level motor and steering control (GPIO + PWM)
- `feedback_node` — steering angle sensor (AS5600 via I2C)
- `path_follower_node` — pure pursuit controller
- `drive_logic_node` — selects between manual and autonomous input
- `keyboard_node` — keyboard control interface

There are 2 launch files for launching multiple nodes at once:

- `jeep_driver.launch.py` — jeep_driver_node + keyboard_node (must be on Jetson Nano)
- `drive_logic.launch.py` — path_follower_node + drive_logic_node (recommended to put on a separate machine)

---

## Workspace setup:

```bash
$ cd ~/ros2_ws/src
$ git clone <this package>
```

Build:
```bash
$ source /opt/ros/humble/setup.bash
$ cd ~/ros2_ws
$ colcon build
$ source install/setup.bash
```

### Jetson Nano with ROS 2 Humble:
Low-level driver + sensor nodes:
```bash
$ ros2 launch jeep_driver jeep_driver.launch.py
```

Keyboard control node:
```bash
$ ros2 run jeep_driver keyboard_node
```
Controls:

w → forward

s → reverse

a → left

d → right

SPACE → toggle Manual vs Autonomous driving


### Another device with ROS 2 Humble:
Drive logic nodes:
```bash
$ ros2 launch jeep_driver drive_logic.launch.py
```