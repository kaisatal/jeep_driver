# Jeep Raptor Driver

ROS 2 Humble Python package for controlling the Jeep Raptor vehicle through Jetson Nano.
Includes manual teleoperation through keyboard and lidar-based autonomous driving using Velodyne VLP-16.

---

## Overview

The system consists of five ROS 2 nodes:

- `jeep_driver_node` — low-level motor and steering control (GPIO + PWM)
- `feedback_node` — steering angle sensor (AS5600 via I2C)
- `keyboard_node` — keyboard control interface (Manual)
- `last_path_recorder_node` — saving traversed path into a file
- `path_follower_node` — pure pursuit controller (Autonomous)
- `drive_logic_node` — selection between Manual and Autonomous input

Launch file for vehicle control (through Jetson Nano):

- `jeep_driver.launch.py` — jeep_driver_node + keyboard_node + drive_logic_node

---

### Additional required ROS packages

On Jetson Nano:
- ackermann_msgs

On external laptop:
- lidar_localization_ros2
- lidarslam_ros2
- ndt_omp_ros2

## Workspace setup

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

### Jetson Nano with ROS 2 Humble

Keyboard control (first terminal):
```bash
$ ros2 run jeep_driver keyboard_node
```

Controls:

w → forward

a → left

s → reverse

d → right

SPACE → toggle Manual vs Autonomous driving

Motor driver (second terminal):
```bash
$ ros2 launch jeep_driver jeep_driver.launch.py
```

### External laptop with ROS 2 Humble

Saving the current path to last_path_bag:
```bash
$ ros2 run jeep_driver last_path_recorder_node
```

Following the given path from last_path_bag:
```bash
$ ros2 run jeep_driver path_follower_node
```