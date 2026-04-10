# Jeep Raptor Driver

This repository contains the Jeep Raptor driver.

### Setup:
* Jetson Nano + ROS 2 Humble
* Ubuntu + ROS 2 Humble

### Nodes:
* _**feedback_node**_ (mrp_sensor.py) - reads values from the MRP sensor attached to the wheel of the Jeep and publishes to the _/feedback_ topic. The angle values can range from -20 to 75.
* _**teleop_node**_ (teleop_keyboard.py) - publishes Ackermann Drive messages to the _/drive_ topic with target steering angle (from 0 to 70) and speed (-1/0/1).
* _**jeep_driver_node**_ (driver.py) - subscrives to _/feedback_ and _/drive_ topic and using the PID controller adjusts the steering angle of the Jeep and sets the movement speed (-1 - backward, 0 - stop, 1 - forward).
* ...

### Set up the workspace and test:

```bash
$ TODO
```
In Terminal 1:
```bash
$ TODO
```

In Terminal 2:
```bash
$ TODO
```
