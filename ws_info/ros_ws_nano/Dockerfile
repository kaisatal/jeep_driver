FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

# ROS 2 core + source code dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-smbus \
    build-essential \
    ros-humble-rclpy \
    ros-humble-ros2cli \
    ros-humble-ros2topic \
    ros-humble-ackermann-msgs \
    ros-humble-velodyne-driver \
    ros-humble-velodyne-pointcloud \
    ros-humble-velodyne-msgs \
    ros-humble-geometry-msgs \
    ros-humble-std-msgs \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --no-cache-dir \
    Jetson.GPIO \
    simple-pid

# Workspace directory
WORKDIR /ros_ws

# Source workspace by default
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /ros_ws/install/setup.bash" >> /root/.bashrc
