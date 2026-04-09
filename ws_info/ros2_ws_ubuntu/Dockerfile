FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

# New workspace + ROS 2 core + source code dependencies
RUN apt-get update && apt-get install -y \
    git \
    cmake \
    build-essential \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep \
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
    libpcl-dev \
    ros-humble-pcl-conversions \
    && rm -rf /var/lib/apt/lists/*


# Initialize rosdep
RUN rosdep init || true && rosdep update

# Create workspace directory
WORKDIR /ros2_ws
RUN mkdir src

# Copy selected repos
COPY localization.repos .

# Fetch only required sources
RUN vcs import src < localization.repos

# Install dependencies
RUN rosdep install --from-paths src --ignore-src -r -y --skip-keys="ros_testing"

# Add map_big.pcd to map_path
RUN sed -i 's|map_path: ""|map_path: "/map/map_big.pcd"|' /ros2_ws/src/lidar_localization_>

# Build workspace
RUN . /opt/ros/humble/setup.sh && colcon build

# Source workspace by default
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc
