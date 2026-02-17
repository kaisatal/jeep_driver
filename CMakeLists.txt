cmake_minimum_required(VERSION 3.5)
project(jeep_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

# Install Python scripts
install(
  PROGRAMS
    src/driver.py
    src/teleop_keyboard.py
    src/mrp_sensor.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
