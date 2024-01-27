cmake_minimum_required(VERSION 3.0.2)
project(jeep_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  ackermann_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########
include_directories(${catkin_INCLUDE_DIRS})

#############
## Install ##
#############
catkin_install_python(PROGRAMS
  src/driver.py
  src/teleop_keyboard.py
  src/test.py
  src/mrp_sensor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

