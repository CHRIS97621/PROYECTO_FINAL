cmake_minimum_required(VERSION 3.8)
project(agro_sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
#find_package(tf2_ros_py REQUIRED)
find_package(turtlesim REQUIRED)

#-------------#
#  C++ NODES  #
#-------------#

#----------------#
#  Python NODES  #
#----------------#
# Install module
ament_python_install_package(${PROJECT_NAME})
# Install py executables
install(PROGRAMS
  
  scripts/imu_node.py
  scripts/gps_node.py


  
  DESTINATION lib/${PROJECT_NAME}
)

#-------------------------#
#  INSTALL LAUNCH FILES  #
#-------------------------#
install(DIRECTORY
launch 
DESTINATION share/${PROJECT_NAME}/
)

ament_package()