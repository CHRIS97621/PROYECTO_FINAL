cmake_minimum_required(VERSION 3.8)
project(agro_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


#----------------#
#  Python NODES  #
#----------------#
# Install module
ament_python_install_package(${PROJECT_NAME})
# Install py executables
install(PROGRAMS
  scripts/plot_odometry.py
  scripts/plot2_odometry.py
  scripts/plot3_odometry.py
  scripts/speed_ekf.py
  scripts/speed_ekf2.py

  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  world
  launch
  DESTINATION share/${PROJECT_NAME}
)


#ament_environment_hooks(
 # "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.sh.in")

ament_package()
