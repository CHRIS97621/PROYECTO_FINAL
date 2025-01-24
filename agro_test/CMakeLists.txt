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
  scripts/test_vangular.py
  scripts/test_vangular2.py
  scripts/test_square.py
  scripts/test_square_ekf.py
  scripts/test_square_ekf2.py
  scripts/test_rotate_odom.py
  scripts/test_line_ekf.py
  scripts/navegation_ekf_gps.py
  scripts/navegation_ekf_gps2.py

  #firmware scripts
  scripts/plot_control.py

  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  #world
  #launch
  DESTINATION share/${PROJECT_NAME}
)


#ament_environment_hooks(
 # "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.sh.in")

ament_package()
