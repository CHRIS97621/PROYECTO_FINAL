cmake_minimum_required(VERSION 3.8)
project(agro_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)  
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(agro_interfaces REQUIRED)


add_executable(agro_control src/agro_control.cpp)
ament_target_dependencies(agro_control rclcpp std_msgs)

install(TARGETS
agro_control
  DESTINATION lib/${PROJECT_NAME})

# Python package installation
ament_python_install_package(${PROJECT_NAME})


# Install Python scripts
install(PROGRAMS
  scripts/agro_control.py
  DESTINATION lib/${PROJECT_NAME}
)

# Mark the package for ament
ament_package()
