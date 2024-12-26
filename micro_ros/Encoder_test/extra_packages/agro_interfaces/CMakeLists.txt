cmake_minimum_required(VERSION 3.8)
project(agro_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
#find_package(geometry_msgs REQUIRED)


find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlStatus.msg"
  "msg/ConfigParams.msg"
  "msg/Motors.msg"
 )



#ament_python_install_package(${PROJECT_NAME})
# Install py executables
#install(PROGRAMS

#  scripts/agro_control.py

#  DESTINATION lib/${PROJECT_NAME}
#)



ament_package()
