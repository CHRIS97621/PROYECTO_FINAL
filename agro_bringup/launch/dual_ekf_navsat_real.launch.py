import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    #~~~~~~~~~~~~~~~~~~ PACKAGES ~~~~~~~~~~~~~~~
    package_name = 'agro_bringup'

    #~~~~~~~~~~~~~~~~~~~~~~~~ PATHS ~~~~~~~~~~~~~~~~~~~~~~~~~~~+
    ekf_share = get_package_share_directory(package_name)
    ekf_yaml =  PathJoinSubstitution([ekf_share,'config', 'dual_ekf_navsat_params_real.yaml'])

    #~~~~~~~~~~~~~~~~~~~~~~~~ NODES ~~~~~~~~~~~~~~~~~~~~~~~~~~~+

    ekf_local = Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node_odom",
            output="screen",
            parameters=[ekf_yaml, {"use_sim_time": False}],
            remappings=[("odometry/filtered", "odometry/local")],
    )

    ekf_global = Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node_map",
            output="screen",
            parameters=[ekf_yaml, {"use_sim_time": False}],
            remappings=[("odometry/filtered", "odometry/global")],
    )

    navsat_transform = Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform",
            output="screen",
            parameters=[ekf_yaml, {"use_sim_time": False}],
            remappings=[
                ("imu/data", "imu"),
                ("gps/fix", "gps/raw"),
                ("odometry/gps", "odometry/gps"),
                ("odometry/filtered", "odometry/global"),
            ],
    )

    return LaunchDescription([
        ekf_local,
        ekf_global,
        navsat_transform,
    ])
