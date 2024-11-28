from launch import LaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    #~~~~~~~~~~~~~~~~~~ PACKAGES ~~~~~~~~~~~~~~~
    package_description = 'agro_description'
    package_gz = "ros_gz_sim"
    package_world = "agro_gazebo"

    #~~~~~~~~~~~~~~~~~~~~~~~~ PATHS ~~~~~~~~~~~~~~~~~~~~~~~~~
    path_gz_sim = PathJoinSubstitution([FindPackageShare(package_gz),"launch","gz_sim.launch.py",])


    #~~~~~~~~~~~~~~~~~~~~~~~~ NODES ~~~~~~~~~~~~~~~~~~~~~~~~~~~+
    robot_name = 'robot_agro'
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            robot_name,
            "-allow_renaming",
            "true",
            "-topic",
            "robot_description",
            "-x",
            LaunchConfiguration("x", default="0.00"),
            "-y",
            LaunchConfiguration("y", default="0.00"),
            "-z",
            LaunchConfiguration("z", default="0.30"),
            "-R",
            LaunchConfiguration("roll", default="0.00"),
            "-P",
            LaunchConfiguration("pitch", default="0.00"),
            "-Y",
            LaunchConfiguration("yaw", default="0.00"),
        ],
        output="screen",
    )

    return LaunchDescription([
        gz_spawn_entity,
    ])