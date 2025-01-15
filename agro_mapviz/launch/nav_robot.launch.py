import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    #~~~~~~~~~~~~~~~~~~ PACKAGES ~~~~~~~~~~~~~~~
    package_gazebo = 'agro_gazebo'
    package_path_planner = 'path_planner_server'
    package_mapviz = 'agro_mapviz'

    #~~~~~~~~~~~~~~~~~~~~~~~~ PATHS ~~~~~~~~~~~~~~~~~~~~~~~~~~~+
    sim_launch_path = PathJoinSubstitution([FindPackageShare(package_gazebo), 'launch', 'sim.launch.py'])
    navigation_launch_path = PathJoinSubstitution([FindPackageShare(package_path_planner), 'launch', 'navigation.launch.py'])
    mapviz_launch_path = PathJoinSubstitution([FindPackageShare(package_mapviz), 'launch', 'mapviz.launch.py'])

    #~~~~~~~~~~~~~~~~~~ INCLUDE LAUNCH FILES ~~~~~~~~~~~~~~~
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch_path)
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_path)
    )

    mapviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mapviz_launch_path)
    )

    return LaunchDescription([
        sim_launch,
        #navigation_launch,
        mapviz_launch,
    ])
