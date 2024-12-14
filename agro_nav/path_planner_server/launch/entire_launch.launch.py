from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch_ros.actions import Node, RosTimer
from launch.event_handlers import OnExecutionComplete, OnProcessStart
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
     

    localization = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("map_server"), '/launch', '/localization.launch.py'])
            )
    path_planner_server = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("path_planner_server"), '/launch', '/pathplanner.launch.py'])
            )
    
    servers = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("rb1_shelf_tools"), '/launch', '/servers.launch.py'])
            )
    return LaunchDescription([
        localization,
        RosTimer(period=5.0, actions=[path_planner_server]),
        RosTimer(period=5.0, actions=[servers]),

    ])