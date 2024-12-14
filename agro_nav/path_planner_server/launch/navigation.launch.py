import launch
import launch.event_handlers 
from  launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import RosTimer
from ament_index_python.packages import get_package_share_directory

import os
                             
def generate_launch_description():

    #~~~~~~~~~~~~~~~~~~ PACKAGES ~~~~~~~~~~~~~~~

    package_map = 'map_server'
    package_path_planner_server = 'path_planner_server'

    #~~~~~~~~~~~~~~~~~~~~~~~~ PATHS ~~~~~~~~~~~~~~~~~~~~~~~~~~~+

    launch_map = PathJoinSubstitution([ get_package_share_directory(package_map),'launch','map_server.launch.py'])
    launch_path_planner_server = PathJoinSubstitution([ get_package_share_directory(package_path_planner_server),'launch','pathplanner.launch.py'])


    #~~~~~~~~~~~~~~~~~~ ARGUMENTS ~~~~~~~~~~~~~~~

    arg_map_file = DeclareLaunchArgument(
        'map_file',
        choices= ['map.yaml','warehouse_map_real.yaml','warehouse_map_sim_edit.yaml'],
        default_value='map.yaml',
        description='Path to the map select'
    )

    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        choices= ['True', 'False'],
        default_value= 'True',
        description='Path to the map select'
    )


    arg_type_simulation = DeclareLaunchArgument(
        'type_simulation',
        default_value='sim_robot',
        choices=['sim_robot', 'real_robot'],
        description='Use for simlation robot or real robot'
    )
    config_map_file = LaunchConfiguration('map_file')
    config_use_sim_time = LaunchConfiguration('use_sim_time')
    config_type_simulation = LaunchConfiguration('type_simulation')



    #~~~~~~~~~~~~~~~~~~ LAUNCH ~~~~~~~~~~~~~~~

    map_server = IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_map),
                    launch_arguments=[['use_sim_time', config_use_sim_time],
                                      ['rviz', 'False'],
                                      ['map_file', config_map_file]]                                           
    )
                    
    path_planner_server = IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_path_planner_server),
                    launch_arguments=[['use_sim_time', config_use_sim_time],
                                      ['type_simulation', config_type_simulation]]                                                  
                                                   
    )

    return launch.LaunchDescription([
        arg_map_file,
        arg_use_sim_time,
        arg_type_simulation,
        map_server,
        RosTimer(period=5.0, actions=[path_planner_server])
    ])