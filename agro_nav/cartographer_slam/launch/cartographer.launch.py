import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    #~~~~~~~~~~~~~~~~~~ PACKAGES ~~~~~~~~~~~~~~~

    package_name = 'cartographer_slam'

    #~~~~~~~~~~~~~~~~~~~~~~~~ PATHS ~~~~~~~~~~~~~~~~~~~~~~~~~~~+

    configuration_basename = 'cartographer.lua'
    cartographer_config_dir = os.path.join(get_package_share_directory(package_name), 'config')
    
    rviz_file = os.path.join(get_package_share_directory(package_name), 'rviz', 'rviz_config.rviz')

    #~~~~~~~~~~~~~~~~~~~~~~~~ ARGUMENTS ~~~~~~~~~~~~~~~~~~~~~~~~~~~+

    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='true', 
        description='Parameter to use simulation time or real robot time')
    
    arg_resolution_map = DeclareLaunchArgument(
        'resolution_map', 
        default_value='0.05', 
        description='Map resolution')

    config_use_sim_time = LaunchConfiguration('use_sim_time')
    config_resolution_map = LaunchConfiguration('resolution_map')

    #~~~~~~~~~~~~~~~~~~~~~~~~ NODES ~~~~~~~~~~~~~~~~~~~~~~~~~~~+

    cartographer_node =  Node(
            package='cartographer_ros', 
            executable='cartographer_node', 
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': config_use_sim_time}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename
            ],
            remappings=[('/odom','/odometry/local')]

    )

    cartographer_occupancy_grid_node = Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': config_use_sim_time}],
            arguments=[
                '-resolution', config_resolution_map,
                '-publish_period_sec', '1.0'
            ]
    )
    
    rviz =  Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_file]
    )

    return LaunchDescription([

        arg_use_sim_time,
        arg_resolution_map,

        cartographer_node,
        cartographer_occupancy_grid_node,
        rviz

    ])
