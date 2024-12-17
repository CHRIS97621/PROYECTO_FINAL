import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution



def generate_launch_description():


    #~~~~~~~~~~~~~~~~~~ PACKAGES ~~~~~~~~~~~~~~~
    package_name = 'agro_mapviz'
    #~~~~~~~~~~~~~~~~~~~~~~~~ PATHS ~~~~~~~~~~~~~~~~~~~~~~~~~~~+
    mapviz_share = get_package_share_directory(package_name)

    mapviz_yaml = PathJoinSubstitution([mapviz_share, 'config','gps_wpf_demo.mvc'])

    mapviz = Node(
        package="mapviz",
        executable="mapviz",
        name="mapviz",
        parameters=[{"config": mapviz_yaml}]
    )
    swri_transform = Node(
        package="swri_transform_util",
        executable="initialize_origin.py",
        name="initialize_origin",
        remappings=[
            ("fix", "gps/raw"),
        ],
        parameters=[
            {"local_xy_frame": "map"},
            {"local_xy_origin": "auto"},
            #{"local_xy_origins": """[
                #{"name": "swri",
                #"latitude": -12.0597,
                #"longitude": -75.2373,
                #"altitude": 233.719,
                #"heading": 0.0},
                #{"name": "back_40",
                #"latitude": -12.0597,
                #"longitude": -75.2373,
                #"altitude": 200.0,
                #"heading": 0.0}
            #]"""},
        ]
    )

    oritin_to_map = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="swri_transform",
        arguments=["0", "0", "0", "0", "0", "0", "map", "origin"]
    
    )
    return LaunchDescription([
        mapviz,
        swri_transform,
        oritin_to_map
    ])