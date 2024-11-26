from launch import LaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():

    #~~~~~~~~~~~~~~~~~~ PACKAGES ~~~~~~~~~~~~~~~
    package_description = 'agro_description'
    package_gz = "ros_gz_sim"
    package_world = "agro_gazebo"

    #~~~~~~~~~~~~~~~~~~~~~~~~ PATHS ~~~~~~~~~~~~~~~~~~~~~~~~~
    path_gz_sim = PathJoinSubstitution([FindPackageShare(package_gz),"launch","gz_sim.launch.py",])
    
    path_world_default = PathJoinSubstitution([FindPackageShare(package_world),"world","test.sdf",])

    #~~~~~~~~~~~~~~~~~~~~~~~~ ARGUMENTS ~~~~~~~~~~~~~~~~~~~~~~~~~~~+

    arg_world = DeclareLaunchArgument(
        'world',
        default_value= path_world_default,
        description='World for simulation'
    )

    config_world = LaunchConfiguration('world')

    #~~~~~~~~~~~~~~~~~~~~~~~~ gz sim ~~~~~~~~~~~~~~~~~~~~~~~~~~~+
    gz_sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(path_gz_sim),
        launch_arguments={
            "gz_args": [path_world_default],
            "on_exit_shutdown": "True",
        }.items(),
    )

    return LaunchDescription([
        arg_world,
        gz_sim
    ])