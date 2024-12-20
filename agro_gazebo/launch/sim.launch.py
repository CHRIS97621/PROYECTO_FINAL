
import os

from launch import LaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare,FindPackagePrefix
from launch_ros.actions import Node



def generate_launch_description():

    #~~~~~~~~~~~~~~~~~~ PACKAGES ~~~~~~~~~~~~~~~
    package_gz_sim = 'agro_gazebo'
    package_spawn = "agro_gazebo"
    package_bringup = "agro_bringup"
    

    #~~~~~~~~~~~~~~~~~~~~~~~~ PATHS ~~~~~~~~~~~~~~~~~~~~~~~~~
    path_gz_sim = PathJoinSubstitution([FindPackageShare(package_gz_sim),"launch","gz_sim.launch.py",])
    path_spawn_robot= PathJoinSubstitution([FindPackageShare(package_spawn),"launch","spawn_robot.launch.py",])
    path_bringup = PathJoinSubstitution([FindPackageShare(package_bringup),"launch","bringup.launch.py",])
    
    current_directory = os.getcwd()
    print(f"Directorio actual: {current_directory}")
    rviz_file = PathJoinSubstitution([current_directory,"src","PROYECTO_FINAL","agro_gazebo","rviz","sim.rviz",])

    #~~~~~~~~~~~~~~~~~~~~~~~~ GZ SIM ~~~~~~~~~~~~~~~~~~~~~~~~~~~+
    gz_sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(path_gz_sim))
    spawn = IncludeLaunchDescription(PythonLaunchDescriptionSource(path_spawn_robot))
    bringup = IncludeLaunchDescription(PythonLaunchDescriptionSource(path_bringup))

    #~~~~~~~~~~~~~~~~~~~~~~~~ RVIZ ~~~~~~~~~~~~~~~~~~~~~~~~~~~+
    rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            emulate_tty=True,
            arguments=['-d', rviz_file],
    )


    return LaunchDescription([
        gz_sim,
        spawn,
        bringup,
        #rviz2
        
    ])