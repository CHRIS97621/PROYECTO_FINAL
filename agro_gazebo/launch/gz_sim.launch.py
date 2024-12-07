from launch import LaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory
def generate_launch_description():

    #~~~~~~~~~~~~~~~~~~ PACKAGES ~~~~~~~~~~~~~~~
    package_description = 'agro_description'
    package_gz = "ros_gz_sim"
    package_world = "agro_gazebo"

    #~~~~~~~~~~~~~~~~~~~~~~~~ PATHS ~~~~~~~~~~~~~~~~~~~~~~~~~
    path_gz_sim = PathJoinSubstitution([FindPackageShare(package_gz),"launch","gz_sim.launch.py",])
    
    path_world_default = PathJoinSubstitution([FindPackageShare(package_world),"models","parcela.sdf",])

    world_str = ParameterValue(path_world_default,value_type=str)

    packagepath = get_package_share_directory('agro_gazebo')  
    #~~~~~~~~~~~~~~~~~~~~~~~~ ARGUMENTS ~~~~~~~~~~~~~~~~~~~~~~~~~~~+

    arg_world = DeclareLaunchArgument(
        'world',
        default_value= path_world_default,
        description='World for simulation'
    )

    config_world = LaunchConfiguration('world')

    #~~~~~~~~~~~~~~~~~~~~~~~~ GZ SIM ~~~~~~~~~~~~~~~~~~~~~~~~~~~+
    gz_args = f"-r {world_str}"

    gz_sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(path_gz_sim),
        launch_arguments=[('gz_args',packagepath + '/world/test.sdf -r -v4 ')]
        
    )

    return LaunchDescription([
        arg_world,
        gz_sim
    ])