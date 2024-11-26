from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    package_name = "agro_description"


    #~~~~~~~~~~~~~~~~~~~~~~~~ PATHS ~~~~~~~~~~~~~~~~~~~~~~~~~~~+
    package_share = get_package_share_directory(package_name)
    
    robot_desc_path = PathJoinSubstitution([package_share,'urdf', 'agro.urdf.xacro'])
    rviz_file = PathJoinSubstitution([package_share,'rviz','display.rviz'])
    #rviz_file = PathJoinSubstitution(['rviz','display.rviz'])
    # Convierto el archivo xacro a un string
    robot_description = ParameterValue(Command(['xacro ', robot_desc_path]), value_type=str)
    
    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            emulate_tty=True,
            parameters=[{'use_sim_time': False, 'robot_description':robot_description}],
            output='screen'
    )
    rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            emulate_tty=True,
            arguments=['-d', rviz_file],
    )
    joint_state_publisher_gui = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
          )
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz
    ])


