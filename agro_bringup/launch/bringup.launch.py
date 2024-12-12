from launch import LaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable, Command
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler 
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    #~~~~~~~~~~~~~~~~~~ PACKAGES ~~~~~~~~~~~~~~~
    package_description = 'agro_description'
    package_gz = "ros_gz_sim"
    package_world = "agro_gazebo"
    package_bringup = "agro_bringup"

    #~~~~~~~~~~~~~~~~~~~~~~~~ PATHS ~~~~~~~~~~~~~~~~~~~~~~~~~
    path_gz_sim = PathJoinSubstitution([FindPackageShare(package_gz),"launch","gz_sim.launch.py",])

    path_xacro = PathJoinSubstitution([FindPackageShare(package_description),'urdf','agro.urdf.xacro'])

    path_bringup = PathJoinSubstitution([FindPackageShare(package_bringup),'config','agro.urdf.xacro'])

    topics_for_bridge = PathJoinSubstitution([FindPackageShare(package_bringup),'config','topics_for_bridge.yaml'])
    #~~~~~~~~~~~~~~~~~~~~~~~~ NODES ~~~~~~~~~~~~~~~~~~~~~~~~~~~+

    robot_description_content = Command([PathJoinSubstitution([FindExecutable(name="xacro")])," ",path_xacro])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description_content},
            {'use_sim_time': True}],
        emulate_tty=True,
    )

    ros_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        parameters=[{"config_file": topics_for_bridge},
            {'use_sim_time': True}],
        output="screen",
        emulate_tty="true"

    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        parameters=[
            {'use_sim_time': True}],
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "120"
        ],
        output= "screen"
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        name="agro_base_controller",
        executable="spawner",
        arguments=[
            "agro_base_controller",
            "--controller-manager-timeout",
            "120"
        ],
        parameters=[
            {'use_sim_time': True}],
        output= "screen"
    )
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    return LaunchDescription([
        robot_state_publisher,
        ros_bridge_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner

    ])