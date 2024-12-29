from launch import LaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable, Command
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler 
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

from launch.conditions import IfCondition, UnlessCondition

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

    #~~~~~~~~~~~~~~~~~~~~~~~~ ARGUMENTS ~~~~~~~~~~~~~~~~~~~~~~~~~~~+

    arg_use_sim = DeclareLaunchArgument(
        'use_sim',
        default_value= "false",
        description='Use simlation or not'
    )

    config_use_sim = LaunchConfiguration('use_sim')


    #~~~~~~~~~~~~~~~~~~~~~~~~ NODES ~~~~~~~~~~~~~~~~~~~~~~~~~~~+
    robot_description_content = ParameterValue(Command(['xacro ', path_xacro, " use_sim:=",config_use_sim]), value_type=str)

    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("agro_bringup"),
            "config",
            "diff_drive_controller_copy.yaml",
        ]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description_content},
            {'use_sim_time': config_use_sim}],
        emulate_tty=True,
    )
     #~~~~~~~~~~~~~~~~~~~~~~~
    ros_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        parameters=[{"config_file": topics_for_bridge},
            {'use_sim_time': config_use_sim}],
        output="screen",
        emulate_tty="true",
        condition=IfCondition(LaunchConfiguration("use_sim")),
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        remappings=[
            ("~/motors_cmd", "/_motors_cmd"),
            ("~/motors_response", "/_motors_response"),
            ("/agro_base_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
        condition=UnlessCondition(LaunchConfiguration("use_sim")),
    )
     #~~~~~~~~~~~~~~~~~~~~~~~
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        parameters=[
            {'use_sim_time': config_use_sim}],
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
            {'use_sim_time': config_use_sim}],
        output= "screen"
    )
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )
    #~~~~~~~~~~~~~~~~~~ LAUNCH ~~~~~~~~~~~~~~~
    path_launch_ekf = PathJoinSubstitution([ get_package_share_directory(package_bringup),'launch','dual_ekf_navsat.launch.py'])

    launch_ekf = IncludeLaunchDescription(PythonLaunchDescriptionSource(path_launch_ekf))

    delay_ekf_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[launch_ekf],
        )
    )

    return LaunchDescription([
        arg_use_sim,
        control_node,
        robot_state_publisher,
        ros_bridge_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_ekf_after_robot_controller_spawner
    ])