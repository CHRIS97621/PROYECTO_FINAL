import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    #~~~~~~~~~~~~~~~~~~ PACKAGES ~~~~~~~~~~~~~~~
    
    package_name = 'path_planner_server'

    #~~~~~~~~~~~~~~~~~~~~~~~~ PATHS ~~~~~~~~~~~~~~~~~~~~~~~~~~~+

    default_value_bt_xml_file =  PathJoinSubstitution([get_package_share_directory(package_name),'config', 'navigate_w_replanning_and_recovery.xml'])
    default_value_bt_poses_xml_file =  PathJoinSubstitution([get_package_share_directory(package_name),'config', 'navigate_w_replanning_and_recovery_poses.xml'])
    
    #~~~~~~~~~~~~~~~~~~~~~~~~ ARGUMENTS ~~~~~~~~~~~~~~~~~~~~~~~~~~~+

    arg_bt_xml_file = DeclareLaunchArgument(
        'bt_xml_file',
        default_value= default_value_bt_xml_file,
        description='Path to the behavior tree XML file for NavigateToPose'
    )

    arg_bt_poses_xml_file = DeclareLaunchArgument(
        'bt_poses_xml_file',
        default_value= default_value_bt_poses_xml_file,
        description='Path to the behavior tree XML file for NavigateToPose'
    )

    arg_type_simulation = DeclareLaunchArgument(
        'type_simulation',
        default_value='sim_robot',
        choices=['sim_robot', 'real_robot'],
        description='Use for simlation robot or real robot'
    )

    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        choices= ['True', 'False'],
        default_value= 'True',
        description='Path to the map select'
    )

    config_bt_xml_file = LaunchConfiguration('bt_xml_file')
    config_bt_poses_xml_file = LaunchConfiguration('bt_poses_xml_file')
    config_type_simulation = LaunchConfiguration('type_simulation')
    config_use_sim_time = LaunchConfiguration('use_sim_time')
    
    #~~~~~~~~~~~~~~~~~~~~~~~~ PATHS ~~~~~~~~~~~~~~~~~~~~~~~~~~~+

    package_share = get_package_share_directory(package_name)
    
    rviz_file = os.path.join(package_share,'rviz','pathplanning.rviz')

    controller_yaml = PathJoinSubstitution([package_share, 'config',config_type_simulation, 'controller.yaml'])
    bt_navigator_yaml = PathJoinSubstitution([package_share, 'config',config_type_simulation, 'bt.yaml'])
    planner_yaml = PathJoinSubstitution([package_share, 'config',config_type_simulation, 'planner_server.yaml'])
    recovery_yaml = PathJoinSubstitution([package_share, 'config',config_type_simulation, 'recovery.yaml'])

    #~~~~~~~~~~~~~~~~~~~~~~~~ NODES ~~~~~~~~~~~~~~~~~~~~~~~~~~~+
    nav2_controller = Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            emulate_tty=True,
            parameters=[controller_yaml],
            remappings=[('/cmd_vel','/agro_base_controller/cmd_vel')]
    )
    mav2_planner = Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            emulate_tty=True,
            parameters=[planner_yaml]
    )
            
    nav2_behaviors = Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            emulate_tty=True,
            parameters=[recovery_yaml],
            output='screen'
    )

    nav2_bt_navigator = Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            emulate_tty=True,
            parameters=[bt_navigator_yaml,{'default_nav_through_poses_bt_xml':config_bt_poses_xml_file }]
    )

    nav2_lifecycle_manager = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            emulate_tty=True,
            parameters=[{'autostart': True},
                        {'use_sim_time': config_use_sim_time},
                        {'node_names': ['planner_server',
                                        'controller_server',
                                        'behavior_server',
                                        'bt_navigator'
                                        ]}]
    )

    rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            emulate_tty=True,
            arguments=['-d', rviz_file],
    )

    return LaunchDescription([ 
        arg_type_simulation,
        arg_bt_xml_file,
        arg_bt_poses_xml_file,
        arg_use_sim_time,

        nav2_controller,
        mav2_planner, 
        nav2_behaviors,
        nav2_bt_navigator,
        nav2_lifecycle_manager,    
        rviz
    ])
