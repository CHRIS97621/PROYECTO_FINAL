from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo vision_node.py
        Node(
            package='agro_vision',
            executable='vision_node2.py',
            name='cacao_detector_node',
            output='screen',
            parameters=[
                {'use_sim_time': False}  
            ]
        ),

        # Nodo tfcoordenadas3.py
        Node(
            package='agro_vision',
            executable='gradient.py',
            name='heatmap_node',
            output='screen',
            parameters=[
                {'use_sim_time': False}  
            ]
        )
    ])
