#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped

class PointProcessor(Node):
    def __init__(self):
        super().__init__('point_processor')

        # Suscriptor al tópico de entrada
        self.subscription = self.create_subscription(
            PointStamped,
            'goal_pose_mapviz',  # Nombre del tópico al que se suscribe
            self.point_callback,
            10
        )
        self.subscription  # Evitar que el garbage collector elimine la suscripción

        # Publicador al tópico de salida
        self.publisher = self.create_publisher(
            PoseStamped,
            'goal_pose',  # Nombre del tópico al que publica
            10
        )

    def point_callback(self, msg):
        self.get_logger().info(f'Recibido punto: [x: {msg.point.x}, y: {msg.point.y}, z: {msg.point.z}]')

        # Procesar el mensaje recibido (por ejemplo, incrementar las coordenadas)
        new_msg = PoseStamped()
        new_msg.header = msg.header  # Copiar el encabezado para mantener la información temporal
        new_msg.pose.position.x = msg.point.x 
        new_msg.pose.position.y = msg.point.y 
        new_msg.pose.orientation.x = 0.0
        new_msg.pose.orientation.y = 0.0
        new_msg.pose.orientation.z = 0.0
        new_msg.pose.orientation.w = 1.0

        # Publicar el mensaje procesado
        self.publisher.publish(new_msg)
        self.get_logger().info(f'Publicado punto: [x: {new_msg.pose.position.x}, y: {new_msg.pose.position.x}, z: {new_msg.pose.position.x}]')

def main(args=None):
    rclpy.init(args=args)
    
    point_processor = PointProcessor()

    try:
        rclpy.spin(point_processor)
    except KeyboardInterrupt:
        pass
    finally:
        point_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
