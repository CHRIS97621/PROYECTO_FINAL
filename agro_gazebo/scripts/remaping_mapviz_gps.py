#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
import math


class NavSatToPose(Node):
    def __init__(self):
        super().__init__('navsat_to_pose_node')

        # Suscriptor al tópico de entrada
        self.subscription = self.create_subscription(
            NavSatFix,
            'goal_pose_gps',  # Nombre del tópico que recibe desde Mapviz
            self.navsat_callback,
            10
        )

        # Publicador al tópico de salida
        self.publisher = self.create_publisher(
            PoseStamped,
            'goal_pose',  # Tópico de salida para Nav2
            10
        )

        # Configuración del origen (punto de referencia)
        self.origin_lat = -12.024824  # Latitud del origen
        self.origin_lon = -77.047441  # Longitud del origen
        self.earth_radius = 6378137.0  # Radio de la Tierra en metros

        self.get_logger().info("Nodo iniciado: Convirtiendo NavSatFix a PoseStamped")

    def navsat_callback(self, msg):
        """
        Callback que recibe NavSatFix y convierte latitud/longitud a X/Y en PoseStamped.
        """
        self.get_logger().info(f"Recibido NavSatFix: [lat: {msg.latitude}, lon: {msg.longitude}]")

        # Convertir lat/lon a X/Y en metros
        x, y = self.latlon_to_xy(msg.latitude, msg.longitude)

        # Crear mensaje PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"  # El frame_id debe coincidir con el global_costmap de Nav2

        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.orientation.w = 1.0  # Sin rotación

        # Publicar el PoseStamped
        self.publisher.publish(pose_msg)
        self.get_logger().info(f"PoseStamped publicado: x={x}, y={y}")

    def latlon_to_xy(self, lat, lon):
        """
        Convierte latitud y longitud a coordenadas X/Y en metros respecto al origen.
        """
        # Diferencia en radianes entre la posición actual y el origen
        delta_lat = math.radians(lat - self.origin_lat)
        delta_lon = math.radians(lon - self.origin_lon)

        # Proyección simple: longitud -> X, latitud -> Y
        x = delta_lon * self.earth_radius * math.cos(math.radians(self.origin_lat))
        y = delta_lat * self.earth_radius
        return x, y


def main(args=None):
    rclpy.init(args=args)
    node = NavSatToPose()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
