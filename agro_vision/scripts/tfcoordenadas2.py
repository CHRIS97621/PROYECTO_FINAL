#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
import pyproj
import json
import math

class OdomToGeoNode(Node):
    def __init__(self):
        super().__init__('odom_to_geo_node')

        # Suscriptor a odometría
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odometry/global',
            self.odom_callback,
            10
        )

        # Suscriptor a detecciones de cacao
        self.detection_subscription = self.create_subscription(
            String,
            '/cacao_detections',
            self.detection_callback,
            10
        )

        # Publicador de datos combinados
        self.geo_data_publisher = self.create_publisher(String, '/cacao_geo_data', 10)

        # Buffer para transformaciones
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Proyección a coordenadas geográficas (UTM a WGS84)
        self.proj = pyproj.Proj(proj="utm", zone=32, ellps="WGS84")

        # Coordenada base inicial (latitud y longitud conocidas)
        self.base_latitude = -12.024824  # Latitud inicial
        self.base_longitude = -77.047441  # Longitud inicial

        # Convertir coordenada base a UTM
        self.base_utm_x, self.base_utm_y = self.proj(self.base_latitude, self.base_longitude)

        # Última posición registrada
        self.last_position = None
        self.min_distance = 1  # Distancia mínima entre puntos

        # Variables para almacenar las últimas detecciones
        self.num_sanos = 0
        self.num_enfermos = 0

    def detection_callback(self, msg):
        """Callback para procesar detecciones de cacao."""
        try:
            detections = json.loads(msg.data)
            self.num_sanos = detections.get("cacao_sano", 0)
            self.num_enfermos = detections.get("cacao_enfermo", 0)
        except Exception as e:
            self.get_logger().error(f"Error al procesar detecciones: {e}")

    def odom_callback(self, msg):
        """Callback para procesar datos de odometría."""
        try:
            # Obtener posición en odometría relativa al marco inicial
            relative_x = msg.pose.pose.position.x
            relative_y = msg.pose.pose.position.y

            # Transformar posición relativa a UTM absoluta usando la base
            utm_x = self.base_utm_x + relative_x
            utm_y = self.base_utm_y + relative_y

            # Verificar si la distancia es suficiente para publicar un nuevo punto
            if self.last_position:
                distancia = self._distancia(self.last_position[0], self.last_position[1], relative_x, relative_y)
                if distancia < self.min_distance:
                    return

            # Transformar coordenadas UTM a latitud y longitud
            lat, lon = self.proj(utm_x, utm_y, inverse=True)

            # Crear mensaje con datos combinados
            data = {
                "latitude": lat,
                "longitude": lon,
                "cacao_sano": self.num_sanos,
                "cacao_enfermo": self.num_enfermos
            }

            # Publicar datos geográficos con detecciones
            geo_message = String()
            geo_message.data = json.dumps(data)
            self.geo_data_publisher.publish(geo_message)
            self.get_logger().info(f"Datos publicados: {geo_message.data}")

            # Actualizar última posición
            self.last_position = (relative_x, relative_y)

        except Exception as e:
            self.get_logger().error(f"Error en odom_callback: {e}")

    def _distancia(self, x1, y1, x2, y2):
        """Calcula la distancia euclidiana entre dos puntos."""
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToGeoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
