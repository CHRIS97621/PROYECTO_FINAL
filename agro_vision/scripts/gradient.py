#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import matplotlib.pyplot as plt
import json
import math
import numpy as np
from scipy.interpolate import griddata


class HeatmapNode(Node):
    def __init__(self):
        super().__init__('heatmap_node')

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

        # Lista para almacenar los datos para el mapa de calor
        self.data_points = []

        # Variables para almacenar las últimas detecciones
        self.num_sanos = 0
        self.num_enfermos = 0

        # Última posición registrada
        self.last_position = None
        self.min_distance = 2.0  # Distancia mínima entre puntos (en metros)

    def detection_callback(self, msg):
        """Procesa las detecciones de cacao."""
        try:
            detections = json.loads(msg.data)
            self.num_sanos = detections.get("cacao_sano", 0)
            self.num_enfermos = detections.get("cacao_enfermo", 0)
        except Exception as e:
            self.get_logger().error(f"Error al procesar detecciones: {e}")

    def odom_callback(self, msg):
        """Procesa datos de odometría."""
        try:
            # Obtener posición en coordenadas x, y
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y

            # Verificar si la distancia es suficiente para registrar un nuevo punto
            if self.last_position:
                distancia = self._distancia(self.last_position[0], self.last_position[1], x, y)
                if distancia < self.min_distance:
                    return

            # Calcular el nivel de contagio
            total = self.num_sanos + self.num_enfermos
            nivel_contagio = (self.num_enfermos / total) * 100 if total > 0 else 0

            # Registrar el nuevo punto
            self.data_points.append((x, y, self.num_sanos, self.num_enfermos, nivel_contagio))

            # Actualizar la última posición registrada
            self.last_position = (x, y)

            self.get_logger().info(f"Punto registrado: x={x}, y={y}, Sanos={self.num_sanos}, Enfermos={self.num_enfermos}, Contagio={nivel_contagio:.2f}%")

            # Generar el mapa de calor
            self.generate_heatmap()

        except Exception as e:
            self.get_logger().error(f"Error en odom_callback: {e}")

    def _distancia(self, x1, y1, x2, y2):
        """Calcula la distancia euclidiana entre dos puntos."""
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def generate_heatmap(self):
        """Genera un mapa de calor continuo basado en los datos recopilados."""
        # Verifica si hay suficientes puntos para la interpolación
        if len(self.data_points) < 4:
            self.get_logger().info(f"No hay suficientes puntos para generar el mapa de calor. Puntos actuales: {len(self.data_points)}")
            return

        # Extraer datos para la gráfica
        x_coords = [point[0] for point in self.data_points]
        y_coords = [point[1] for point in self.data_points]
        contagios = [point[4] for point in self.data_points]  # Nivel de contagio

        # Crear una malla regular para interpolar los datos
        grid_x, grid_y = np.mgrid[min(x_coords):max(x_coords):100j, min(y_coords):max(y_coords):100j]

        # Interpolar los datos
        grid_z = griddata((x_coords, y_coords), contagios, (grid_x, grid_y), method='cubic')

        # Crear la gráfica del mapa de calor
        plt.figure(figsize=(10, 8))
        heatmap = plt.contourf(grid_x, grid_y, grid_z, levels=100, cmap='RdYlGn_r')
        plt.colorbar(heatmap, label='Nivel de Contagio (%)')
        plt.title('Mapa de Calor Continuo de Cacao Enfermo y Sano')
        plt.xlabel('Coordenada X (metros)')
        plt.ylabel('Coordenada Y (metros)')
        plt.grid(True)

        # Guardar la gráfica en un archivo
        plt.savefig('/home/christopher/heatmap_cacao4.png')
        plt.close()

        self.get_logger().info("Mapa de calor continuo actualizado y guardado como heatmap_cacao3.png")


def main(args=None):
    rclpy.init(args=args)
    node = HeatmapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
