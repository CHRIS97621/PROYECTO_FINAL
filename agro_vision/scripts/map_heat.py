#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import folium 
import os
import json

class HeatMapGeneratorNode(Node):
    def __init__(self):
        super().__init__('heatmap_generator_node')

        # Suscriptor al tópico de datos geográficos con detecciones
        self.subscription = self.create_subscription(
            String,
            '/cacao_geo_data',
            self.data_callback,
            10
        )

        # Lista para almacenar datos
        self.cacao_data = []

        # Ruta para guardar el mapa
        self.map_output = '/home/christopher/cacao_map.html'

    def data_callback(self, msg):
        """Procesa datos de detecciones geográficas y actualiza el mapa."""
        try:
            # Decodificar mensaje JSON
            data = json.loads(msg.data)

            # Añadir datos a la lista
            self.cacao_data.append(data)

            # Generar el mapa con los datos actuales
            self.generar_mapa()

        except Exception as e:
            self.get_logger().error(f"Error en data_callback: {e}")

    def generar_mapa(self):
        """Genera y guarda un mapa interactivo basado en los datos de cacao."""
        if len(self.cacao_data) == 0:
            return

        # Crear un mapa base centrado en el primer punto
        first_data = self.cacao_data[0]
        mapa = folium.Map(location=[first_data['latitude'], first_data['longitude']], zoom_start=18)

        # Añadir marcadores al mapa
        for data in self.cacao_data:
            lat = data['latitude']
            lon = data['longitude']
            sanos = data['cacao_sano']
            enfermos = data['cacao_enfermo']

            popup_text = f"Sanos: {sanos}, Enfermos: {enfermos}"
            color = "green" if sanos > enfermos else "red"
            folium.CircleMarker(
                location=[lat, lon],
                radius=5,
                color=color,
                fill=True,
                fill_color=color,
                fill_opacity=0.7,
                popup=popup_text
            ).add_to(mapa)

        # Guardar el mapa en un archivo HTML
        mapa.save(self.map_output)
        self.get_logger().info(f"Mapa actualizado: {self.map_output}")


def main(args=None):
    rclpy.init(args=args)
    node = HeatMapGeneratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
