#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np

class IMUPolarPlotter(Node):
    def __init__(self):
        super().__init__('imu_polar_plotter')
        
        # Subscripción al tópico IMU
        self.subscription_imu = self.create_subscription(
            Imu,
            'imu',  # Asegúrate de que este sea el nombre correcto del tópico
            self.imu_callback,qos_profile=rclpy.qos.qos_profile_sensor_data
        )
        self.subscription_imu  # Evita que se recolecte como basura
        
        # Subscripción al tópico Odom
        self.subscription_odom = self.create_subscription(
            Odometry,
            'agro_base_controller/odom',  # Asegúrate de que este sea el nombre correcto del tópico
            self.odom_callback,qos_profile=rclpy.qos.qos_profile_sensor_data
        )
        self.subscription_odom  # Evita que se recolecte como basura

        # Subscripción al tópico odometry/local
        self.subscription_local_odom = self.create_subscription(
            Odometry,
            '/odometry/local',  # Asegúrate de que este sea el nombre correcto del tópico
            self.local_odom_callback,qos_profile=rclpy.qos.qos_profile_sensor_data
        )
        self.subscription_local_odom  # Evita que se recolecte como basura

        # Subscripción al tópico odometry/global
        self.subscription_global_odom = self.create_subscription(
            Odometry,
            '/odometry/global',  # Asegúrate de que este sea el nombre correcto del tópico
            self.global_odom_callback,qos_profile=rclpy.qos.qos_profile_sensor_data
        )
        self.subscription_global_odom  # Evita que se recolecte como basura

        # Configuración de los gráficos
        self.fig, ((self.ax_imu, self.ax_odom), (self.ax_local_odom, self.ax_global_odom)) = plt.subplots(2, 2, subplot_kw={'projection': 'polar'})
        self.fig.suptitle('Orientación del Robot', va='bottom')

        self.ax_imu.set_title('IMU (Eje Z)', va='bottom')
        self.ax_odom.set_title('Odom (Eje Z)', va='bottom')
        self.ax_local_odom.set_title('Local Odom (Eje Z)', va='bottom')
        self.ax_global_odom.set_title('Global Odom (Eje Z)', va='bottom')
        
        self.angle_imu = 0.0
        self.angle_odom = 0.0
        self.angle_local_odom = 0.0
        self.angle_global_odom = 0.0

        # Inicia la actualización de los gráficos
        plt.ion()
        plt.show()

    def imu_callback(self, msg: Imu):
        # Extrae la orientación en forma de cuaterniones
        q = msg.orientation

        # Convierte el cuaternión a ángulos de Euler
        t3 = 2.0 * (q.w * q.z + q.x * q.y)
        t4 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.angle_imu = np.arctan2(t3, t4)

        # Actualiza el gráfico
        self.update_plot()

    def odom_callback(self, msg: Odometry):
        # Extrae la orientación en forma de cuaterniones
        q = msg.pose.pose.orientation

        # Convierte el cuaternión a ángulos de Euler
        t3 = 2.0 * (q.w * q.z + q.x * q.y)
        t4 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.angle_odom = np.arctan2(t3, t4)

        # Actualiza el gráfico
        self.update_plot()

    def local_odom_callback(self, msg: Odometry):
        # Extrae la orientación en forma de cuaterniones
        q = msg.pose.pose.orientation

        # Convierte el cuaternión a ángulos de Euler
        t3 = 2.0 * (q.w * q.z + q.x * q.y)
        t4 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.angle_local_odom = np.arctan2(t3, t4)

        # Actualiza el gráfico
        self.update_plot()

    def global_odom_callback(self, msg: Odometry):
        # Extrae la orientación en forma de cuaterniones
        q = msg.pose.pose.orientation

        # Convierte el cuaternión a ángulos de Euler
        t3 = 2.0 * (q.w * q.z + q.x * q.y)
        t4 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.angle_global_odom = np.arctan2(t3, t4)

        # Actualiza el gráfico
        self.update_plot()

    def update_plot(self):
        # Actualiza gráfico de IMU
        self.ax_imu.clear()
        self.ax_imu.set_title('IMU (Eje Z)', va='bottom')
        self.ax_imu.plot([0, self.angle_imu], [0, 1], marker='o')  # Dibuja la orientación
        self.ax_imu.set_rticks([])  # Oculta los radios

        # Actualiza gráfico de Odom
        self.ax_odom.clear()
        self.ax_odom.set_title('Odom (Eje Z)', va='bottom')
        self.ax_odom.plot([0, self.angle_odom], [0, 1], marker='o')  # Dibuja la orientación
        self.ax_odom.set_rticks([])  # Oculta los radios

        # Actualiza gráfico de Local Odom
        self.ax_local_odom.clear()
        self.ax_local_odom.set_title('Local Odom (Eje Z)', va='bottom')
        self.ax_local_odom.plot([0, self.angle_local_odom], [0, 1], marker='o')  # Dibuja la orientación
        self.ax_local_odom.set_rticks([])  # Oculta los radios

        # Actualiza gráfico de Global Odom
        self.ax_global_odom.clear()
        self.ax_global_odom.set_title('Global Odom (Eje Z)', va='bottom')
        self.ax_global_odom.plot([0, self.angle_global_odom], [0, 1], marker='o')  # Dibuja la orientación
        self.ax_global_odom.set_rticks([])  # Oculta los radios

        plt.draw()
        plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    node = IMUPolarPlotter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
