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
            self.imu_callback, qos_profile=rclpy.qos.qos_profile_sensor_data
        )
        self.subscription_imu  # Evita que se recolecte como basura
        
        # Subscripción al tópico Odom
        self.subscription_odom = self.create_subscription(
            Odometry,
            'agro_base_controller/odom',  # Asegúrate de que este sea el nombre correcto del tópico
            self.odom_callback, qos_profile=rclpy.qos.qos_profile_sensor_data
        )
        self.subscription_odom  # Evita que se recolecte como basura

        # Subscripción al tópico odometry/local
        self.subscription_local_odom = self.create_subscription(
            Odometry,
            '/odometry/local',  # Asegúrate de que este sea el nombre correcto del tópico
            self.local_odom_callback, qos_profile=rclpy.qos.qos_profile_sensor_data
        )
        self.subscription_local_odom  # Evita que se recolecte como basura

        # Configuración de los gráficos
        self.fig, ((self.ax_imu, self.ax_odom, self.ax_local_odom), 
                  (self.ax_imu_vel, self.ax_odom_vel, self.ax_local_odom_vel)) = plt.subplots(2, 3, subplot_kw={})
        self.fig.suptitle('Orientación y Velocidad Angular del Robot', va='bottom')

        # Gráficos polares arriba
        self.ax_imu = self.fig.add_subplot(2, 3, 1, polar=True)
        self.ax_odom = self.fig.add_subplot(2, 3, 2, polar=True)
        self.ax_local_odom = self.fig.add_subplot(2, 3, 3, polar=True)

        self.ax_imu.set_title('IMU (Eje Z)', va='bottom')
        self.ax_odom.set_title('Odom (Eje Z)', va='bottom')
        self.ax_local_odom.set_title('Local Odom (Eje Z)', va='bottom')

        self.ax_imu_vel.set_title('Velocidad Z IMU', va='bottom')
        self.ax_odom_vel.set_title('Velocidad Z Odom', va='bottom')
        self.ax_local_odom_vel.set_title('Velocidad Z Local Odom', va='bottom')
        
        self.angle_imu = 0.0
        self.angle_odom = 0.0
        self.angle_local_odom = 0.0

        self.vel_imu = 0.0
        self.vel_odom = 0.0
        self.vel_local_odom = 0.0

        self.time_imu = []
        self.time_odom = []
        self.time_local_odom = []

        self.vel_z_imu = []
        self.vel_z_odom = []
        self.vel_z_local_odom = []

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

        # Actualiza la velocidad angular en Z
        self.vel_imu = msg.angular_velocity.z
        current_time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
        self.time_imu.append(current_time)
        self.vel_z_imu.append(self.vel_imu)

        # Mantén solo los últimos 5 segundos
        self.trim_data(self.time_imu, self.vel_z_imu)

        # Actualiza el gráfico
        self.update_plot()

    def odom_callback(self, msg: Odometry):
        # Extrae la orientación en forma de cuaterniones
        q = msg.pose.pose.orientation

        # Convierte el cuaternión a ángulos de Euler
        t3 = 2.0 * (q.w * q.z + q.x * q.y)
        t4 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.angle_odom = np.arctan2(t3, t4)

        # Actualiza la velocidad angular en Z
        self.vel_odom = msg.twist.twist.angular.z
        current_time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
        self.time_odom.append(current_time)
        self.vel_z_odom.append(self.vel_odom)

        # Mantén solo los últimos 5 segundos
        self.trim_data(self.time_odom, self.vel_z_odom)

        # Actualiza el gráfico
        self.update_plot()

    def local_odom_callback(self, msg: Odometry):
        # Extrae la orientación en forma de cuaterniones
        q = msg.pose.pose.orientation

        # Convierte el cuaternión a ángulos de Euler
        t3 = 2.0 * (q.w * q.z + q.x * q.y)
        t4 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.angle_local_odom = np.arctan2(t3, t4)

        # Actualiza la velocidad angular en Z
        self.vel_local_odom = msg.twist.twist.angular.z
        current_time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
        self.time_local_odom.append(current_time)
        self.vel_z_local_odom.append(self.vel_local_odom)

        # Mantén solo los últimos 5 segundos
        self.trim_data(self.time_local_odom, self.vel_z_local_odom)

        # Actualiza el gráfico
        self.update_plot()

    def trim_data(self, time_list, value_list, window=2.0):
        """Elimina datos más antiguos que el tamaño de la ventana de tiempo."""
        current_time = time_list[-1] if time_list else 0
        while time_list and (current_time - time_list[0] > window):
            time_list.pop(0)
            value_list.pop(0)

    def update_plot(self):
        # Actualiza gráfico de orientación IMU
        self.ax_imu.clear()
        self.ax_imu.set_title('IMU (Eje Z)', va='bottom')
        self.ax_imu.plot([0, self.angle_imu], [0, 1], marker='o')  # Dibuja la orientación
        self.ax_imu.set_rticks([])  # Oculta los radios

        # Actualiza gráfico de orientación Odom
        self.ax_odom.clear()
        self.ax_odom.set_title('Odom (Eje Z)', va='bottom')
        self.ax_odom.plot([0, self.angle_odom], [0, 1], marker='o')  # Dibuja la orientación
        self.ax_odom.set_rticks([])  # Oculta los radios

        # Actualiza gráfico de orientación Local Odom
        self.ax_local_odom.clear()
        self.ax_local_odom.set_title('Local Odom (Eje Z)', va='bottom')
        self.ax_local_odom.plot([0, self.angle_local_odom], [0, 1], marker='o')  # Dibuja la orientación
        self.ax_local_odom.set_rticks([])  # Oculta los radios

        # Actualiza gráfico de velocidad angular IMU
        self.ax_imu_vel.clear()
        self.ax_imu_vel.set_title('Velocidad Z IMU', va='bottom')
        self.ax_imu_vel.set_ylim(-1, 1)  # Fija el rango de -10 a 10
        self.ax_imu_vel.plot(self.time_imu, self.vel_z_imu, marker='o')

        # Actualiza gráfico de velocidad angular Odom
        self.ax_odom_vel.clear()
        self.ax_odom_vel.set_title('Velocidad Z Odom', va='bottom')
        self.ax_odom_vel.set_ylim(-1, 1)  # Fija el rango de -10 a 10
        self.ax_odom_vel.plot(self.time_odom, self.vel_z_odom, marker='o')

        # Actualiza gráfico de velocidad angular Local Odom
        self.ax_local_odom_vel.clear()
        self.ax_local_odom_vel.set_title('Velocidad Z Local Odom', va='bottom')
        self.ax_local_odom_vel.set_ylim(-1, 1)  # Fija el rango de -10 a 10
        self.ax_local_odom_vel.plot(self.time_local_odom, self.vel_z_local_odom, marker='o')

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
