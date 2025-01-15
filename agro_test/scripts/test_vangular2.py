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
            'imu',  # Ajusta según el nombre correcto del tópico IMU en tu sistema
            self.imu_callback, 
            qos_profile=rclpy.qos.qos_profile_sensor_data
        )
        self.subscription_imu  # Evita que se recolecte como basura
        
        # Subscripción al tópico Odom
        self.subscription_odom = self.create_subscription(
            Odometry,
            'agro_base_controller/odom',  # Ajusta según el nombre correcto del tópico Odom en tu sistema
            self.odom_callback, 
            qos_profile=rclpy.qos.qos_profile_sensor_data
        )
        self.subscription_odom  # Evita que se recolecte como basura

        # Subscripción al tópico odometry/local
        self.subscription_local_odom = self.create_subscription(
            Odometry,
            '/odometry/local',  # Ajusta según el nombre correcto del tópico local
            self.local_odom_callback, 
            qos_profile=rclpy.qos.qos_profile_sensor_data
        )
        self.subscription_local_odom  # Evita que se recolecte como basura

        # Subscripción al tópico odometry/global (nuevo)
        self.subscription_global_odom = self.create_subscription(
            Odometry,
            '/odometry/global',  # Ajusta según el nombre correcto del tópico global
            self.global_odom_callback,
            qos_profile=rclpy.qos.qos_profile_sensor_data
        )
        self.subscription_global_odom  # Evita que se recolecte como basura

        # Configuración de la figura con 2 filas y 4 columnas
        self.fig, ((self.ax_imu, self.ax_odom, self.ax_local_odom, self.ax_global_odom),
                    (self.ax_imu_vel, self.ax_odom_vel, self.ax_local_odom_vel, self.ax_global_odom_vel)) = plt.subplots(2, 4, subplot_kw={'projection': 'polar'})
        self.fig.suptitle('Orientación y Velocidad Angular del Robot', va='bottom')

        # Ajusta a gráfico polar únicamente las tres primeras columnas de la fila superior;
        # La cuarta (global) también la configuramos como polar
        self.ax_imu.set_title('IMU (Eje Z)', va='bottom')
        self.ax_odom.set_title('Odom (Eje Z)', va='bottom')
        self.ax_local_odom.set_title('Local Odom (Eje Z)', va='bottom')
        self.ax_global_odom.set_title('Global Odom (Eje Z)', va='bottom')

        # Abajo, gráficas de velocidad vs tiempo (estas NO son polares):
        # Cambiamos `projection='polar'` manualmente a None, pues matplotlib usa
        # subplots en todas si no se especifica. Lo más sencillo es recrearlas,
        # pero haremos un pequeño truco para convertirlas a ejes cartesianos:
        self.ax_imu_vel.remove()
        self.ax_odom_vel.remove()
        self.ax_local_odom_vel.remove()
        self.ax_global_odom_vel.remove()

        self.ax_imu_vel = self.fig.add_subplot(2, 4, 5)   
        self.ax_odom_vel = self.fig.add_subplot(2, 4, 6)
        self.ax_local_odom_vel = self.fig.add_subplot(2, 4, 7)
        self.ax_global_odom_vel = self.fig.add_subplot(2, 4, 8)

        self.ax_imu_vel.set_title('Vel. Z IMU', va='bottom')
        self.ax_odom_vel.set_title('Vel. Z Odom', va='bottom')
        self.ax_local_odom_vel.set_title('Vel. Z Local', va='bottom')
        self.ax_global_odom_vel.set_title('Vel. Z Global', va='bottom')

        # Variables de orientación (ángulo) y velocidad angular
        self.angle_imu = 0.0
        self.angle_odom = 0.0
        self.angle_local_odom = 0.0
        self.angle_global_odom = 0.0

        self.vel_imu = 0.0
        self.vel_odom = 0.0
        self.vel_local_odom = 0.0
        self.vel_global_odom = 0.0

        # Arreglos para graficar el historial de velocidad vs tiempo
        self.time_imu = []
        self.vel_z_imu = []

        self.time_odom = []
        self.vel_z_odom = []

        self.time_local_odom = []
        self.vel_z_local_odom = []

        self.time_global_odom = []
        self.vel_z_global_odom = []

        # Inicia la actualización de los gráficos en modo interactivo
        plt.ion()
        plt.show()

    # ----------------- Callbacks -------------------
    def imu_callback(self, msg: Imu):
        # Extrae la orientación en forma de cuaterniones
        q = msg.orientation
        # Convierte el cuaternión a ángulos de Euler (solo yaw)
        t3 = 2.0 * (q.w * q.z + q.x * q.y)
        t4 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.angle_imu = np.arctan2(t3, t4)

        # Extrae velocidad angular en Z
        self.vel_imu = msg.angular_velocity.z
        # Timestamp actual
        current_time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9

        self.time_imu.append(current_time)
        self.vel_z_imu.append(self.vel_imu)

        # Limpia datos mayores a la ventana de 5 s (opcional, puedes ajustarla)
        self.trim_data(self.time_imu, self.vel_z_imu, window=5.0)

        # Actualiza las gráficas
        self.update_plot()

    def odom_callback(self, msg: Odometry):
        # Extrae la orientación en forma de cuaterniones
        q = msg.pose.pose.orientation
        t3 = 2.0 * (q.w * q.z + q.x * q.y)
        t4 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.angle_odom = np.arctan2(t3, t4)

        # Velocidad angular en Z
        self.vel_odom = msg.twist.twist.angular.z
        # Timestamp
        current_time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9

        self.time_odom.append(current_time)
        self.vel_z_odom.append(self.vel_odom)

        self.trim_data(self.time_odom, self.vel_z_odom, window=5.0)
        self.update_plot()

    def local_odom_callback(self, msg: Odometry):
        # Extrae la orientación en forma de cuaterniones
        q = msg.pose.pose.orientation
        t3 = 2.0 * (q.w * q.z + q.x * q.y)
        t4 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.angle_local_odom = np.arctan2(t3, t4)

        # Velocidad angular en Z
        self.vel_local_odom = msg.twist.twist.angular.z
        # Timestamp
        current_time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9

        self.time_local_odom.append(current_time)
        self.vel_z_local_odom.append(self.vel_local_odom)

        self.trim_data(self.time_local_odom, self.vel_z_local_odom, window=5.0)
        self.update_plot()

    def global_odom_callback(self, msg: Odometry):
        # Extrae la orientación en forma de cuaterniones
        q = msg.pose.pose.orientation
        t3 = 2.0 * (q.w * q.z + q.x * q.y)
        t4 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.angle_global_odom = np.arctan2(t3, t4)

        # Velocidad angular en Z
        self.vel_global_odom = msg.twist.twist.angular.z
        # Timestamp
        current_time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9

        self.time_global_odom.append(current_time)
        self.vel_z_global_odom.append(self.vel_global_odom)

        self.trim_data(self.time_global_odom, self.vel_z_global_odom, window=5.0)
        self.update_plot()

    # ------------------------------------------------

    def trim_data(self, time_list, value_list, window=2.0):
        """Elimina datos más antiguos que el tamaño de la ventana de tiempo (en segundos)."""
        if not time_list:
            return
        current_time = time_list[-1]
        while time_list and (current_time - time_list[0] > window):
            time_list.pop(0)
            value_list.pop(0)

    def update_plot(self):
        # --- Orientación en la fila superior (gráficos polares) ---
        # IMU
        self.ax_imu.clear()
        self.ax_imu.set_title('IMU (Eje Z)', va='bottom')
        self.ax_imu.plot([0, self.angle_imu], [0, 1], marker='o')
        self.ax_imu.set_rticks([])  # Opcional: quita marcas radiales

        # Odom
        self.ax_odom.clear()
        self.ax_odom.set_title('Odom (Eje Z)', va='bottom')
        self.ax_odom.plot([0, self.angle_odom], [0, 1], marker='o')
        self.ax_odom.set_rticks([])

        # Local Odom
        self.ax_local_odom.clear()
        self.ax_local_odom.set_title('Local Odom (Eje Z)', va='bottom')
        self.ax_local_odom.plot([0, self.angle_local_odom], [0, 1], marker='o')
        self.ax_local_odom.set_rticks([])

        # Global Odom
        self.ax_global_odom.clear()
        self.ax_global_odom.set_title('Global Odom (Eje Z)', va='bottom')
        self.ax_global_odom.plot([0, self.angle_global_odom], [0, 1], marker='o')
        self.ax_global_odom.set_rticks([])

        # --- Velocidad Angular en la fila inferior (gráficos cartesianos) ---
        # IMU
        self.ax_imu_vel.clear()
        self.ax_imu_vel.set_title('Vel. Z IMU', va='bottom')
        self.ax_imu_vel.set_ylim(-1, 1)  
        self.ax_imu_vel.plot(self.time_imu, self.vel_z_imu, marker='o')

        # Odom
        self.ax_odom_vel.clear()
        self.ax_odom_vel.set_title('Vel. Z Odom', va='bottom')
        self.ax_odom_vel.set_ylim(-1, 1)
        self.ax_odom_vel.plot(self.time_odom, self.vel_z_odom, marker='o')

        # Local Odom
        self.ax_local_odom_vel.clear()
        self.ax_local_odom_vel.set_title('Vel. Z Local', va='bottom')
        self.ax_local_odom_vel.set_ylim(-1, 1)
        self.ax_local_odom_vel.plot(self.time_local_odom, self.vel_z_local_odom, marker='o')

        # Global Odom
        self.ax_global_odom_vel.clear()
        self.ax_global_odom_vel.set_title('Vel. Z Global', va='bottom')
        self.ax_global_odom_vel.set_ylim(-1, 1)
        self.ax_global_odom_vel.plot(self.time_global_odom, self.vel_z_global_odom, marker='o')

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
