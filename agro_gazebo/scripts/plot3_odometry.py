#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray  # Importar PoseArray para el puente de Gazebo
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('odom_subscriber')

        # Suscriptores para ambos tópicos
        self.local_odom_sub = self.create_subscription(
            Odometry,
            '/odometry/local',
            self.local_odom_callback,
            10)

        self.sim_odom_sub = self.create_subscription(
            Odometry,
            '/agro_base_controller/odom',
            self.sim_odom_callback,
            10)

        self.pose_sub = self.create_subscription(
            PoseArray,
            '/world/empty/pose/info',
            self.pose_callback,
            10)

        self.global_odom_sub = self.create_subscription(
            Odometry,
            '/odometry/global',
            self.global_odom_callback,
            10)

        # Variables para almacenar coordenadas
        self.local_x, self.local_y = [], []
        self.sim_x, self.sim_y = [], []
        self.robot_agro_x, self.robot_agro_y = [], []
        self.global_x, self.global_y = [], []

        # Configuración inicial de la gráfica
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        self.local_line, = self.ax.plot([], [], label="Kalman Filter (/odometry/local)", color="blue")
        self.sim_line, = self.ax.plot([], [], label="Gazebo Odom (/agro_base_controller/odom)", color="red")
        self.robot_agro_line, = self.ax.plot([], [], label="robot_agro (/world/empty/pose/info)", color="green")
        self.global_line, = self.ax.plot([], [], label="Global Odom (/odometry/global)", color="purple")

        self.ax.set_title("Odometry Visualization")
        self.ax.set_xlabel("X Coordinate")
        self.ax.set_ylabel("Y Coordinate")
        self.ax.legend()
        self.ax.grid()

        # Fijar los límites del gráfico
        self.ax.set_xlim(-10, 10)  # Límites de -10m a 10m en el eje X
        self.ax.set_ylim(-10, 10)  # Límites de -10m a 10m en el eje Y

        # Manejo de cierre de la gráfica
        self.fig.canvas.mpl_connect('close_event', self.handle_close)

        self.running = True  # Estado del nodo

    def local_odom_callback(self, msg):
        # Extraer coordenadas de /odometry/local
        self.local_x.append(msg.pose.pose.position.x)
        self.local_y.append(msg.pose.pose.position.y)

    def sim_odom_callback(self, msg):
        # Extraer coordenadas de /agro_base_controller/odom
        self.sim_x.append(msg.pose.pose.position.x)
        self.sim_y.append(msg.pose.pose.position.y)

    def pose_callback(self, msg):
        robot_index = 1  # Cambia esto si `robot_agro` no está en el índice 0
        try:
            x = msg.poses[robot_index].position.x
            y = msg.poses[robot_index].position.y
            self.robot_agro_x.append(x)
            self.robot_agro_y.append(y)
        except IndexError:
            self.get_logger().warn("Índice fuera de rango en PoseArray")

    def global_odom_callback(self, msg):
        # Extraer coordenadas de /odometry/global
        self.global_x.append(msg.pose.pose.position.x)
        self.global_y.append(msg.pose.pose.position.y)

    def update_plot(self, frame):
        # Actualizar datos en la gráfica
        self.local_line.set_data(self.local_x, self.local_y)
        self.sim_line.set_data(self.sim_x, self.sim_y)
        self.robot_agro_line.set_data(self.robot_agro_x, self.robot_agro_y)
        self.global_line.set_data(self.global_x, self.global_y)

        return self.local_line, self.sim_line, self.robot_agro_line, self.global_line

    def handle_close(self, event):
        # Cerrar el nodo y detener el programa al cerrar la ventana de la gráfica
        self.get_logger().info('Ventana de la gráfica cerrada. Terminando nodo...')
        self.running = False

def main(args=None):
    rclpy.init(args=args)
    node = OdomSubscriber()

    # Configurar animación para la gráfica en tiempo real
    ani = FuncAnimation(node.fig, node.update_plot, interval=100)  # Intervalo de actualización en ms

    try:
        # Mostrar la gráfica en tiempo real
        plt.show(block=False)  # Permitir que el flujo del programa continúe
        while rclpy.ok() and node.running:
            rclpy.spin_once(node, timeout_sec=0.1)  # Procesar mensajes continuamente
            node.fig.canvas.flush_events()  # Actualizar la gráfica
    except KeyboardInterrupt:
        node.get_logger().info('Interrupción por teclado. Cerrando...')
    finally:
        node.destroy_node()
        plt.close('all')  # Cerrar todas las ventanas
        rclpy.shutdown()

if __name__ == '__main__':
    main()
