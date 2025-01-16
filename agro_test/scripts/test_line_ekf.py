#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import math
import time

class StraightMover(Node):
    def __init__(self):
        super().__init__('straight_mover')
        
        # Publicador de velocidad
        self.publisher = self.create_publisher(TwistStamped, '/agro_base_controller/cmd_vel', 10)
        
        # Suscripción a la odometría global
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odometry/global',  # La ruta de odometría que deseas usar
            self.odometry_callback,
            10
        )

        # Temporizador (frecuencia de control ~10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        # Variables de estado
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # Para medir el tiempo en cada paso
        self.last_time = time.time()

        # Parámetros del control PI para distancia
        self.kp_dist = 1.0
        self.ki_dist = 0.0
        self.integral_dist = 0.0

        # Parámetros del control PI para orientación
        self.kp_yaw = 2.0
        self.ki_yaw = 0.0
        self.integral_yaw = 0.0
        
        # Velocidades máximas para seguridad
        self.max_lin_vel = 0.5   # m/s
        self.max_ang_vel = 1.5   # rad/s

        # Distancia objetivo (2 metros en línea recta)
        self.distance_goal = 2.0

        # Banderas y valores para la lógica del movimiento
        self.movement_started = False
        self.movement_completed = False
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.initial_yaw = 0.0

        self.get_logger().info("StraightMover node initialized. Waiting for odometry messages...")

    def odometry_callback(self, msg):
        """
        Callback para obtener la posición y la orientación desde /odometry/global.
        """
        # Posición actual
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Convertir la orientación (cuaternión) a yaw
        orientation_q = msg.pose.pose.orientation
        roll, pitch, yaw = self.quaternion_to_euler(
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        )
        self.current_yaw = yaw

    def quaternion_to_euler(self, x, y, z, w):
        """
        Convierte cuaterniones a ángulos de Euler (roll, pitch, yaw).
        """
        t0 = +2.0 * (w*x + y*z)
        t1 = +1.0 - 2.0 * (x*x + y*y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w*y - z*x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w*z + x*y)
        t4 = +1.0 - 2.0 * (y*y + z*z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

    def control_loop(self):
        """
        Bucle de control que se ejecuta periódicamente (~10 Hz).
        """
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Creamos el mensaje TwistStamped
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = 'base_link'

        # Si ya completamos el movimiento, detener y salir.
        if self.movement_completed:
            twist_stamped.twist.linear.x = 0.0
            twist_stamped.twist.angular.z = 0.0
            self.publisher.publish(twist_stamped)
            return

        # Si aún no empezamos el movimiento, registramos la posición y yaw inicial
        if not self.movement_started:
            self.movement_started = True
            self.initial_x = self.current_x
            self.initial_y = self.current_y
            self.initial_yaw = self.current_yaw
            self.integral_dist = 0.0
            self.integral_yaw = 0.0
            self.get_logger().info("Iniciando movimiento recto de 2 metros...")

        # 1) Calcular la distancia recorrida
        dx = self.current_x - self.initial_x
        dy = self.current_y - self.initial_y
        traveled_dist = math.sqrt(dx**2 + dy**2)

        # 2) Error de distancia
        error_dist = self.distance_goal - traveled_dist

        # 3) Control PI en distancia
        self.integral_dist += error_dist * dt
        linear_speed = self.kp_dist*error_dist + self.ki_dist*self.integral_dist

        # 4) Limitar velocidad lineal
        linear_speed = max(min(linear_speed, self.max_lin_vel), -self.max_lin_vel)

        # 5) Mantener yaw con un pequeño control en orientación
        #    Queremos conservar la orientación inicial
        error_yaw = self.normalize_angle(self.initial_yaw - self.current_yaw)
        self.integral_yaw += error_yaw * dt
        angular_speed = self.kp_yaw*error_yaw + self.ki_yaw*self.integral_yaw
        
        # Limitar velocidad angular
        angular_speed = max(min(angular_speed, self.max_ang_vel), -self.max_ang_vel)

        # 6) Verificar si ya recorrió los 2 metros (con un pequeño umbral)
        if error_dist < 0.05:
            # Movimiento completado
            self.get_logger().info("Movimiento completado. Deteniendo el robot.")
            self.movement_completed = True
            # Detenemos el robot
            twist_stamped.twist.linear.x = 0.0
            twist_stamped.twist.angular.z = 0.0
        else:
            # Aplicamos velocidades
            twist_stamped.twist.linear.x = linear_speed
            twist_stamped.twist.angular.z = angular_speed

        # Publicamos el comando
        self.publisher.publish(twist_stamped)

    def normalize_angle(self, angle):
        """
        Normaliza un ángulo entre -pi y pi.
        """
        while angle > math.pi:
            angle -= 2.0*math.pi
        while angle < -math.pi:
            angle += 2.0*math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = StraightMover()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
