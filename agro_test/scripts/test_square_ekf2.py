#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import math
import time

class SquareMover(Node):
    def __init__(self):
        super().__init__('square_mover')
        # Publicador de velocidad
        self.publisher = self.create_publisher(TwistStamped, '/agro_base_controller/cmd_vel', 10)
        
        # Suscripción a la odometría global
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odometry/global',  # <--- Se cambió de '/odometry/local' a '/odometry/global'
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
        self.max_lin_vel = 0.5    # m/s
        self.max_ang_vel = 1.5    # rad/s

        # Máquina de estados para cada lado del cuadrado:
        # - 2 pasos por lado: moverse 1 m, girar 90°
        # - Se repite 4 veces (4 lados)
        self.side_count = 0       # Cuántos lados llevamos completados (0..3)
        self.phase = 0            # 0 = traslación, 1 = giro
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.initial_yaw = 0.0
        self.target_yaw = 0.0

        # Distancia objetivo para cada lado
        self.dist_goal = 1.0      # 1 metro

        self.get_logger().info("SquareMover node initialized. Waiting for odometry messages...")

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

        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = 'base_link'

        # Verificamos si ya completamos 4 lados
        if self.side_count >= 4:
            # Movimiento completado
            self.get_logger().info("Movimiento completado. Deteniendo el robot.")
            twist_stamped.twist.linear.x = 0.0
            twist_stamped.twist.angular.z = 0.0
            self.publisher.publish(twist_stamped)
            # Destruimos timer y nodo
            self.destroy_timer(self.timer)
            self.destroy_node()
            return
        
        # Si estamos en la fase de traslación
        if self.phase == 0:
            self.move_forward(dt, twist_stamped)
        # Si estamos en la fase de giro
        else:
            self.turn_90_degrees(dt, twist_stamped)

        # Publicamos el comando final
        self.publisher.publish(twist_stamped)

    def move_forward(self, dt, twist_stamped):
        """
        Fase de traslación: recorrer 1m desde la posición actual
        utilizando un control PI para evitar frenadas bruscas.
        """
        # Si es la primera iteración en esta fase, guardamos la posición inicial
        if dt > 0 and self.integral_dist == 0.0 and self.integral_yaw == 0.0:
            self.initial_x = self.current_x
            self.initial_y = self.current_y
            self.initial_yaw = self.current_yaw
            self.get_logger().info(f"Iniciando traslación del lado {self.side_count+1}...")

        # 1) Calcular la distancia recorrida desde la posición inicial
        dx = self.current_x - self.initial_x
        dy = self.current_y - self.initial_y
        traveled_dist = math.sqrt(dx*dx + dy*dy)

        # 2) Error de distancia (queremos 1 m)
        error_dist = self.dist_goal - traveled_dist

        # 3) Control PI en distancia
        self.integral_dist += error_dist * dt
        linear_speed = self.kp_dist*error_dist + self.ki_dist*self.integral_dist

        # 4) Limitar velocidad lineal
        linear_speed = max(min(linear_speed, self.max_lin_vel), -self.max_lin_vel)

        # 5) Mantener yaw con un pequeño control en orientación
        error_yaw = self.normalize_angle(self.initial_yaw - self.current_yaw)
        self.integral_yaw += error_yaw * dt
        angular_speed = self.kp_yaw*error_yaw + self.ki_yaw*self.integral_yaw
        angular_speed = max(min(angular_speed, self.max_ang_vel), -self.max_ang_vel)

        # 6) Si ya recorrimos 1 m (o un poco menos/mas con umbral)
        if error_dist < 0.05:  
            # Reseteamos integrales para la siguiente fase
            self.integral_dist = 0.0
            self.integral_yaw = 0.0

            # Pasamos a la fase de giro
            self.phase = 1
            self.get_logger().info(f"Traslación finalizada. Iniciando giro.")
            return
        else:
            # Aplicamos velocidades
            twist_stamped.twist.linear.x = linear_speed
            twist_stamped.twist.angular.z = angular_speed

    def turn_90_degrees(self, dt, twist_stamped):
        """
        Fase de giro: girar 90° (π/2 rad) desde la orientación inicial de este lado.
        Usamos control PI en yaw.
        """
        # Si es la primera iteración en esta fase, guardamos la orientación inicial y definimos yaw objetivo
        if dt > 0 and self.integral_dist == 0.0 and self.integral_yaw == 0.0:
            # La initial_yaw se guardó al final de la fase anterior, la reutilizamos
            self.target_yaw = self.normalize_angle(self.initial_yaw + math.pi/2)
            self.get_logger().info(f"Girando 90 grados...")

        # 1) Calcular error de yaw
        error_yaw = self.target_yaw - self.current_yaw
        error_yaw = self.normalize_angle(error_yaw)

        # 2) Control PI en yaw
        self.integral_yaw += error_yaw * dt
        angular_speed = self.kp_yaw*error_yaw + self.ki_yaw*self.integral_yaw

        # 3) Limitamos velocidad angular
        angular_speed = max(min(angular_speed, self.max_ang_vel), -self.max_ang_vel)

        # 4) Condición para terminar el giro
        if abs(error_yaw) < 0.05:  # Umbral de ~3 grados
            # Giro completado
            self.get_logger().info(f"Giro completado.")
            # Pasamos a la siguiente arista del cuadrado
            self.side_count += 1
            # Reseteamos integrales para la siguiente fase
            self.integral_dist = 0.0
            self.integral_yaw = 0.0
            # Cambiamos a fase de traslación nuevamente
            self.phase = 0
            return
        else:
            # Aplicamos velocidades
            twist_stamped.twist.linear.x = 0.0
            twist_stamped.twist.angular.z = angular_speed

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
    node = SquareMover()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
