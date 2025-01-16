#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

# TF2
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion

# Mensajes y servicios
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from robot_localization.srv import FromLL

def normalize_angle(angle_degs):
    """
    Normaliza un ángulo en grados al rango (-180, 180].
    """
    angle_degs = angle_degs % 360
    angle_degs = (angle_degs + 360) % 360
    if angle_degs > 180:
        angle_degs -= 360
    return angle_degs

class GPSGoalNavigation(Node):
    def __init__(self):
        super().__init__('gps_goal_navigation')

        # Buffer y Listener TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Cliente del servicio "fromLL"
        self.client = self.create_client(FromLL, srv_name='fromLL')
        self.get_logger().info('Esperando al servicio "fromLL"...')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servicio "fromLL" no disponible, reintentando...')
        self.get_logger().info('Servicio "fromLL" disponible.')

        # Variables para destino
        self.req = FromLL.Request()
        self.POINT_READY = 0
        self.IS_READY_FOR_NEXT_POINT = True

        # Suscripción a la odometría global
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/global',
            self.odometry_callback,
            QoSProfile(depth=10)
        )
        self.ODOMETRY_READY = 0

        # Publicador de velocidad
        self.publisher = self.create_publisher(TwistStamped, '/agro_base_controller/cmd_vel', 10)

        # Timer de control (~10 Hz)
        self.timer = self.create_timer(0.1, self.on_timer)

        # Pose actual
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

    def odometry_callback(self, msg):
        pose = msg.pose.pose
        self.x = pose.position.x
        self.y = pose.position.y

        q = (pose.orientation.x,
             pose.orientation.y,
             pose.orientation.z,
             pose.orientation.w)
        roll, pitch, yaw = euler_from_quaternion(q)
        self.yaw = yaw

        self.ODOMETRY_READY = 1

    def on_timer(self):
        """
        En cada ciclo (~10 Hz) calculamos el error en posición (dx, dy) y orientación (delta_th).
        Ajustamos la velocidad lineal y angular para acercarnos al objetivo.
        """
        if self.POINT_READY == 1 and self.ODOMETRY_READY == 1:
            x_goal = self.X_GOAL
            y_goal = self.Y_GOAL

            dx = x_goal - self.x
            dy = y_goal - self.y
            dist = math.sqrt(dx*dx + dy*dy)

            target_angle = math.atan2(dy, dx)
            delta_th = target_angle - self.yaw

            # Normalizar el ángulo en grados para imprimir
            delta_th_deg = math.degrees(delta_th)
            delta_th_deg = normalize_angle(delta_th_deg)
            # Pasarlo de nuevo a radianes normalizados
            delta_th = math.radians(delta_th_deg)

            self.get_logger().info(
                f'Error angular: {delta_th_deg:.2f} [deg], '
                f'Error dist: {dist:.2f} [m]'
            )

            # --- CONTROL DE VELOCIDAD ---
            # Ajusta estos parámetros para reducir giros bruscos y velocidades altas
            K_ang = 0.5       # Ganancia angular reducida (antes era 1.0)
            K_lin = 0.3       # Ganancia lineal más baja (antes era 0.5)
            max_ang_vel = 0.4 # Máx. velocidad angular menor (antes 0.8)
            max_lin_vel = 0.5 # Podrías bajarlo si deseas aún menos velocidad lineal

            # Distancia "efectiva" para el control lineal
            # (Si dist > 1 m, usamos 1.0 para no acelerar mucho)
            dist_cmd = min(dist, 1.0)

            # Control angular (salida = K_ang * error)
            raw_angular = K_ang * delta_th
            # Limitar la velocidad angular
            angular_cmd = max(-max_ang_vel, min(max_ang_vel, raw_angular))

            # --- Evitar avanzar si el robot está muy desorientado ---
            # Si el error angular es grande, reducimos la velocidad lineal
            abs_delta_deg = abs(delta_th_deg)
            if abs_delta_deg > 40:
                # Si estamos muy desalineados, reducimos mucho la lineal
                linear_cmd = 0.0
            elif abs_delta_deg > 20:
                # Si el error es mediano, reducimos la lineal a un valor bajo
                linear_cmd = 0.1
            else:
                # Control lineal normal
                linear_cmd = K_lin * dist_cmd
            
            # Limitar la velocidad lineal a un máximo
            if linear_cmd > max_lin_vel:
                linear_cmd = max_lin_vel

            # --- Construir TwistStamped ---
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_footprint'

            msg.twist.angular.z = angular_cmd
            msg.twist.linear.x  = linear_cmd

            # Si estamos muy cerca de la meta, paramos
            if dist < 0.1:
                self.get_logger().info('Meta alcanzada. Deteniendo...')
                msg.twist.angular.z = 0.0
                msg.twist.linear.x  = 0.0
                self.POINT_READY = 0
                self.IS_READY_FOR_NEXT_POINT = True

            # Publicar TwistStamped
            self.publisher.publish(msg)

    def send_request(self, latitude, longitude):
        """
        Llamamos al servicio "fromLL" para convertir (lat, lon) en coordenadas x, y en el marco del mapa.
        """
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servicio "fromLL" no disponible, reintentando...')
        self.get_logger().info('Servicio "fromLL" disponible.')

        self.req.ll_point.altitude  = 0.0
        self.req.ll_point.latitude  = latitude
        self.req.ll_point.longitude = longitude

        future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        if future.done():
            response = future.result()
            self.X_GOAL = response.map_point.x
            self.Y_GOAL = response.map_point.y
            self.get_logger().info(
                f'Recibida meta: X_GOAL={self.X_GOAL:.2f}, Y_GOAL={self.Y_GOAL:.2f}'
            )
            self.POINT_READY = 1
        else:
            self.get_logger().warn('Fallo al llamar al servicio fromLL')
            self.POINT_READY = 0


def main(args=None):
    rclpy.init(args=args)
    node = GPSGoalNavigation()

    # Lista de waypoints de ejemplo (lat, lon)
    waypoints = [
        (-12.0248971, -77.0474840),
        (-12.0249430, -77.0474311),
        (-12.0248796, -77.0473791),
        (-12.0248363, -77.0474299),
    ]

    for (lat, lon) in waypoints:
        node.get_logger().info(f'Requerir ir a lat={lat}, lon={lon}')
        node.send_request(latitude=lat, longitude=lon)

        # Esperar hasta llegar
        while rclpy.ok() and node.POINT_READY == 1:
            rclpy.spin_once(node, timeout_sec=0.1)

    node.get_logger().info("Se completaron todos los waypoints.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
