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
from geometry_msgs.msg import TwistStamped  # <-- Usaremos TwistStamped
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

        # Buffer y Listener TF (por si se necesitan transformaciones)
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

        # Publicador de velocidad con TwistStamped en "/agro_base_controller/cmd_vel"
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
        if self.POINT_READY == 1 and self.ODOMETRY_READY == 1:
            x_goal = self.X_GOAL
            y_goal = self.Y_GOAL

            dx = x_goal - self.x
            dy = y_goal - self.y
            dist = math.sqrt(dx*dx + dy*dy)

            target_angle = math.atan2(dy, dx)
            delta_th = target_angle - self.yaw

            # Normalizar en grados
            delta_th_deg = math.degrees(delta_th)
            delta_th_deg = normalize_angle(delta_th_deg)
            # Regresar a radianes
            delta_th = math.radians(delta_th_deg)

            self.get_logger().info(
                f'Error angular: {delta_th_deg:.2f} [deg], '
                f'Error dist: {dist:.2f} [m]'
            )

            # Generar mensaje TwistStamped
            msg = TwistStamped()
            # Cabecera (opcional: frame_id = "base_footprint" o similar)
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_footprint'

            # Ganancias
            K_ang = 1.0
            K_lin = 0.5

            # Limitar la velocidad lineal si dist > 1.0
            dist_cmd = dist if dist < 1.0 else 1.0

            # Calcular velocidad angular
            raw_angular = K_ang * delta_th
            # Limitar la velocidad angular a ±1.0 rad/s (opcional)
            max_ang_vel = 0.8
            angular_cmd = max(-max_ang_vel, min(max_ang_vel, raw_angular))

            # Asignar velocidades al TwistStamped
            msg.twist.angular.z = angular_cmd
            msg.twist.linear.x  = K_lin * dist_cmd

            # Si estamos cerca de la meta, paramos
            if dist < 0.1:
                self.get_logger().info('Meta alcanzada. Deteniendo...')
                msg.twist.angular.z = 0.0
                msg.twist.linear.x  = 0.0
                self.POINT_READY = 0
                self.IS_READY_FOR_NEXT_POINT = True

            # Publicar TwistStamped
            self.publisher.publish(msg)

    def send_request(self, latitude, longitude):
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
