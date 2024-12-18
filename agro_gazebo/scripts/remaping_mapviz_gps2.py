#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class OdomToPose(Node):
    def __init__(self):
        super().__init__('odom_to_pose_node')

        # Suscriptor al tópico transformado
        self.subscription = self.create_subscription(
            Odometry,
            'odometry/gps',  # Salida del navsat_transform_node
            self.odom_callback,
            10
        )

        # Publicador de PoseStamped para Nav2
        self.publisher = self.create_publisher(
            PoseStamped,
            'goal_pose',
            10
        )

        self.get_logger().info("Nodo iniciado: OdomToPose")

    def odom_callback(self, msg):
        # Crear PoseStamped a partir de Odometry
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.pose = msg.pose.pose  # Copia posición y orientación

        # Publicar en goal_pose
        self.publisher.publish(pose_msg)
        self.get_logger().info(f"PoseStamped publicado: x={pose_msg.pose.position.x}, y={pose_msg.pose.position.y}")

def main(args=None):
    rclpy.init(args=args)
    node = OdomToPose()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
