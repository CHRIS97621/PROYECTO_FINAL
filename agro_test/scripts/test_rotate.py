#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SpinRobot(Node):
    def __init__(self):
        super().__init__('spin_robot')
        self.publisher = self.create_publisher(Twist, '/agro_base_controller/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.spin_in_place)  # Llama cada 0.1 s
        self.start_time = time.time()
        self.angle_turned = 0.0  # Acumulador del ángulo girado
        self.turn_rate = 1.0  # Velocidad angular en rad/s
        self.total_turns = 10  # Número total de vueltas
        self.done = False

    def spin_in_place(self):
        if self.done:
            return

        twist = Twist()
        elapsed_time = time.time() - self.start_time

        if self.angle_turned < self.total_turns * 2 * 3.14159:  # 10 vueltas (10 * 2 * pi radianes)
            twist.angular.z = self.turn_rate  # Velocidad angular fija
            self.angle_turned += self.turn_rate * 0.1  # Incrementa el ángulo acumulado (0.1 es el tiempo de llamada del timer)
        else:
            twist.angular.z = 0.0  # Detén el giro
            self.publisher.publish(twist)
            self.get_logger().info('10 vueltas completadas.')
            self.done = True
            self.destroy_timer(self.timer)
            return

        self.publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = SpinRobot()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
