#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SquareMover(Node):
    def __init__(self):
        super().__init__('square_mover')
        self.publisher = self.create_publisher(Twist, '/agro_base_controller/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move_square)  # Llama cada 0.1 s
        self.start_time = time.time()
        self.step = 0  # Control de los pasos

    def move_square(self):
        elapsed_time = time.time() - self.start_time
        twist = Twist()

        if self.step < 4:
            if elapsed_time < 3:  # Mover recto por 3 segundos
                twist.linear.x = 0.5  # Velocidad lineal
            elif elapsed_time < 3.5:  # Girar 90° en 0.5 segundos
                twist.angular.z = 1.57  # Velocidad angular (rad/s para 90°)
            else:  # Reinicia para el siguiente lado
                self.start_time = time.time()
                self.step += 1
        else:
            # Detén el robot al completar el cuadrado
            self.get_logger().info("Movimiento completado.")
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            
            # Cancela el temporizador y destruye el nodo
            self.destroy_timer(self.timer)
            self.destroy_node()

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = SquareMover()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
