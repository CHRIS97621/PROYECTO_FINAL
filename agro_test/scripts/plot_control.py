#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Float32
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from tkinter import ttk
import threading
import signal

class PointSubscriber(Node):
    def __init__(self):
        super().__init__('point_subscriber')
        self.subscription = self.create_subscription(Point, 'control_info', self.listener_callback, 10)
        self.time = []
        self.velocidad = []
        self.control = []
        self.error = []
        self.start_time = self.get_clock().now().nanoseconds / 1e9

    def listener_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time
        self.time.append(current_time)
        self.velocidad.append(msg.x)
        self.control.append(msg.y)
        self.error.append(msg.z)
        self.get_logger().info(f'Recibido: Velocidad={msg.x}, Control={msg.y}, Error={msg.z}')


class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.motor_publisher = self.create_publisher(Bool, 'set_motor_state', 10)
        self.speed_publisher = self.create_publisher(Float32, 'vel_motor1', 10)

    def publish_motor_status(self, status):
        msg = Bool()
        msg.data = status
        self.motor_publisher.publish(msg)
        self.get_logger().info(f'Motor status publicado: {status}')

    def publish_speed(self, speed):
        msg = Float32()
        msg.data = speed
        self.speed_publisher.publish(msg)
        self.get_logger().info(f'Velocidad publicada: {speed}')


def update_plot(node, ax1, ax2, canvas):
    if not rclpy.ok():
        return

    ax1.clear()
    ax2.clear()

    ax1.set_title('Velocidad y Error')
    ax1.plot(node.time, node.velocidad, label='Velocidad', color='blue')
    ax1.plot(node.time, node.error, label='Error', color='red')
    ax1.set_ylabel('Velocidad / Error')
    ax1.set_ylim(-20, 20)
    ax1.legend()

    ax2.set_title('Señal de Control (y)')
    ax2.plot(node.time, node.control, label='Control', color='green')
    ax2.set_ylabel('Control')
    ax2.set_xlabel('Tiempo (s)')
    ax2.set_ylim(-100, 100)
    canvas.draw()
    canvas.get_tk_widget().after(100, update_plot, node, ax1, ax2, canvas)


def main(args=None):
    rclpy.init(args=args)
    subscriber_node = PointSubscriber()
    motor_node = MotorController()

    root = tk.Tk()
    root.title("Interfaz de Control y Gráficos")

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8))
    canvas = FigureCanvasTkAgg(fig, master=root)
    canvas_widget = canvas.get_tk_widget()
    canvas_widget.grid(row=0, column=0, columnspan=2)

    motor_status = tk.BooleanVar(value=False)

    def toggle_motor():
        motor_status.set(not motor_status.get())
        motor_node.publish_motor_status(motor_status.get())
        motor_button.config(text="Desactivar Motor" if motor_status.get() else "Activar Motor")

    motor_button = ttk.Button(root, text="Activar Motor", command=toggle_motor)
    motor_button.grid(row=1, column=0, padx=10, pady=10)

    speed_var = tk.IntVar(value=0)

    def set_speed(val):
        motor_node.publish_speed(speed_var.get())

    speed_slider = ttk.Scale(root, from_=0, to=100, orient='horizontal', variable=speed_var, command=set_speed)
    speed_slider.grid(row=1, column=1, padx=10, pady=10)
    speed_label = ttk.Label(root, text="Velocidad de la Rueda")
    speed_label.grid(row=2, column=1)

    def on_closing():
        root.quit()
        subscriber_node.destroy_node()
        motor_node.destroy_node()
        rclpy.shutdown()

    root.protocol("WM_DELETE_WINDOW", on_closing)

    def handle_signal(sig, frame):
        on_closing()

    signal.signal(signal.SIGINT, handle_signal)

    ros_thread = threading.Thread(target=lambda: rclpy.spin(subscriber_node), daemon=True)
    ros_thread.start()

    canvas_widget.after(100, update_plot, subscriber_node, ax1, ax2, canvas)

    root.mainloop()
    ros_thread.join()

if __name__ == '__main__':
    main()
