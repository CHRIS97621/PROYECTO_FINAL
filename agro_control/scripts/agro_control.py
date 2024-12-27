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
from agro_interfaces.msg import ControlStatus

class PointSubscriber(Node):
    def __init__(self):
        super().__init__('point_subscriber')
        self.subscription = self.create_subscription(ControlStatus, 'control_info', self.listener_callback, 10)
        self.time = []
        self.motor1_data = {
            'velocidad': [],
            'control': [],
            'error': [],
            'setpoint': [],
            'velocidad_filtered': []
        }
        self.motor2_data = {
            'velocidad': [],
            'control': [],
            'error': [],
            'setpoint': [],
            'velocidad_filtered': []
        }
        self.start_time = self.get_clock().now().nanoseconds / 1e9

    def listener_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time
        self.time.append(current_time)

        self.motor1_data['velocidad'].append(msg.current_speed.motor1)
        self.motor1_data['control'].append(msg.current_control.motor1)
        self.motor1_data['error'].append(msg.current_error.motor1)
        self.motor1_data['setpoint'].append(msg.setpoint.motor1)
        self.motor1_data['velocidad_filtered'].append(msg.current_speed_filtered.motor1)

        self.motor2_data['velocidad'].append(msg.current_speed.motor2)
        self.motor2_data['control'].append(msg.current_control.motor2)
        self.motor2_data['error'].append(msg.current_error.motor2)
        self.motor2_data['setpoint'].append(msg.setpoint.motor2)
        self.motor2_data['velocidad_filtered'].append(msg.current_speed_filtered.motor2)

        self.get_logger().info(f'Recibido: Velocidad1={msg.current_speed.motor1}, Control1={msg.current_control.motor1}, Error1={msg.current_error.motor1}')
        self.get_logger().info(f'Recibido: Velocidad2={msg.current_speed.motor2}, Control2={msg.current_control.motor2}, Error2={msg.current_error.motor2}')


class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.motor_publisher = self.create_publisher(Bool, 'set_switch_control', 10)


    def publish_motor_status(self, status):
        msg = Bool()
        msg.data = status
        self.motor_publisher.publish(msg)
        self.get_logger().info(f'Motor status publicado: {status}')


def update_plot(node, ax, motor_data, canvas):
    if not rclpy.ok():
        return

    ax.clear()

    current_time = node.time[-1] if node.time else 0
    time_range = 5
    filtered_indices = [i for i, t in enumerate(node.time) if current_time - t <= time_range]

    filtered_time = [node.time[i] for i in filtered_indices]
    for key, color, label in zip(['error', 'velocidad_filtered', 'setpoint'],
                                  ['red', 'black', 'green'],
                                  ['Error', 'Velocidad Filtrada', 'Setpoint']):
        ax.plot(filtered_time, [motor_data[key][i] for i in filtered_indices], label=label, color=color)

    ax.set_title('Datos del Motor')
    ax.legend()
    ax.set_xlabel('Tiempo (s)')
    ax.set_ylabel('Valor')
    ax.set_ylim(-40, 40)
    ax.legend()

    canvas.draw()
    canvas.get_tk_widget().after(100, update_plot, node, ax, motor_data, canvas)


def main(args=None):
    rclpy.init(args=args)
    subscriber_node = PointSubscriber()
    motor_node = MotorController()

    root = tk.Tk()
    root.title("Interfaz de Control y Gráficos")

    fig, axes = plt.subplots(2, 2, figsize=(14, 8))
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

    speed_var_motor1 = tk.IntVar(value=0)
    speed_var_motor2 = tk.IntVar(value=0)

    def set_speed_motor1(val):
        motor_node.publish_speed_motor1(speed_var_motor1.get())

    def set_speed_motor2(val):
        motor_node.publish_speed_motor2(speed_var_motor2.get())

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

    canvas_widget.after(100, update_plot, subscriber_node, axes[0][0], subscriber_node.motor1_data, canvas)
    canvas_widget.after(100, update_plot, subscriber_node, axes[0][1], subscriber_node.motor2_data, canvas)

    root.mainloop()
    ros_thread.join()

if __name__ == '__main__':
    main()