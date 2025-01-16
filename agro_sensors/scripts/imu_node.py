#!/usr/bin/env python3
# coding: UTF-8
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import tf_transformations as tf
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

# Almacenamiento de la data cruda
ACCData = [0.0] * 8             # 8 bytes que representan la lectura en los ejes x,y,z del acelerómetro
GYROData = [0.0] * 8            # 8 bytes que representan la lectura en los ejes x,y,z del giroscopio
AngleData = [0.0] * 8           # 8 bytes que representan la lectura en los ángulos de ejes
FrameState = 0                  # Estado de la lectura de datos
Bytenum = 0                     # Contador de bytes leídos
CheckSum = 0                    # Bit de verificación de datos recibidos


class IMUNode(Node):

    def __init__(self):
        super().__init__('imu_node')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 9600)
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.ser = serial.Serial(port, baud, timeout=0.005)
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.get_logger().info("Serial is Opened: %s" % self.ser.is_open)

        # Inicializa las variables
        self.acc = None
        self.gyro = None
        self.angle = None

        # Publicador de IMU
        self.imu_publisher = self.create_publisher(Imu, 'imu', 10)

        # Añadido: Broadcaster para publicar TF
        self.tf_broadcaster = TransformBroadcaster(self)

    def timer_callback(self):
        datahex = self.ser.read(33)
        self.DueData(datahex)

        # Publicar datos IMU si están disponibles
        if self.acc is not None and self.gyro is not None and self.angle is not None:
            quaternion = self.compute_quaternion_from_angle(self.angle)
            self.publish_imu_data(self.acc, self.gyro, quaternion)
            self.publish_tf(quaternion)

            # Imprime los valores en la consola
            print(
                f"acc:      {self.acc[0]:10.3f}  {self.acc[1]:10.3f}  {self.acc[2]:10.3f}\n"
                f"gyro:     {self.gyro[0]:10.3f}  {self.gyro[1]:10.3f}  {self.gyro[2]:10.3f}\n"
                f"angle:    {self.angle[0]:10.3f}  {self.angle[1]:10.3f}  {self.angle[2]:10.3f}\n"
                f"quaternion: {quaternion[0]:10.3f}  {quaternion[1]:10.3f}  {quaternion[2]:10.3f}  {quaternion[3]:10.3f}\n"
            )

    def compute_quaternion_from_angle(self, angle):
        # Convierte los ángulos de Euler (roll, pitch, yaw) a un cuaternión
        quaternion = tf.quaternion_from_euler(
            math.radians(angle[0]), math.radians(angle[1]), math.radians(angle[2])
        )
        return quaternion

    def publish_imu_data(self, acc, gyro, quaternion):
        imu_msg = Imu()

        # Aceleración lineal (m/s^2)
        imu_msg.linear_acceleration.x = acc[0]
        imu_msg.linear_acceleration.y = acc[1]
        imu_msg.linear_acceleration.z = acc[2]

        # Velocidad angular (rad/s)
        imu_msg.angular_velocity.x = gyro[0] * 2 * math.pi/360.0
        imu_msg.angular_velocity.y = gyro[1]* 2 * math.pi/360.0
        imu_msg.angular_velocity.z = gyro[2]* 2 * math.pi/360.0

        # Orientación en cuaterniones
        imu_msg.orientation.x = quaternion[0]
        imu_msg.orientation.y = quaternion[1]
        imu_msg.orientation.z = quaternion[2]
        imu_msg.orientation.w = quaternion[3]

        # Header: incluye el frame_id
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"  # Correcto: el IMU está en base_link

        # Publicar el mensaje
        self.imu_publisher.publish(imu_msg)

    def publish_tf(self, quaternion):
        # Publicar la transformación desde 'base_link' al mundo
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_footprint'  # Puedes usar 'map' si es un frame absoluto
        t.child_frame_id = 'imu_link'

        # Posición y orientación
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]

        # Publicar la transformación
        self.tf_broadcaster.sendTransform(t)

    def DueData(self, inputdata):
        global FrameState, Bytenum, CheckSum
        for data in inputdata:  # Recorre los datos de entrada
            if FrameState == 0:  # Estado inicial
                if data == 0x55 and Bytenum == 0:  # Primer byte 0x55
                    CheckSum = data
                    Bytenum = 1
                    continue
                elif data == 0x51 and Bytenum == 1:  # Identificación de la trama de acelerómetro
                    CheckSum += data
                    FrameState = 1
                    Bytenum = 2
                elif data == 0x52 and Bytenum == 1:  # Identificación de la trama de giroscopio
                    CheckSum += data
                    FrameState = 2
                    Bytenum = 2
                elif data == 0x53 and Bytenum == 1:  # Identificación de la trama de ángulo
                    CheckSum += data
                    FrameState = 3
                    Bytenum = 2
            elif FrameState == 1:  # Acelerómetro
                if Bytenum < 10:  # Lee 8 bytes de datos
                    ACCData[Bytenum - 2] = data
                    CheckSum += data
                    Bytenum += 1
                else:
                    if data == (CheckSum & 0xff):  # Verifica el bit de comprobación
                        self.acc = self.get_acc(ACCData)
                    self.reset_frame()
            elif FrameState == 2:  # Giroscopio
                if Bytenum < 10:
                    GYROData[Bytenum - 2] = data
                    CheckSum += data
                    Bytenum += 1
                else:
                    if data == (CheckSum & 0xff):
                        self.gyro = self.get_gyro(GYROData)
                    self.reset_frame()
            elif FrameState == 3:  # Ángulo
                if Bytenum < 10:
                    AngleData[Bytenum - 2] = data
                    CheckSum += data
                    Bytenum += 1
                else:
                    if data == (CheckSum & 0xff):
                        self.angle = self.get_angle(AngleData)
                    self.reset_frame()

    def reset_frame(self):
        global FrameState, Bytenum, CheckSum
        FrameState = 0
        Bytenum = 0
        CheckSum = 0

    def get_acc(self, datahex):
        axl = datahex[0]
        axh = datahex[1]
        ayl = datahex[2]
        ayh = datahex[3]
        azl = datahex[4]
        azh = datahex[5]
        k_acc = 16.0
        acc_x = (axh << 8 | axl) / 32768.0 * k_acc
        acc_y = (ayh << 8 | ayl) / 32768.0 * k_acc
        acc_z = (azh << 8 | azl) / 32768.0 * k_acc
        if acc_x >= k_acc:
            acc_x -= 2 * k_acc
        if acc_y >= k_acc:
            acc_y -= 2 * k_acc
        if acc_z >= k_acc:
            acc_z -= 2 * k_acc
        return acc_x, acc_y, acc_z

    def get_gyro(self, datahex):
        wxl = datahex[0]
        wxh = datahex[1]
        wyl = datahex[2]
        wyh = datahex[3]
        wzl = datahex[4]
        wzh = datahex[5]
        k_gyro = 2000.0
        gyro_x = (wxh << 8 | wxl) / 32768.0 * k_gyro
        gyro_y = (wyh << 8 | wyl) / 32768.0 * k_gyro
        gyro_z = (wzh << 8 | wzl) / 32768.0 * k_gyro
        if gyro_x >= k_gyro:
            gyro_x -= 2 * k_gyro
        if gyro_y >= k_gyro:
            gyro_y -= 2 * k_gyro
        if gyro_z >= k_gyro:
            gyro_z -= 2 * k_gyro
        return gyro_x, gyro_y, gyro_z

    def get_angle(self, datahex):
        rxl = datahex[0]
        rxh = datahex[1]
        ryl = datahex[2]
        ryh = datahex[3]
        rzl = datahex[4]
        rzh = datahex[5]
        k_angle = 180.0
        angle_x = (rxh << 8 | rxl) / 32768.0 * k_angle
        angle_y = (ryh << 8 | ryl) / 32768.0 * k_angle
        angle_z = (rzh << 8 | rzl) / 32768.0 * k_angle
        if angle_x >= k_angle:
            angle_x -= 2 * k_angle
        if angle_y >= k_angle:
            angle_y -= 2 * k_angle
        if angle_z >= k_angle:
            angle_z -= 2 * k_angle
        #pruebas#
        #alpha = 0.5  # Coeficiente del filtro (0.0: sin suavizar, 1.0: muy suave)
        #angle_z_filtered = alpha * angle_z + (1 - alpha) * self.previous_angle_z
        #self.previous_angle_z = angle_z_filtered
        #print(f"Calculated angles: X: {angle_x}, Y: {angle_y}, Z: {angle_z}")
        return angle_x, angle_y, angle_z
        


def main(args=None):
    rclpy.init(args=args)
    imu_node = IMUNode()
    rclpy.spin(imu_node)
    imu_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
