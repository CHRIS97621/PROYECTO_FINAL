#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point,PointStamped, PoseStamped,TransformStamped
from nav_msgs.msg import Path
import tkinter as tk
from tkinter import messagebox
import threading
import tf2_ros
import tf2_geometry_msgs
from rclpy.duration import Duration
from pyproj import Proj
from pyproj.aoi import AreaOfInterest
from pyproj.database import query_utm_crs_info
from pyproj import Transformer

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class PathCollectorNode(Node):
    def __init__(self):
        super().__init__('path_collector_node')

        # Crear instancia del BasicNavigator
        self.navigator = BasicNavigator()

        # Suscriptor al tópico /goal_pose_gps
        self.subscription = self.create_subscription(
            PointStamped,
            '/goal_pose_gps',
            self.goal_pose_callback,
            10
        )

        # Publicador de Transformaciones
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Crear el objeto tf2 para transformaciones
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Define proyecciones: WGS84 (GPS) y UTM
        self.wgs84 = Proj(proj="latlong", datum="WGS84")  # Sistema global GPS
        self.utm_zone = None  # Se calculará dinámicamente

        # Parámetros iniciales
        self.origin_easting = None
        self.origin_northing = None
        self.origin_set = False  # Para definir un origen relativo
        self.get_logger().info("Nodo iniciado: Convirtiendo GPS a UTM")



        # Publicador al tópico /path
        self.publisher = self.create_publisher(Path, '/path', 10)

        # Lista para almacenar los puntos recolectados
        self.points = []
        self.points_from_map = []
        self.collecting = False  # Estado de recolección

        self.get_logger().info('Nodo inicializado y listo para la recolección de puntos.')

    def goal_pose_callback(self, msg):
        """Callback para recolectar puntos desde /goal_pose_gps."""
        if self.collecting:
            self.points.append(msg)
            self.get_logger().info(f'Punto recibido y almacenado: {msg.point}')
            path_msg = Path()
            path_msg.header.frame_id = 'wgs84'
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.poses = [
                self._point_to_pose_stamped(point) for point in self.points
            ]
            self.publisher.publish(path_msg)

            
        longitude = msg.point.x
        latitude = msg.point.y


        transformer = Transformer.from_crs("EPSG:4326", "EPSG:32718", always_xy=True)
        
        # Realizar la transformación
        easting, northing = transformer.transform(longitude, latitude)
        self.get_logger().warn(f"easting  {easting} , northing {northing}")

        # Crear la transformación de UTM a un frame estático
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = "utm"  
        transform_stamped.child_frame_id = "utm_frame" 
        transform_stamped.transform.translation.x = easting
        transform_stamped.transform.translation.y = northing
        transform_stamped.transform.translation.z = 0.0
        transform_stamped.transform.rotation.x = 0.0
        transform_stamped.transform.rotation.y = 0.0
        transform_stamped.transform.rotation.z = 0.0
        transform_stamped.transform.rotation.w = 1.0

        # Publicar la transformación
        self.tf_broadcaster.sendTransform(transform_stamped)

        self.get_logger().info(f"Publicado transformación de 'utm' a 'utm_frame'")

        # Obtener la transformación de 'utm_frame' a 'map' (puede ser un error si no se tiene la transformación)
        try:
            transform = self.tf_buffer.lookup_transform('map', 'utm_frame', rclpy.time.Time())
            self.get_logger().info(f"Transformación de 'utm_frame' a 'map' obtenida")
            goal_nav = PoseStamped() 
            goal_nav.header.stamp =  self.get_clock().now().to_msg()
            goal_nav.header.frame_id  = "map"
            goal_nav.pose.position.x  = transform.transform.translation.x
            goal_nav.pose.position.y = transform.transform.translation.y
            goal_nav.pose.position.z  = transform.transform.translation.z
            goal_nav.pose.orientation.x  = transform.transform.rotation.x
            goal_nav.pose.orientation.y  = transform.transform.rotation.y
            goal_nav.pose.orientation.z  = transform.transform.rotation.z
            goal_nav.pose.orientation.w  = transform.transform.rotation.w
            self.points_from_map.append(goal_nav)


        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"No se pudo obtener la transformación de 'utm' a 'map': {e}")




    def start_collecting(self):
        """Iniciar la recolección de puntos."""
        self.collecting = True
        self.get_logger().info('Recolección de puntos iniciada.')

    def execute_nav(self):
        """Iniciar la recolección de puntos."""
        for point in self.points_from_map:
            goal_pose = PoseStamped()
            goal_pose = point
            self.get_logger().info('Leyendo los puntos.')
            print(self.points_from_map)

        self.navigator.goThroughPoses(self.points_from_map)
        self.points_from_map = []
        while not self.navigator.isTaskComplete():
            ################################################
            #
            # Implement some code here for your application!
            #
            ################################################

            # Do something with the feedback

            feedback = self.navigator.getFeedback()
            i=0
            if feedback and i % 5 == 0:
                print(
                    'Estimated time of arrival: '
                    + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                        / 1e9
                    )
                    + ' seconds.'
                )

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.navigator.cancelTask()
            i = i+1

        # Do something depending on the return code
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

        exit(0)

    def stop_and_save_points(self):
        """Detener la recolección y publicar los puntos en /path."""
        if not self.points:
            self.get_logger().warn('No hay puntos para guardar.')
            return

        self.collecting = False
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.poses = [
            self._point_to_pose_stamped(point) for point in self.points
        ]

        self.publisher.publish(path_msg)
        self.get_logger().info(f'{len(self.points)} puntos guardados y publicados en /path.')
        
        self.get_logger().info(f'{len(self.points_from_map)} puntos guardados con referencia a map.')
        # Limpiar la lista de puntos después de publicar
        self.points = []
        #self.points_from_map = []
    def _point_to_pose_stamped(self, point_msg):
        """Convertir PointStamped a PoseStamped."""
        pose_stamped = PoseStamped()
        pose_stamped.header = point_msg.header
        pose_stamped.pose.position.x = point_msg.point.x
        pose_stamped.pose.position.y = point_msg.point.y
        pose_stamped.pose.position.z = point_msg.point.z
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = 0.0
        pose_stamped.pose.orientation.w = 1.0
        return pose_stamped

class GUI:
    def __init__(self, node):
        self.node = node
        self.root = tk.Tk()
        self.root.title("Path Collector")

        # Botón para iniciar la recolección
        self.collect_button = tk.Button(
            self.root, text="Iniciar Recolección", command=self.start_collecting, width=20
        )

        self.collect_button.pack(pady=10)
        # Botón para guardar puntos
        self.save_button = tk.Button(
            self.root, text="Guardar Puntos", command=self.save_points, width=20
        )
        self.save_button.pack(pady=10)
        # Botón para guardar puntos
        self.execute_button = tk.Button(
            self.root, text="Ejecutar plan", command=self.execute_points, width=20
        )
        self.execute_button.pack(pady=10)

    def start_collecting(self):
        self.node.start_collecting()
        messagebox.showinfo("Info", "Recolección de puntos iniciada.")
    def execute_points(self):
        self.node.execute_nav()
        messagebox.showinfo("Info", "Iniciación ejecución del plan.")

    def save_points(self):
        self.node.stop_and_save_points()
        messagebox.showinfo("Info", "Puntos guardados y publicados en /path.")

    def run(self):
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    node = PathCollectorNode()

    # Crear un hilo para rclpy.spin()
    def spin_node():
        rclpy.spin(node)

    spin_thread = threading.Thread(target=spin_node)
    spin_thread.start()

    gui = GUI(node)

    try:
        gui.run()  # Esto se ejecuta en el hilo principal
    except KeyboardInterrupt:
        pass
    finally:
        # Apagar ROS 2 correctamente
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join()


if __name__ == '__main__':
    main()
