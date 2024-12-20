#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TransformStamped, PoseStamped, Quaternion, PointStamped
from nav_msgs.msg import Odometry
from pyproj import Proj
from pyproj.aoi import AreaOfInterest
from pyproj.database import query_utm_crs_info
from pyproj import Transformer
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Point


class GPSToUTMNode(Node):
    def __init__(self):
        super().__init__('gps_to_utm_node')

  

        # Suscriptor goal_poal_gps
        self.subscription = self.create_subscription(
            PointStamped,
            'goal_pose_gps',  # Tópico de GPS
            self.goal_pose_gps_callback,
            10
        )


        # Publicador de Odometry para el frame UTM
        self.publisher = self.create_publisher(
            PoseStamped,
            'goal_pose',  # Tópico que publica las coordenadas UTM
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




    def goal_pose_gps_callback(self, msg):
        """
        Callback para procesar los datos de GPS (NavSatFix) y convertirlos a UTM.
        """

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
            self.publisher.publish(goal_nav)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"No se pudo obtener la transformación de 'utm' a 'map': {e}")


def main(args=None):
    rclpy.init(args=args)
    node = GPSToUTMNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
