#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import json
import sys

class CacaoDetectorNode(Node):
    def __init__(self):
        super().__init__('cacao_detector_node')

        # Suscribirse al tópico de imágenes
        self.subscription = self.create_subscription(
            Image,
            'camera',
            self.camera_callback,
            10
        )

        # Inicializar CvBridge para convertir mensajes ROS a OpenCV
        self.br = CvBridge()

        # Publicador para detecciones
        self.detection_publisher = self.create_publisher(String, '/cacao_detections', 10)

        # Añadir el entorno pipx al sys.path
        pipx_venv_path = "/home/christopher/.local/share/pipx/venvs/ultralytics/lib/python3.12/site-packages"
        sys.path.append(pipx_venv_path)

        # Cargar el modelo YOLOv8
        self.model = self.load_yolov8_model()

    def load_yolov8_model(self):
        try:
            # Importar torch y ultralytics
            import torch  # type: ignore
            from ultralytics import YOLO  # type: ignore

            # Cargar el modelo YOLOv8
            model = YOLO('/home/christopher/yolo_results/train3/weights/best.pt')
            return model
        except Exception as e:
            self.get_logger().error(f"Error al cargar el modelo YOLOv8: {e}")
            return None

    def camera_callback(self, msg):
        try:
            # Convertir el mensaje de imagen de ROS a OpenCV
            self.get_logger().info("Imagen recibida")
            frame = self.br.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Error al convertir la imagen: {e}")
            return

        # Realizar detección con YOLOv8
        if self.model is not None:
            results = self.model(frame)

            # Inicializar contadores de cacaos sanos y enfermos
            num_sanos = 0
            num_enfermos = 0

            # Obtener las detecciones
            detections = results[0].boxes.xyxy.cpu().numpy()
            classes = results[0].boxes.cls.cpu().numpy()  # Clases detectadas
            class_names = self.model.model.names  # Nombres de las clases

            for i, box in enumerate(detections):
                cls_id = int(classes[i])  # ID de la clase detectada
                label = class_names[cls_id]  # Obtener el nombre de la clase

                # Invertir etiquetas según tu lógica
                if label == 'cacao sano':
                    label = 'cacao enfermo'
                    num_enfermos += 1
                elif label == 'cacao enfermo':
                    label = 'cacao sano'
                    num_sanos += 1

            # Crear un mensaje JSON con los resultados
            detection_msg = {
                "cacao_sano": num_sanos,
                "cacao_enfermo": num_enfermos
            }

            # Publicar detecciones en formato JSON
            self.detection_publisher.publish(String(data=json.dumps(detection_msg)))
            self.get_logger().info(f"Detecciones publicadas: {detection_msg}")

            # Visualizar las detecciones en el marco de la cámara
            for i, box in enumerate(detections):
                x_min, y_min, x_max, y_max = box[:4]

                # Colores para los rectángulos (verde: sano, rojo: enfermo)
                color = (0, 255, 0) if label == 'cacao sano' else (0, 0, 255)
                cv2.rectangle(frame, (int(x_min), int(y_min)), (int(x_max), int(y_max)), color, 2)
                cv2.putText(frame, label, (int(x_min), int(y_min) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

            cv2.imshow('Cacao Detection', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.destroy_node()

    def destroy_node(self):
        # Cierra todas las ventanas de OpenCV al finalizar
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CacaoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
