#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import os
import sys

class CacaoDetectorNode(Node):
    def __init__(self):
        super().__init__('cacao_detector_node_v8')

        # Suscribirse al tópico de imágenes "camera_frame"
        self.subscription = self.create_subscription(
            Image,
            'camera',
            self.camera_callback,
            10
        )

        # Inicializar CvBridge para convertir mensajes ROS a OpenCV
        self.br = CvBridge()

        # Añadir el entorno pipx al sys.path
        pipx_venv_path = "/home/christopher/.local/share/pipx/venvs/ultralytics/lib/python3.12/site-packages"
        sys.path.append(pipx_venv_path)

        # Cargar el modelo YOLOv8
        self.model = self.load_yolov8_model()

        # Publicador para detecciones
        self.publisher_ = self.create_publisher(String, '/cacao_detection', 10)

        # Directorio donde se guardarán las imágenes capturadas
        self.save_dir = '/home/christopher/cacao_capturas2'
        os.makedirs(self.save_dir, exist_ok=True)

        # Control de tiempo para guardar una imagen cada segundo
        self.last_capture_time = time.time()

    def load_yolov8_model(self):
        try:
            # Importar torch y ultralytics del entorno pipx
            import torch                  # type: ignore
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
            print("data recibida")
            frame = self.br.imgmsg_to_cv2(msg, 'bgr8')

        except Exception as e:
            self.get_logger().error(f"Error al convertir la imagen: {e}")
            return

        # Realizar detección con YOLOv8
        if self.model is not None:
            results = self.model(frame)

            # Acceder correctamente a las coordenadas de los cuadros
            detections = results[0].boxes.xyxy.cpu().numpy()
            classes = results[0].boxes.cls.cpu().numpy()  # Las clases detectadas

            # Obtener los nombres de las clases del modelo
            class_names = self.model.model.names

            # Dibujar rectángulos alrededor de las detecciones
            for i, box in enumerate(detections):
                x_min, y_min, x_max, y_max = box[:4]  # Solo las coordenadas de la caja
                cls_id = int(classes[i])  # ID de la clase detectada
                label = class_names[cls_id]  # Obtener el nombre de la clase

                # Invertimos las etiquetas debido al entrenamiento
                if label == 'cacao sano':
                    label = 'cacao enfermo'
                elif label == 'cacao enfermo':
                    label = 'cacao sano'

                # Colores para las etiquetas
                color = (0, 255, 0) if label == 'cacao sano' else (0, 0, 255)  # Verde para sano, rojo para enfermo

                cv2.rectangle(frame, (int(x_min), int(y_min)), (int(x_max), int(y_max)), color, 2)
                cv2.putText(frame, label, (int(x_min), int(y_min) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

            # Publicar detección
            detection_msg = String()
            detection_msg.data = 'Cacao detectado'
            self.publisher_.publish(detection_msg)
            self.get_logger().info(f'Detección publicada: {detection_msg.data}')

            # Guardar una imagen cada 2 segundos
            current_time = time.time()
            if current_time - self.last_capture_time >= 2.0:
                timestamp = time.strftime("%Y%m%d-%H%M%S")
                image_path = os.path.join(self.save_dir, f"captura_{timestamp}.jpg")
                cv2.imwrite(image_path, frame)
                self.get_logger().info(f'Imagen guardada: {image_path}')
                self.last_capture_time = current_time

            # Mostrar el frame con las detecciones
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
