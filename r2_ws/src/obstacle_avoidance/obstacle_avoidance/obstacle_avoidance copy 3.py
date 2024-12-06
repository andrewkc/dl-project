#!/usr/bin/env python3

import os
import cv2
import base64
from itertools import chain
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from datetime import datetime


def encode_image(image):
    """Convierte una imagen en base64."""
    _, buffer = cv2.imencode(".jpg", image)
    return base64.b64encode(buffer).decode("utf-8")


class Turtlebot3Obstacle(Node):
    def __init__(self):
        super().__init__("turtlebot3_obstacle")

        # Configuración de QoS compatible con el publicador del tópico /scan
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Compatible con BEST_EFFORT
            depth=10,
        )

        # Publisher para /cmd_vel
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Subscriber para /scan con el QoS modificado
        self.sub = self.create_subscription(
            LaserScan, "/scan", self.callback, qos_profile
        )

        # Objeto Twist para el movimiento del robot
        self.move = Twist()
        self.velocity = 0.15

        # -1 for left, 1 for right
        self.rotating_direction = 0

        # Inicialización de la cámara
        self.cap = cv2.VideoCapture(0)  # Usa la cámara predeterminada

        # Directorio para guardar las imágenes
        self.image_directory = "captured_images"
        if not os.path.exists(self.image_directory):
            os.makedirs(self.image_directory)

    def callback(self, msg):
        self.get_logger().info(f"Front Range: {msg.ranges[0]}")
        ranges_size = len(msg.ranges)

        offset = 1 / 8
        right_index = int(ranges_size * offset)
        left_index = int(ranges_size * (1 - offset))

        ask_distance = 0.22
        any_in_front = any(
            msg.ranges[i] < ask_distance
            for i in chain(
                range(0, right_index + 1),
                range(left_index, len(msg.ranges)),
            )
        )

        if any_in_front:
            self.move.linear.x = 0.0
            self.move.angular.z = 0.0
            self.pub.publish(self.move)

            if self.rotating_direction == -1:
                self.turn(-0.5, "left")
            elif self.rotating_direction == 1:
                self.turn(0.5, "right")
            elif msg.ranges[right_index] <= msg.ranges[left_index]:
                self.rotating_direction = -1
                self.turn(-0.5, "left")
            else:
                self.rotating_direction = 1
                self.turn(0.5, "right")
        else:
            self.rotating_direction = 0
            self.move.linear.x = self.velocity
            self.move.angular.z = 0.0
            self.pub.publish(self.move)

        # Capturar imagen de la cámara
        ret, frame = self.cap.read()
        if ret:
            # Guardar la imagen en un archivo
            self.save_image(frame)

    def save_image(self, frame):
        """Guarda una imagen en un archivo en el directorio especificado."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        file_path = os.path.join(self.image_directory, f"image_{timestamp}.jpg")
        cv2.imwrite(file_path, frame)
        self.get_logger().info(f"Image saved: {file_path}")

    def turn(self, angular_speed, direction):
        self.move.linear.x = 0.0
        self.move.angular.z = angular_speed
        self.pub.publish(self.move)

        self.get_clock().sleep_for(rclpy.time.Duration(seconds=1.0))

        self.move.angular.z = 0.0
        self.pub.publish(self.move)
        self.get_logger().info(f" -> {direction}")

    def destroy_node(self):
        super().destroy_node()
        self.cap.release()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)

    # Crear el nodo
    node = Turtlebot3Obstacle()

    try:
        rclpy.spin(node)  # Mantener el nodo en ejecución
    except KeyboardInterrupt:
        pass

    # Limpiar y cerrar
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()