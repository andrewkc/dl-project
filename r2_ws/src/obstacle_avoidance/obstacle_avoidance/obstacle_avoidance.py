#!/usr/bin/env python3

from typing import Union
from pydantic import BaseModel

import sys
import openai
import pprint
import base64

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class Turn(BaseModel):
    direction: str
    amount: float

class Forward(BaseModel):
    speed: float

class TurtleInstruction(BaseModel):
    instruction: Union[Turn, Forward]
    explanation: str


def encode_image(image_path):
    with open(image_path, "rb") as image_file:
        return base64.b64encode(image_file.read()).decode('utf-8')


def ask_for_instruction():
    base64_image = 0 # TODO

    client = openai.OpenAI()

    img_type = "image/jpeg"

    # Abre el archivo de la imagen y envíalo a través de la API
    response = client.beta.chat.completions.parse(
        model="gpt-4o",
        # model="gpt-4o-mini",
        messages=[
            {
                "role": "system",
                "content": "You're in charge of directing the path of a wheeled robot. Based on an image taken from the frontal camera, you're to indicate whether the next immediate action should be to go forward, rotate left or right, or go backwards",
            },
            {
                "role": "user",
                "content": [
                    {
                        "type": "text",
                        "text": "This is the image from front view of the robot's current position. I want to know what should be my next immediate action, whether to go forward, rotate left or right, or go back if needed.",
                    },
                    {
                        "type": "image_url",
                        "image_url": {"url": f"data:{img_type};base64,{base64_image}"},
                    },
                ],
            },
        ],
        response_format = TurtleInstruction,
    )

    # Imprime la respuesta
    # print(response["choices"][0]["message"]["content"])
    # pprint.pprint(response)

    instruction = response.choices[0].message.parsed
    pprint.pprint(instruction)


class Turtlebot3Obstacle(Node):
    def __init__(self):
        super().__init__('turtlebot3_obstacle')

        # Configuración de QoS compatible con el publicador del tópico /scan
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Compatible con BEST_EFFORT
            depth=10
        )

        # Publisher para /cmd_vel
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber para /scan con el QoS modificado
        self.sub = self.create_subscription(LaserScan, '/scan', self.callback, qos_profile)

        # Objeto Twist para el movimiento del robot
        self.move = Twist()

    def callback(self, msg):
        self.get_logger().info(f'Front Range: {msg.ranges[0]}')
        ranges_size = len(msg.ranges)

        self.get_logger().info(f'Ranges size: {ranges_size}')

        offset = 1/8

        # Ajustar los índices según el tamaño del arreglo
        right_index = int(ranges_size * offset)  # 25% del total, aproximadamente 90°
        left_index = int(ranges_size * (1 - offset))  # 75% del total, aproximadamente 270°

        ask_distance = 0.22

        if msg.ranges[0] < ask_distance:
            self.move.linear.x = 0.0
            self.move.angular.z = 0.0
            self.pub.publish(self.move)

            if msg.ranges[right_index] <= msg.ranges[left_index]:
                self.move.linear.x = 0.0
                self.move.angular.z = -0.5
                self.pub.publish(self.move)

                self.get_clock().sleep_for(rclpy.time.Duration(seconds=1.0))

                self.move.angular.z = 0.0
                self.pub.publish(self.move)
                self.get_logger().info(" -> right")
            else:
                self.move.linear.x = 0.0
                self.move.angular.z = 0.5
                self.pub.publish(self.move)

                self.get_clock().sleep_for(rclpy.time.Duration(seconds=1.0))

                self.move.angular.z = 0.0
                self.pub.publish(self.move)
                self.get_logger().info(" -> left")

            # Avanzar para evitar el obstáculo
            self.move.linear.x = 0.22
            self.move.angular.z = 0.0
            self.pub.publish(self.move)
        else:
            self.move.linear.x = 0.2
            self.move.angular.z = 0.0
            self.pub.publish(self.move)


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


if __name__ == '__main__':
    main()
