#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


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
        
        self.velocity = 0.05
        
        self.move.linear.x = self.velocity

    def callback(self, msg):
        self.get_logger().info(f'Front Range: {msg.ranges[0]}')
        ranges_size = len(msg.ranges)
        self.get_logger().info(f'Ranges size: {ranges_size}')

        # Ajustar los índices según el tamaño del arreglo
        right_index = int(ranges_size * 0.25)  # 25% del total, aproximadamente 90°
        left_index = int(ranges_size * 0.75)  # 75% del total, aproximadamente 270°

        if msg.ranges[0] < 0.425:
            # Retroceder
            self.move.linear.x = 0.0

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
