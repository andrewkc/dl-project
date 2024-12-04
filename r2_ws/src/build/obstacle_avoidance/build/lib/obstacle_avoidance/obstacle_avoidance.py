#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class Turtlebot3Obstacle(Node):
    def __init__(self):
        super().__init__('turtlebot3_obstacle')
        
        # Publisher for /cmd_vel
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber for /scan
        self.sub = self.create_subscription(LaserScan, '/scan', self.callback, 10)
        
        # Twist object for robot movement
        self.move = Twist()

    def callback(self, msg):
        self.get_logger().info(f'Front Range: {msg.ranges[0]}')
        
        if msg.ranges[0] < 0.425:
            # Reverse
            self.move.linear.x = -0.05
            self.move.angular.z = 0
            self.pub.publish(self.move)
            self.get_clock().sleep_for(rclpy.time.Duration(seconds=1))
            
            # Decide which direction to turn
            if msg.ranges[90] <= msg.ranges[270]:
                self.move.linear.x = 0
                self.move.angular.z = -0.5
                self.pub.publish(self.move)
                self.get_clock().sleep_for(rclpy.time.Duration(seconds=1.0))
                self.move.angular.z = 0
                self.pub.publish(self.move)
                self.get_logger().info(" -> right")
            else:
                self.move.linear.x = 0
                self.move.angular.z = 0.5
                self.pub.publish(self.move)
                self.get_clock().sleep_for(rclpy.time.Duration(seconds=1.0))
                self.move.angular.z = 0
                self.pub.publish(self.move)
                self.get_logger().info(" -> left")
            
            # Move forward to avoid the obstacle
            self.move.linear.x = 0.05
            self.move.angular.z = 0
            self.pub.publish(self.move)
            
            while msg.ranges[0] < 0.2:
                self.get_clock().sleep_for(rclpy.time.Duration(seconds=0.1))
                self.pub.publish(self.move)
        else:
            self.move.linear.x = 0.05
            self.move.angular.z = 0
            self.pub.publish(self.move)

def main(args=None):
    rclpy.init(args=args)
    
    # Create the node
    node = Turtlebot3Obstacle()
    
    try:
        rclpy.spin(node)  # Keep the node running
    except KeyboardInterrupt:
        pass
    
    # Clean up and shut down
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
