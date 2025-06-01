#! /usr/bin/env python3

import rclpy
import rclpy.logging
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

class MyNode(Node):
    
    def __init__(self):
        super().__init__("csj_read_lidar")
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, qos_profile_sensor_data)
    
    def scan_callback(self, scan:LaserScan):
        self.get_logger().info(f"{scan.ranges[-5]}")


def main(args=None):
    rclpy.init(args=args)
    node = MyNode() 
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()