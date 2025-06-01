#! /usr/bin/env python3

import rclpy
import rclpy.logging
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist


class MyNode(Node):
    
    def __init__(self):
        super().__init__("csj_collision_avoid")
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, qos_profile_sensor_data)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
    
    def scan_callback(self, scan:LaserScan):

        msg_vel = Twist()
        range_0 = scan.ranges[0]
        range_5 = scan.ranges[5]
        range_neg_5 = scan.ranges[-5]
        
        range_avg = (range_0 + range_5 + range_neg_5) / 3
        self.get_logger().info(f"range_avg: {range_avg}")
        
        
        if range_avg < 1:
            msg_vel.linear.x = 0.0
            msg_vel.angular.z = 0.3
            self.get_logger().info("Obstacle detected")
        else:
            msg_vel.linear.x = 0.12
            msg_vel.angular.z = 0.0
            self.get_logger().info("No obstacle, move forward!")

        self.cmd_vel_pub.publish(msg_vel)


def main(args=None):
    rclpy.init(args=args)
    node = MyNode() 
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()