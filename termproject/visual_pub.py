#! /usr/bin/env python3

import rclpy
import rclpy.logging
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion
import math

class MyNode(Node):
    
    def __init__(self):
        super().__init__("csj_visual_marker")
        self.create_timer(1.0,self.timer_callback)
        self.marker_pub = self.create_publisher(Marker, "/marker_goals", 10)
        goal_deg = 90.0
        goal_rad = math.pi * (goal_deg /180.0)
        self.goals = [(1.0, -0.5, goal_rad),]
        
    def timer_callback(self):
        
        for idx, (x_goal, y_goal, theta_goal) in enumerate(self.goals):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "goals"
            marker.id = idx
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            
            marker.pose.position.x = float(x_goal)
            marker.pose.position.y = float(y_goal)
            marker.pose.position.z = 0.0
            
            qz = math.sin(theta_goal / 2.0)
            qw = math.cos(theta_goal / 2.0)
            marker.pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)
            
            marker.scale.x = 0.3
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0


            self.marker_pub.publish(marker)
        
def main(args=None):
    rclpy.init(args=args)
    node = MyNode() 
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()