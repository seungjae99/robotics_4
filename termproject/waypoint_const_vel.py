#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__("team4_waypoint")

        self.pose_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/marker_goals_array", 10)

        # 런치파일에서 파라미터식으로 받을 수 있게 수정함
        self.declare_parameter('waypoint_1_x', 2.0)
        self.declare_parameter('waypoint_1_y', 1.0)
        self.declare_parameter('waypoint_2_x', 4.0)
        self.declare_parameter('waypoint_2_y', -1.0)
        self.declare_parameter('waypoint_3_x', 6.0)
        self.declare_parameter('waypoint_3_y', 1.0)

        w1 = (self.get_parameter('waypoint_1_x').get_parameter_value().double_value,
            self.get_parameter('waypoint_1_y').get_parameter_value().double_value)
        w2 = (self.get_parameter('waypoint_2_x').get_parameter_value().double_value,
            self.get_parameter('waypoint_2_y').get_parameter_value().double_value)
        w3 = (self.get_parameter('waypoint_3_x').get_parameter_value().double_value,
            self.get_parameter('waypoint_3_y').get_parameter_value().double_value)

        self.waypoints = [w1, w2, w3, (0.0, 0.0)]
        self.current_idx = 0

        # Thresholds
        self.pos_threshold = 0.05

        # Constant speed
        self.v_const = 0.2
        self.omega_max = 0.8
        self.k_alpha = 1.0

        self.get_logger().info("WAYPOINT FOLLOWING STARTED")
        self.create_timer(1.0, self.publish_marker_array)

    def publish_marker_array(self):
        marker_array = MarkerArray()
        for idx, (x, y) in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoints"
            marker.id = idx
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.2
            marker.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            marker.scale.z = 0.3
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker.text = f"WP{idx + 1}"
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def odom_callback(self, odom_msg: Odometry):
        if self.current_idx >= len(self.waypoints):
            self.cmd_vel_pub.publish(Twist())
            return

        x_curr = odom_msg.pose.pose.position.x
        y_curr = odom_msg.pose.pose.position.y
        qx = odom_msg.pose.pose.orientation.x
        qy = odom_msg.pose.pose.orientation.y
        qz = odom_msg.pose.pose.orientation.z
        qw = odom_msg.pose.pose.orientation.w
        _, _, theta_curr = euler_from_quaternion([qx, qy, qz, qw])

        x_goal, y_goal = self.waypoints[self.current_idx]
        dx = x_goal - x_curr
        dy = y_goal - y_curr
        rho = math.sqrt(dx**2 + dy**2)

        if rho < self.pos_threshold:
            self.get_logger().info(f"[{self.current_idx}] ({x_goal:.2f},{y_goal:.2f}) 도달")
            self.current_idx += 1
            return

        alpha = self.normalize_angle(math.atan2(dy, dx) - theta_curr)
        omega = self.k_alpha * alpha
        omega = max(-self.omega_max, min(self.omega_max, omega))

        cmd_msg = Twist()
        cmd_msg.linear.x = self.v_const
        cmd_msg.angular.z = omega
        self.cmd_vel_pub.publish(cmd_msg)

        self.get_logger().info(
            f"목표:({x_goal:.2f},{y_goal:.2f}) 현재:({x_curr:.2f},{y_curr:.2f}, θ={theta_curr:.2f}) "
            f"alpha={alpha:.2f}, v={self.v_const:.2f}, omega={omega:.2f}"
        )

    @staticmethod
    def normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    try:
        rclpy.spin(node)
    finally:
        node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()