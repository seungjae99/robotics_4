#!/usr/bin/env python3

import math
from math import isfinite

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from tf_transformations import euler_from_quaternion
from rclpy.duration import Duration


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__("team4_waypoint")

        self.pose_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, qos_profile_sensor_data)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, "/marker_goals_array", 10)

        self.declare_parameter('waypoint_1_x', 2.0)
        self.declare_parameter('waypoint_1_y', 1.0)
        self.declare_parameter('waypoint_2_x', 4.0)
        self.declare_parameter('waypoint_2_y', -1.0)
        self.declare_parameter('waypoint_3_x', 6.0)
        self.declare_parameter('waypoint_3_y', 1.0)

        w1 = (self.get_parameter('waypoint_1_x').value,
            self.get_parameter('waypoint_1_y').value)
        w2 = (self.get_parameter('waypoint_2_x').value,
            self.get_parameter('waypoint_2_y').value)
        w3 = (self.get_parameter('waypoint_3_x').value,
            self.get_parameter('waypoint_3_y').value)

        # 마지막에 원점(0,0)으로 복귀
        self.waypoints = [w1, w2, w3, (0.0, 0.0)]
        self.current_idx = 0

        self.pos_threshold = 0.05
        self.v_const = 0.2
        self.omega_max = 0.8
        self.k_alpha = 1.0

        self.range_avg = float('inf')
        self.obstacle_dist_threshold = 1.0  # 미터
        
        self.avoiding = False
        self.avoid_duration = 1.0  # seconds
        self.avoid_end_time = None

        self.get_logger().info("WAYPOINT FOLLOWING STARTED")
        self.create_timer(1.0, self.publish_marker_array)

    def publish_marker_array(self):
        marker_array = MarkerArray()
        for idx, (x, y) in enumerate(self.waypoints):
            m = Marker()
            m.header.frame_id = "odom"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "waypoints"
            m.id = idx
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.2
            m.pose.orientation = Quaternion(w=1.0)
            m.scale.z = 0.3
            m.color.r = m.color.g = m.color.b = m.color.a = 1.0
            m.text = f"WP{idx+1}"
            marker_array.markers.append(m)
        self.marker_pub.publish(marker_array)

    def scan_callback(self, scan: LaserScan):
        N = len(scan.ranges)
        n_deg = 60

        neg_slice = scan.ranges[N - n_deg : N]
        pos_slice = scan.ranges[0 : n_deg + 1]

        window = [r for r in (neg_slice + pos_slice) if isfinite(r)]

        if window:
            self.range_avg = sum(window) / len(window)
        else:
            self.range_avg = float('inf')


    def odom_callback(self, odom: Odometry):
        if self.current_idx >= len(self.waypoints):
            self.cmd_vel_pub.publish(Twist())
            return

        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        q = odom.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        if self.current_idx == len(self.waypoints) - 1:
            now = self.get_clock().now()

            if self.avoiding:
                if now < self.avoid_end_time:
                    twist = Twist()
                    twist.linear.x = 0.1
                    twist.angular.z = 0.3
                    self.cmd_vel_pub.publish(twist)
                    self.get_logger().info("⛔ 회피 중 (타이머 작동)")
                    return
                else:
                    self.avoiding = False

            if self.range_avg < self.obstacle_dist_threshold:
                self.avoiding = True
                self.avoid_end_time = now + Duration(seconds=self.avoid_duration)

                twist = Twist()
                twist.linear.x = 0.1
                twist.angular.z = 0.3
                self.cmd_vel_pub.publish(twist)
                self.get_logger().info("⛔ 장애물 감지! 1초 회피 시작")
                return

        x_goal, y_goal = self.waypoints[self.current_idx]
        dx = x_goal - x
        dy = y_goal - y
        rho = math.hypot(dx, dy)

        if rho < self.pos_threshold:
            self.get_logger().info(f"[{self.current_idx}] WP 도달 ({x_goal:.2f}, {y_goal:.2f})")
            self.current_idx += 1
            return

        alpha = self.normalize_angle(math.atan2(dy, dx) - yaw)
        omega = max(-self.omega_max, min(self.omega_max, self.k_alpha * alpha))

        cmd = Twist()
        cmd.linear.x = self.v_const
        cmd.angular.z = omega
        self.cmd_vel_pub.publish(cmd)

        self.get_logger().info(
            f"→ WP{self.current_idx+1}: goal=({x_goal:.2f},{y_goal:.2f}), "
            f"pos=({x:.2f},{y:.2f}, θ={yaw:.2f}), "
            f"α={alpha:.2f}, ω={omega:.2f}, "
            f"range_avg={self.range_avg:.2f}m"
        )
        
    @staticmethod
    def normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2*math.pi
        while angle < -math.pi:
            angle += 2*math.pi
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
