#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__("team4_waypoint_with_conditional_avoidance")

        self.pose_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, qos_profile_sensor_data)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.declare_parameter('waypoint_1_x', 1.0)
        self.declare_parameter('waypoint_1_y', -0.5)
        self.declare_parameter('waypoint_2_x', 3.0)
        self.declare_parameter('waypoint_2_y', 0.5)
        self.declare_parameter('waypoint_3_x', 2.0)
        self.declare_parameter('waypoint_3_y', -0.5)

        w1 = (self.get_parameter('waypoint_1_x').get_parameter_value().double_value,
            self.get_parameter('waypoint_1_y').get_parameter_value().double_value)
        w2 = (self.get_parameter('waypoint_2_x').get_parameter_value().double_value,
            self.get_parameter('waypoint_2_y').get_parameter_value().double_value)
        w3 = (self.get_parameter('waypoint_3_x').get_parameter_value().double_value,
            self.get_parameter('waypoint_3_y').get_parameter_value().double_value)

        self.waypoints = [w1, w2, w3, (0.0, 0.0)]
        self.current_idx = 0

        self.pos_threshold = 0.1
        self.theta_threshold = math.radians(15.0)
        self.v_min = 0.08
        self.v_max = 0.12
        self.omega_max = 0.8
        
        # Control Gain
        self.k_rho = 0.2
        self.k_alpha = 1.0
        self.k_beta = -0.3

        self.phase = "MOVE"
        self.SAFE_DISTANCE = 0.5
        self.avoid_start_time = None
        self.avoid_rotate_duration = 1.5
        self.avoid_forward_duration = 5.0

        self.get_logger().info("TASK3 START")

    def scan_callback(self, scan: LaserScan):
        # 회피는 오직 wp3 → wp4 이동 중일 때만 (idx == 3)
        if self.current_idx != 3 or self.phase not in ["MOVE", "ROTATE"]:
            return

        front_ranges = []

        for i, dist in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment

            if scan.range_min < dist < scan.range_max:
                # 전방 60도 = [330° ~ 360°] ∪ [0° ~ 30°]
                if (angle >= 5.76 and angle <= 6.28) or (angle >= 0.0 and angle <= 0.523):
                    front_ranges.append(dist)

        if front_ranges and min(front_ranges) < self.SAFE_DISTANCE:
            self.phase = "AVOID_ROTATE"
            self.avoid_start_time = self.get_clock().now()
            self.get_logger().warn("장애물 감지됨! 왼쪽으로 회피 시작.")

    def odom_callback(self, odom_msg: Odometry):
        now = self.get_clock().now()

        if self.phase == "AVOID_ROTATE":
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.8
            self.cmd_vel_pub.publish(twist)

            if (now - self.avoid_start_time).nanoseconds * 1e-9 >= self.avoid_rotate_duration:
                self.phase = "AVOID_FORWARD"
                self.avoid_start_time = now
            return

        elif self.phase == "AVOID_FORWARD":
            twist = Twist()
            twist.linear.x = 0.12
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)

            if (now - self.avoid_start_time).nanoseconds * 1e-9 >= self.avoid_forward_duration:
                self.phase = "MOVE"
                self.get_logger().info("회피 완료. 웨이포인트 추적 재개.")
            return

        # 현재 위치 및 자세 계산
        x_curr = odom_msg.pose.pose.position.x
        y_curr = odom_msg.pose.pose.position.y
        q = odom_msg.pose.pose.orientation
        _, _, theta_curr = euler_from_quaternion([q.x, q.y, q.z, q.w])

        x_goal, y_goal = self.waypoints[self.current_idx]
        dx = x_goal - x_curr
        dy = y_goal - y_curr
        rho = math.sqrt(dx**2 + dy**2)

        if self.phase == "MOVE":
            if rho >= self.pos_threshold:
                alpha = self.normalize_angle(math.atan2(dy, dx) - theta_curr)
                if self.current_idx < len(self.waypoints) - 1:
                    beta = self.normalize_angle(-theta_curr - alpha)
                    omega = self.k_alpha * alpha + self.k_beta * beta
                else:
                    omega = self.k_alpha * alpha

                v = max(self.v_min, min(self.v_max, self.k_rho * rho))
                omega = max(-self.omega_max, min(self.omega_max, omega))

                twist = Twist()
                twist.linear.x = v
                twist.angular.z = omega
                self.cmd_vel_pub.publish(twist)

                self.get_logger().info(
                    f"목표: ({x_goal:.2f},{y_goal:.2f}), "
                    f"현재: ({x_curr:.2f},{y_curr:.2f}, θ={theta_curr:.2f}), "
                    f"rho={rho:.2f}, alpha={alpha:.2f}, omega={omega:.2f}, v={v:.2f}"
                )
            else:
                self.get_logger().info(f"웨이포인트 {self.current_idx} 도달: ({x_goal:.2f}, {y_goal:.2f})")
                if self.current_idx < len(self.waypoints) - 1:
                    self.phase = "ROTATE"
                    x_next, y_next = self.waypoints[self.current_idx + 1]
                    self.theta_goal = math.atan2(y_next - y_goal, x_next - x_goal)
                else:
                    self.get_logger().info("모든 웨이포인트 완료. 정지")
                    self.cmd_vel_pub.publish(Twist())
            return

        elif self.phase == "ROTATE":
            theta_err = self.normalize_angle(self.theta_goal - theta_curr)
            if abs(theta_err) >= self.theta_threshold:
                omega = max(-self.omega_max, min(self.omega_max, self.k_alpha * theta_err))
                twist = Twist()
                twist.angular.z = omega
                self.cmd_vel_pub.publish(twist)
            else:
                self.get_logger().info(f"회전 완료. idx {self.current_idx} → {self.current_idx + 1}")
                self.current_idx += 1
                self.phase = "MOVE"
            return

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
