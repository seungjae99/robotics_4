#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__("csj_waypoint")

        self.pose_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # 런치파일에서 파라미터식으로 받을 수 있게 수정함
        self.declare_parameter('waypoint_1_x', 1.0)
        self.declare_parameter('waypoint_1_y', -0.5)
        self.declare_parameter('waypoint_2_x', 3.0)
        self.declare_parameter('waypoint_2_y', 0.5)
        self.declare_parameter('waypoint_3_x', 2.0)
        self.declare_parameter('waypoint_3_y', -0.5)

        w1_x = self.get_parameter('waypoint_1_x').get_parameter_value().double_value
        w1_y = self.get_parameter('waypoint_1_y').get_parameter_value().double_value
        w2_x = self.get_parameter('waypoint_2_x').get_parameter_value().double_value
        w2_y = self.get_parameter('waypoint_2_y').get_parameter_value().double_value
        w3_x = self.get_parameter('waypoint_3_x').get_parameter_value().double_value
        w3_y = self.get_parameter('waypoint_3_y').get_parameter_value().double_value

        # waypoint_1 = (1.0, -0.5)
        # waypoint_2 = (3.0, 0.5)
        # waypoint_3 = (2.0, -0.5)
        
        waypoint_1 = (w1_x, w1_y)
        waypoint_2 = (w2_x, w2_y)
        waypoint_3 = (w3_x, w3_y)
        
        self.waypoints = [waypoint_1, waypoint_2, waypoint_3, (0.0, 0.0)]

        self.current_idx = 0

        self.pos_threshold   = 0.05
        self.theta_threshold = math.radians(2.0)
        self.v_max     = 0.12
        self.omega_max = 0.8

        # 제어 이득
        self.k_rho   = 0.2
        self.k_alpha = 1.0
        self.k_beta  = -0.6

        self.phase = "MOVE"

        self.get_logger().info("WAYPOINT START")

    def odom_callback(self, odom_msg: Odometry):
        # 현재 위치
        x_curr = odom_msg.pose.pose.position.x
        y_curr = odom_msg.pose.pose.position.y

        # 현재 헤딩(yaw) 계산 (쿼터니언 → Euler)
        qx = odom_msg.pose.pose.orientation.x
        qy = odom_msg.pose.pose.orientation.y
        qz = odom_msg.pose.pose.orientation.z
        qw = odom_msg.pose.pose.orientation.w
        _, _, theta_curr = euler_from_quaternion([qx, qy, qz, qw])

        # 현재 목표 좌표
        x_goal, y_goal = self.waypoints[self.current_idx]

        # 거리 rho 계산
        dx = x_goal - x_curr
        dy = y_goal - y_curr
        rho = math.sqrt(dx**2 + dy**2)

        if self.phase == "MOVE":
            if rho >= self.pos_threshold:
                
                alpha = math.atan2(dy, dx) - theta_curr
                alpha = self.normalize_angle(alpha)

                if self.current_idx < len(self.waypoints) - 1:
                    beta = self.normalize_angle(-theta_curr - alpha)
                    omega = self.k_alpha * alpha + self.k_beta * beta
                else:
                    # 마지막 원점 복귀 시에는 alpha만 사용(수정함)
                    omega = self.k_alpha * alpha

                v = self.k_rho * rho

                # 속도 제한
                v = max(-self.v_max, min(self.v_max, v))
                omega = max(-self.omega_max, min(self.omega_max, omega))

                # publish
                cmd_msg = Twist()
                cmd_msg.linear.x  = v
                cmd_msg.angular.z = omega
                self.cmd_vel_pub.publish(cmd_msg)
                
                self.get_logger().info(
                    f"목표지점: ({x_goal:.2f},{y_goal:.2f}), "
                    f"현재위치: ({x_curr:.2f},{y_curr:.2f},θ={theta_curr:.2f}), "
                    f"rho={rho:.2f}, alpha={alpha:.2f}, omega={omega:.2f}, v={v:.2f}"
                )

                return

            else:
                # 웨이포인트 도달
                self.get_logger().info(
                    f"웨이포인트 idx={self.current_idx} ({x_goal:.2f},{y_goal:.2f}) 도달"
                )
                # 마지막 원점이 아니라면 ROTATE 단계로 전환
                if self.current_idx < len(self.waypoints) - 1:
                    self.phase = "ROTATE"
                    # 다음 웨이포인트 방향 계산
                    x_next, y_next = self.waypoints[self.current_idx + 1]
                    self.theta_goal = math.atan2(y_next - y_goal, x_next - x_goal)
                    return
                else:
                    # 마지막 원점 도달 → 정지
                    self.get_logger().info("모든 웨이포인트 완료, 정지합니다.")
                    stop_msg = Twist()
                    self.cmd_vel_pub.publish(stop_msg)
                    return

        elif self.phase == "ROTATE":
            
            theta_err = self.normalize_angle(self.theta_goal - theta_curr)
            if abs(theta_err) >= self.theta_threshold:
                omega = self.k_alpha * theta_err
                omega = max(-self.omega_max, min(self.omega_max, omega))

                # publish
                cmd_msg = Twist()
                cmd_msg.linear.x  = 0.0
                cmd_msg.angular.z = omega
                self.cmd_vel_pub.publish(cmd_msg)

                return

            else:
                # 회전 완료 → 다음 웨이포인트로 인덱스 증가, MOVE 단계로 전환
                self.get_logger().info(f"idx={self.current_idx} → idx={self.current_idx + 1}")
                self.current_idx += 1
                self.phase = "MOVE"
                return

        # 그 외에는 정지 상태 유지
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
        # 종료 시 무조건 정지
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        node.cmd_vel_pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
