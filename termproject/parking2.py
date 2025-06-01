#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from tf_transformations import euler_from_quaternion

class ParkingNode(Node):
    def __init__(self):
        super().__init__("csj_parking")

        self.pose_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, qos_profile_sensor_data)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # control gain
        self.k_rho   = 0.2
        self.k_alpha = 1.0
        self.k_beta  = -0.6

        self.rho_threshold   = 0.05
        self.theta_threshold = math.radians(2.0)
        self.v_max     = 0.12
        self.omega_max = 0.8

        # 런치파일에서 파라미터 식으로 받을 수 있게 수정
        self.declare_parameter('x_goal', 0.0)
        self.declare_parameter('y_goal', 0.0)
        self.declare_parameter('theta_goal_deg', 0.0)

        self.x_goal     = self.get_parameter('x_goal').get_parameter_value().double_value
        self.y_goal     = self.get_parameter('y_goal').get_parameter_value().double_value
        theta_goal_deg = self.get_parameter('theta_goal_deg').get_parameter_value().double_value
        self.theta_goal = math.radians(theta_goal_deg)

        self.is_finished = False

        self.get_logger().info(f"goal = ({self.x_goal:.2f}, {self.y_goal:.2f}, θ={self.theta_goal:.2f} rad)")

    def odom_callback(self, odom_msg: Odometry):
        
        if self.is_finished:
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)
            return

        # 현재 위치/방향 읽기
        x_curr = odom_msg.pose.pose.position.x
        y_curr = odom_msg.pose.pose.position.y
        
        qx = odom_msg.pose.pose.orientation.x
        qy = odom_msg.pose.pose.orientation.y
        qz = odom_msg.pose.pose.orientation.z
        qw = odom_msg.pose.pose.orientation.w
        (roll, pitch, yaw) = euler_from_quaternion([qx, qy, qz, qw])
        theta_curr = yaw

        dx = self.x_goal - x_curr
        dy = self.y_goal - y_curr
        rho = math.sqrt(dx**2 + dy**2)

        theta_err = self.normalize_angle(self.theta_goal - theta_curr)

        # translate
        if rho >= self.rho_threshold:

            alpha = math.atan2(dy, dx) - theta_curr
            alpha = self.normalize_angle(alpha)

            beta = self.normalize_angle(self.theta_goal - theta_curr - alpha)

            v = self.k_rho * rho
            omega = self.k_alpha * alpha + self.k_beta * beta

            if v > self.v_max:
                v = self.v_max
            if v < -self.v_max:
                v = -self.v_max

            if omega > self.omega_max:
                omega = self.omega_max
            if omega < -self.omega_max:
                omega = -self.omega_max

            # publish
            cmd_msg = Twist()
            cmd_msg.linear.x  = v
            cmd_msg.angular.z = omega
            self.cmd_vel_pub.publish(cmd_msg)

            # DEBUG
            self.get_logger().info(
                f"[위치 보정] curr=({x_curr:.2f},{y_curr:.2f},θ={theta_curr:.2f}), "
                f"rho={rho:.2f}, alpha={alpha:.2f}, beta={beta:.2f}, v={v:.2f}, ω={omega:.2f}"
            )
            return

        # heading
        if abs(theta_err) >= self.theta_threshold:

            omega = self.k_alpha * theta_err
            # 속도 제한
            if omega > self.omega_max:
                omega = self.omega_max
            if omega < -self.omega_max:
                omega = -self.omega_max

            cmd_msg = Twist()
            cmd_msg.linear.x  = 0.0
            cmd_msg.angular.z = omega
            self.cmd_vel_pub.publish(cmd_msg)

            # DEBUG
            self.get_logger().info(
                f"[헤딩 보정] curr_theta={theta_curr:.2f}, theta_err={theta_err:.2f}, omega={omega:.2f}"
            )
            return

        # 도착 시 정지
        stop_msg = Twist()
        stop_msg.linear.x  = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_msg)
        self.get_logger().info(f"parking 완료 (최종 위치=({x_curr:.2f},{y_curr:.2f}), 최종 θ={theta_curr:.2f} rad)")

        self.is_finished = True
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
    node = ParkingNode()
    try:
        rclpy.spin(node)
    finally:
        # 종료 시 정지
        stop = Twist()
        stop.linear.x  = 0.0
        stop.angular.z = 0.0
        node.cmd_vel_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
