#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('tb3_controller')

    rviz_config = os.path.join(pkg_share, 'rviz', 'robotics_rviz.rviz')
    
    x_goal_value = 1.0
    y_goal_value = -0.5
    theta_goal_deg_value = 90.0

    return LaunchDescription([
        # rviz2 노드 실행
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),

        # task2 : parking 노드 실행
        Node(
            package='tb3_controller',
            executable='parking2',
            name='parking2',
            output='screen',
            parameters=[
                {'x_goal': x_goal_value},
                {'y_goal': y_goal_value},
                {'theta_goal_deg': theta_goal_deg_value}
            ]
        ),
    ])

