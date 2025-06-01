#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('tb3_controller')

    rviz_config = os.path.join(pkg_share, 'rviz', 'robotics_rviz.rviz')
    
    waypoint_1_x_value = 1.0
    waypoint_1_y_value = -0.5
    waypoint_2_x_value = 3.0
    waypoint_2_y_value = 0.5
    waypoint_3_x_value = 2.0
    waypoint_3_y_value = -0.5

    return LaunchDescription([
        # rviz2 노드 실행
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),

        # task3 : waypoint 노드 실행
        Node(
            package='tb3_controller',
            executable='waypoint',
            name='waypoint',
            output='screen',
            parameters=[
                {'waypoint_1_x': waypoint_1_x_value},
                {'waypoint_1_y': waypoint_1_y_value},
                {'waypoint_2_x': waypoint_2_x_value},
                {'waypoint_2_y': waypoint_2_y_value},
                {'waypoint_3_x': waypoint_3_x_value},
                {'waypoint_3_y': waypoint_3_y_value},
            ]
        ),
    ])

