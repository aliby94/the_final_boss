#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('the_final_boss')
    ekf_config_file = os.path.join(pkg_share, 'config', 'ekf.yaml')
    
    # Extended Kalman Filter (fuses IMU + pose estimates)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file],
        remappings=[
            ('odometry/filtered', '/odometry/filtered')
        ]
    )
    
    return LaunchDescription([
        ekf_node
    ])
