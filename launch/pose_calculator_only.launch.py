#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Robot Pose Calculator (calculates robot pose from AprilTag + URDF)
    pose_calculator_node = Node(
        package='the_final_boss',
        executable='robot_pose_calculator.py',
        name='robot_pose_calculator',
        output='screen',
        parameters=[
            {'use_sim_time': False}
        ]
    )
    
    return LaunchDescription([
        pose_calculator_node
    ])
