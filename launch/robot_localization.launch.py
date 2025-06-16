#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('the_final_boss')
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    ekf_config_file = os.path.join(pkg_share, 'config', 'ekf.yaml')
    
    # Robot State Publisher (publishes robot URDF transforms)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', urdf_file])}],
        output='screen'
    )
    
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
    
    # TF Static Publisher (publishes static transforms if needed)
    # This connects your robot's world frame to the camera frame
    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_world_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',  # translation
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',  # rotation
            '--frame-id', 'world',  # parent frame
            '--child-frame-id', 'camera_color_optical_frame'  # child frame
        ],
        output='screen'
    )
    
    return LaunchDescription([
        robot_state_publisher_node,
        pose_calculator_node,
        ekf_node,
        static_tf_publisher
    ])
