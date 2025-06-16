# the_final_boss


ros2 launch the_final_boss robot_description_only.launch.py

Publishes robot structure - defines base_link → {imu_link, Apriltag} transforms from URDF

ros2 launch the_final_boss pose_calculator_only.launch.py

Calculates robot pose - combines camera detection + URDF to compute camera → base_link, publishes /robot_pose_estimate

ros2 launch the_final_boss ekf_only.launch.py

Sensor fusion brain - fuses IMU + vision data, publishes continuous /odometry/filtered

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom camera_link
