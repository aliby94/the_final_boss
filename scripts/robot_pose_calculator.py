#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
import numpy as np
from scipy.spatial.transform import Rotation as R
import time

class RobotPoseCalculator(Node):
    def __init__(self):
        super().__init__('robot_pose_calculator')
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/robot_pose_estimate', 
            10
        )
        
        self.timer = self.create_timer(0.1, self.calculate_and_publish_pose)
        
        # Covariance matrix for pose estimates
        self.pose_covariance = [
            0.01, 0.0,  0.0,  0.0,  0.0,  0.0,     # x
            0.0,  0.01, 0.0,  0.0,  0.0,  0.0,     # y
            0.0,  0.0,  0.01, 0.0,  0.0,  0.0,     # z
            0.0,  0.0,  0.0,  0.01, 0.0,  0.0,     # roll
            0.0,  0.0,  0.0,  0.0,  0.01, 0.0,     # pitch
            0.0,  0.0,  0.0,  0.0,  0.0,  0.01     # yaw
        ]
        
        self.get_logger().info("Robot Pose Calculator Node Started")
        
    def calculate_and_publish_pose(self):
        try:
            # Get AprilTag detection: camera_link -> object (robotics frame)
            camera_to_object = self.tf_buffer.lookup_transform(
                'camera_link',  # Changed from camera_color_optical_frame
                'object',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # Get URDF transform: base_link -> Apriltag
            base_to_apriltag = self.tf_buffer.lookup_transform(
                'base_link',
                'Apriltag',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # Calculate robot pose
            robot_pose = self.calculate_robot_pose(camera_to_object, base_to_apriltag)
            
            # Publish pose estimate
            self.publish_pose_estimate(robot_pose, camera_to_object.header.stamp)
            
        except TransformException as ex:
            self.get_logger().debug(f'Could not get transforms: {ex}')
            return
        except Exception as ex:
            self.get_logger().error(f'Error in pose calculation: {ex}')
            return
    
    def calculate_robot_pose(self, camera_to_object, base_to_apriltag):
        """
        Calculate camera_link -> base_link transformation
        Both are now in robotics coordinate frame (X=forward, Y=left, Z=up)
        """
        
        # Convert transforms to homogeneous matrices
        T_camera_object = self.transform_to_matrix(camera_to_object.transform)
        T_base_apriltag = self.transform_to_matrix(base_to_apriltag.transform)
        
        # Calculate apriltag -> base (inverse of base -> apriltag)
        T_apriltag_base = np.linalg.inv(T_base_apriltag)
        
        # Calculate camera -> base_link
        # Since object and Apriltag are the same physical entity:
        T_camera_base = T_camera_object @ T_apriltag_base
        
        return T_camera_base
    
    def transform_to_matrix(self, transform):
        """Convert ROS Transform to 4x4 homogeneous matrix"""
        t = np.array([
            transform.translation.x,
            transform.translation.y,
            transform.translation.z
        ])
        
        q = [
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w
        ]
        rotation_matrix = R.from_quat(q).as_matrix()
        
        T = np.eye(4)
        T[:3, :3] = rotation_matrix
        T[:3, 3] = t
        
        return T
    
    def matrix_to_pose(self, matrix):
        """Convert 4x4 homogeneous matrix to ROS Pose"""
        from geometry_msgs.msg import Pose
        
        pose = Pose()
        
        pose.position.x = matrix[0, 3]
        pose.position.y = matrix[1, 3]
        pose.position.z = matrix[2, 3]
        
        rotation_matrix = matrix[:3, :3]
        quat = R.from_matrix(rotation_matrix).as_quat()
        
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        
        return pose
    
    def publish_pose_estimate(self, robot_pose_matrix, timestamp):
        """Publish calculated robot pose as PoseWithCovarianceStamped"""
        
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = timestamp
        pose_msg.header.frame_id = 'camera_link'  # Changed from odom to camera_link
        
        pose_msg.pose.pose = self.matrix_to_pose(robot_pose_matrix)
        pose_msg.pose.covariance = self.pose_covariance
        
        self.pose_pub.publish(pose_msg)
        
        pos = pose_msg.pose.pose.position
        self.get_logger().debug(
            f'Published robot pose: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f}'
        )

def main(args=None):
    rclpy.init(args=args)
    
    node = RobotPoseCalculator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
