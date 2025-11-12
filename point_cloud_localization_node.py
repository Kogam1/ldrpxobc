#!/usr/bin/env python3
# pointcloud_localization_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu, NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import TransformBroadcaster
import tf_transformations

import numpy as np
import open3d as o3d
import struct
import time
import threading
from datetime import datetime
import os
import json
from pathlib import Path

class PointCloudLocalization(Node):
    def __init__(self):
        super().__init__('pointcloud_localization_node')
        
        # Parameters
        self.declare_parameter('data_save_path', '/home/ubuntu/lidar_data')
        self.declare_parameter('save_interval', 5.0)  # seconds
        self.declare_parameter('voxel_size', 0.1)
        self.declare_parameter('max_range', 100.0)
        
        self.data_path = self.get_parameter('data_save_path').value
        self.save_interval = self.get_parameter('save_interval').value
        self.voxel_size = self.get_parameter('voxel_size').value
        self.max_range = self.get_parameter('max_range').value
        
        # Create data directory
        Path(self.data_path).mkdir(parents=True, exist_ok=True)
        
        # Data storage
        self.pointcloud_data = []
        self.current_pose = None
        self.current_gps = None
        self.current_imu = None
        self.odometry_data = []
        
        # Threading
        self.save_timer = None
        self.is_running = True
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribers
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self.pointcloud_callback,
            10
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/mavros/imu/data',
            self.imu_callback,
            10
        )
        
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/mavros/local_position/odom',
            self.odom_callback,
            10
        )
        
        # Publishers
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/lidar_localization/pose',
            10
        )
        
        # Start save timer
        self.start_save_timer()
        
        self.get_logger().info('Point Cloud Localization Node started')
        
    def pointcloud_callback(self, msg):
        """Process incoming point cloud data"""
        try:
            # Convert PointCloud2 to numpy array
            points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
            
            if points:
                points_array = np.array(points)
                
                # Filter by range
                ranges = np.linalg.norm(points_array, axis=1)
                filtered_points = points_array[ranges <= self.max_range]
                
                # Store with timestamp and pose
                cloud_data = {
                    'timestamp': time.time(),
                    'points': filtered_points,
                    'pose': self.current_pose,
                    'gps': self.current_gps
                }
                
                self.pointcloud_data.append(cloud_data)
                
                # Perform real-time localization
                self.perform_localization(filtered_points, msg.header)
                
        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {str(e)}')
    
    def imu_callback(self, msg):
        """Process IMU data"""
        self.current_imu = {
            'orientation': [
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            ],
            'angular_velocity': [
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ],
            'linear_acceleration': [
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z
            ],
            'timestamp': time.time()
        }
    
    def gps_callback(self, msg):
        """Process GPS data"""
        self.current_gps = {
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'altitude': msg.altitude,
            'timestamp': time.time()
        }
    
    def odom_callback(self, msg):
        """Process odometry data"""
        self.current_pose = {
            'position': [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            ],
            'orientation': [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            ],
            'timestamp': time.time()
        }
        
        # Publish transform
        self.publish_transform(msg.pose.pose, msg.header)
    
    def perform_localization(self, points, header):
        """Perform basic localization using point cloud data"""
        if points.shape[0] < 100:  # Not enough points for localization
            return
            
        try:
            # Convert to Open3D point cloud
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            
            # Simple ground detection (for demo purposes)
            # In production, use more sophisticated SLAM algorithms
            if len(points) > 0:
                # Calculate centroid as simple position estimate
                centroid = np.mean(points, axis=0)
                
                # Create pose message
                pose_msg = PoseWithCovarianceStamped()
                pose_msg.header = header
                pose_msg.header.frame_id = "map"
                
                pose_msg.pose.pose.position.x = centroid[0]
                pose_msg.pose.pose.position.y = centroid[1]
                pose_msg.pose.pose.position.z = centroid[2]
                
                # Use orientation from IMU if available
                if self.current_imu:
                    pose_msg.pose.pose.orientation.x = self.current_imu['orientation'][0]
                    pose_msg.pose.pose.orientation.y = self.current_imu['orientation'][1]
                    pose_msg.pose.pose.orientation.z = self.current_imu['orientation'][2]
                    pose_msg.pose.pose.orientation.w = self.current_imu['orientation'][3]
                else:
                    pose_msg.pose.pose.orientation.w = 1.0  # Default orientation
                
                # Publish pose
                self.pose_pub.publish(pose_msg)
                
        except Exception as e:
            self.get_logger().error(f'Localization error: {str(e)}')
    
    def publish_transform(self, pose, header):
        """Publish TF transform"""
        try:
            t = TransformStamped()
            t.header.stamp = header.stamp
            t.header.frame_id = "map"
            t.child_frame_id = "velodyne"
            
            t.transform.translation.x = pose.position.x
            t.transform.translation.y = pose.position.y
            t.transform.translation.z = pose.position.z
            
            t.transform.rotation.x = pose.orientation.x
            t.transform.rotation.y = pose.orientation.y
            t.transform.rotation.z = pose.orientation.z
            t.transform.rotation.w = pose.orientation.w
            
            self.tf_broadcaster.sendTransform(t)
            
        except Exception as e:
            self.get_logger().error(f'TF publish error: {str(e)}')
    
    def start_save_timer(self):
        """Start periodic data saving"""
        self.save_timer = threading.Timer(self.save_interval, self.save_data_periodically)
        self.save_timer.daemon = True
        self.save_timer.start()
    
    def save_data_periodically(self):
        """Save data at regular intervals"""
        if self.is_running:
            try:
                self.save_current_data()
            except Exception as e:
                self.get_logger().error(f'Error saving data: {str(e)}')
            
            # Restart timer
            self.start_save_timer()
    
    def save_current_data(self):
        """Save current point cloud and localization data"""
        if not self.pointcloud_data:
            return
            
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        try:
            # Save point cloud as PCD file
            if len(self.pointcloud_data) > 0:
                latest_cloud = self.pointcloud_data[-1]
                points = latest_cloud['points']
                
                if len(points) > 0:
                    pcd = o3d.geometry.PointCloud()
                    pcd.points = o3d.utility.Vector3dVector(points)
                    
                    pcd_filename = f"{self.data_path}/pointcloud_{timestamp}.pcd"
                    o3d.io.write_point_cloud(pcd_filename, pcd)
                    
                    # Save metadata
                    metadata = {
                        'timestamp': latest_cloud['timestamp'],
                        'pose': latest_cloud['pose'],
                        'gps': latest_cloud['gps'],
                        'num_points': len(points),
                        'filename': pcd_filename
                    }
                    
                    meta_filename = f"{self.data_path}/metadata_{timestamp}.json"
                    with open(meta_filename, 'w') as f:
                        json.dump(metadata, f, indent=2)
            
            # Save odometry trajectory
            if self.odometry_data:
                traj_filename = f"{self.data_path}/trajectory_{timestamp}.txt"
                np.savetxt(traj_filename, np.array(self.odometry_data), 
                          fmt='%.6f', header='x y z qx qy qz qw timestamp')
            
            self.get_logger().info(f'Data saved for timestamp: {timestamp}')
            
        except Exception as e:
            self.get_logger().error(f'Error saving files: {str(e)}')
    
    def shutdown(self):
        """Clean shutdown"""
        self.is_running = False
        if self.save_timer:
            self.save_timer.cancel()
        
        # Save remaining data
        self.save_current_data()
        self.get_logger().info('Node shutdown complete')

def main():
    rclpy.init()
    
    node = PointCloudLocalization()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Received keyboard interrupt')
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
