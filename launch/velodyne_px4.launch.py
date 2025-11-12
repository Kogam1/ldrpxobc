from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    return LaunchDescription([
        # Velodyne driver node
        Node(
            package='velodyne_driver',
            executable='velodyne_driver_node',
            name='velodyne_driver_node',
            parameters=[{
                'device_ip': '192.168.1.201',  # Adjust based on your setup
                'frame_id': 'velodyne',
                'model': 'VLP16',
                'port': 2368,
                'rpm': 600,
                'read_once': False,
                'read_fast': False,
                'repeat_delay': 0.0,
                'cut_angle': -0.01
            }],
            output='screen'
        ),
        
        # Velodyne point cloud conversion
        Node(
            package='velodyne_pointcloud',
            executable='velodyne_transform_node',
            name='velodyne_transform_node',
            parameters=[{
                'model': 'VLP16',
                'calibration': '/opt/ros/humble/share/velodyne_pointcloud/params/VLP16db.yaml',
                'min_range': 0.9,
                'max_range': 130.0,
                'view_direction': 0.0,
                'view_width': 6.2831
            }],
            output='screen'
        ),
        
        # Our localization node
        Node(
            package='pointcloud_localization',
            executable='pointcloud_localization_node',
            name='pointcloud_localization_node',
            parameters=[{
                'data_save_path': '/home/ubuntu/lidar_data',
                'save_interval': 5.0,
                'voxel_size': 0.1,
                'max_range': 100.0
            }],
            output='screen'
        ),
        
        # SLAM Toolbox for advanced localization (optional)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[{
                'params_file': '/path/to/your/slam_config.yaml'
            }],
            output='screen'
        )
    ])
