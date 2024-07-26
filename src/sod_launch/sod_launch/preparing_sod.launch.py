from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, SetEnvironmentVariable

def generate_launch_description():
    return LaunchDescription([
        # Set ROS_DOMAIN_ID
        SetEnvironmentVariable('ROS_DOMAIN_ID', '30'),

        # Start lidar_odom_from_scratch node
        Node(
            package='lidar_odom_from_scratch',
            executable='lidar_odom_from_scratch_node',
            name='lidar_odom_from_scratch'
        ),

        # Start static_transform_publisher
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0.12', '0', '0.02', '0', '0', '0', 'base_link', 'camera_link']
        ),

        # Start sod_mds node
        Node(
            package='sod_mds',
            executable='sod_mbs_node',
            name='sod_mbs'
        )
    ])