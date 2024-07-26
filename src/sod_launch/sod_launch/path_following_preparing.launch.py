from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, SetEnvironmentVariable

def generate_launch_description():
    return LaunchDescription([
        # Set ROS_DOMAIN_ID
        SetEnvironmentVariable('ROS_DOMAIN_ID', '30'),
        
        # Start obs_pcl_filter node
        Node(
            package='pointcloud_handle',
            executable='obs_pcl_filter',
            name='obs_pcl_filter'
        ),
        
        # Start obs_pcl_circle node
        Node(
            package='pointcloud_handle',
            executable='obs_pcl_circle',
            name='obs_pcl_circle'
        ),
        
        # Start preprocessing node
        Node(
            package='preprocessing',
            executable='preprocessing_node',
            name='preprocessing'
        ),
        
        # Start obs_circle_rviz node
        Node(
            package='pointcloud_handle',
            executable='obs_circle_rviz',
            name='obs_circle_rviz'
        ),
        
        # Start path_generator node
        Node(
            package='path_following_with_obstacles',
            executable='path_generator.py',
            name='path_generator'
        ),
        
        # Start path_rviz_visualizer node
        Node(
            package='path_following_with_obstacles',
            executable='path_rviz_visualizer.py',
            name='path_rviz_visualizer'
        )
    ])