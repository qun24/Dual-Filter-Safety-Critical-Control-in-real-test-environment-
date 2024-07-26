import launch
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    
    # Set ROS domain ID to 30
    set_domain = SetEnvironmentVariable('ROS_DOMAIN_ID', '30')
    config_file_arg = DeclareLaunchArgument('params_file', 
                                            default_value='/home/fantasyyeah/ros2_ws/src/slam_toolbox/online_async_mapping.yaml')
    slam_dir = get_package_share_directory('car_slam')
    rviz_config_dir = os.path.join(slam_dir,'param','slam_toolbox_default.rviz')

    data_sync_py_launch = Node(
        package='sensor_sync_py',
        executable='sensor_sync_node_py',
        name='sensor_sync_py'
    )
    lidar_odom_launch = Node(
        package='lidar_odom_from_scratch',
        executable='lidar_odom_from_scratch_node',
        name='lidar_odom_node',
        output='screen'
    )
    slam_toolbox_launch_file = os.path.join(
        # get_package_share_directory('slam_toolbox'),
        # 'launch',
        # 'online_async_launch.py'
        get_package_share_directory('car_slam'),
        'launch',
        'online_async_launch.py'
    )
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch_file),
        launch_arguments={'params_file' : 
                          LaunchConfiguration('params_file')}.items(),
    )
    rviz_node =  Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen')


    return LaunchDescription([
        set_domain,
        config_file_arg,
        lidar_odom_launch,
        #data_sync_launch,
        #data_sync_py_launch,
        slam_toolbox_launch,
        rviz_node
    ])
