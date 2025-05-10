from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('ssl_slam')

    # Launch arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )

    # Parameters
    use_sim_time = True
    scan_period = 0.1
    vertical_angle = 2.0
    max_dis = 10.0
    map_resolution = 0.02
    min_dis = 0.1
    skip_frames = 1
    map_path = os.path.join(pkg_share, 'map')
    min_map_update_distance = 1.0
    min_map_update_angle = 30.0
    min_map_update_frame = 8.0

    # Nodes
    laser_processing_node = Node(
        package='ssl_slam',
        executable='ssl_slam_laser_processing_node',
        name='ssl_slam_laser_processing_node',
        output='screen',
        parameters=[{
            'scan_period': scan_period,
            'vertical_angle': vertical_angle,
            'max_dis': max_dis,
            'map_resolution': map_resolution,
            'min_dis': min_dis,
            'skip_frames': skip_frames,
            'map_path': map_path,
            'min_map_update_distance': min_map_update_distance,
            'min_map_update_angle': min_map_update_angle,
            'min_map_update_frame': min_map_update_frame,
            'use_sim_time': use_sim_time
        }]
    )

    odom_estimation_node = Node(
        package='ssl_slam',
        executable='ssl_slam_odom_estimation_mapping_node',
        name='ssl_slam_odom_estimation_mapping_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    map_optimization_node = Node(
        package='ssl_slam',
        executable='ssl_slam_map_optimization_node',
        name='ssl_slam_map_optimization_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    # Static transform publisher
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='baselink2cam_tf',
        arguments=['0.0', '0', '0.0', '0', '0', '0', 'base_link', 'camera_link']
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'SSLMapping.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        rviz_arg,
        laser_processing_node,
        odom_estimation_node,
        map_optimization_node,
        static_tf,
        rviz_node,
        # trajectory_server
    ])