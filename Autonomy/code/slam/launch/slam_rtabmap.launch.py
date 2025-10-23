#!/usr/bin/env python3
"""
Main launch file for RGB-D SLAM system with GPS integration.

Launches:
1. Depth processor for desert noise filtering
2. RTAB-Map SLAM with RGB-D camera
3. GPS fusion layer for global localization
4. SLAM orchestrator for health monitoring
"""

from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.compositions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os


def generate_launch_description():
    """Generate launch description for complete SLAM system."""
    
    # Get package share directory
    slam_pkg_share = FindPackageShare('autonomy_slam')
    config_dir = PathJoinSubstitution([slam_pkg_share, 'config'])
    
    # Configuration files
    rtabmap_config = PathJoinSubstitution([config_dir, 'rtabmap_desert.yaml'])
    depth_config = PathJoinSubstitution([config_dir, 'depth_processing.yaml'])
    gps_fusion_config = PathJoinSubstitution([config_dir, 'gps_fusion.yaml'])
    
    return LaunchDescription([
        # Depth Processor Node
        Node(
            package='autonomy_slam',
            executable='depth_processor',
            name='depth_processor',
            output='screen',
            parameters=[depth_config],
            remappings=[
                ('camera/rgb/image_raw', '/camera/rgb/image_raw'),
                ('camera/depth/image_raw', '/camera/depth/image_raw'),
                ('camera/depth/camera_info', '/camera/depth/camera_info'),
            ]
        ),
        
        # RTAB-Map SLAM Node
        Node(
            package='rtabmap_ros',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[rtabmap_config],
            arguments=[
                '--delete_db_on_start',  # Clean database on startup for testing
            ],
            remappings=[
                ('rgb/image', 'slam/rgb/image'),
                ('depth/image', 'slam/depth/processed'),
                ('rgb/camera_info', 'slam/depth/camera_info'),
                ('odom', 'odom'),
            ],
        ),
        
        # GPS Fusion Node
        Node(
            package='autonomy_slam',
            executable='gps_fusion_node',
            name='gps_fusion_node',
            output='screen',
            parameters=[gps_fusion_config],
            remappings=[
                ('slam/pose', 'rtabmap/pose'),
                ('gps/fix', '/gps/fix'),
            ]
        ),
        
        # SLAM Orchestrator (health monitoring and coordination)
        Node(
            package='autonomy_slam',
            executable='slam_node',
            name='slam_orchestrator',
            output='screen',
            parameters=[
                {'enable_depth_processing': True},
                {'enable_gps_fusion': True},
                {'enable_diagnostics': True},
                {'min_feature_count': 50},
            ],
            remappings=[
                ('slam/pose', 'rtabmap/pose'),
                ('slam/pose/fused', 'gps_fusion_node/slam/pose/fused'),
                ('rtabmap/stat', 'rtabmap/stat'),
            ]
        ),
    ])

