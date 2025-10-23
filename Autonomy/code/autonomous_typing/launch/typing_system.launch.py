#!/usr/bin/env python3
"""
Launch file for autonomous typing system.

Starts:
- Keyboard localization node (ArUco marker detection)
- Autonomous typing node (main orchestrator)
- Optional: visualization nodes
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for autonomous typing system."""
    
    # Get package directory
    typing_pkg = get_package_share_directory('autonomy_autonomous_typing')
    
    # Declare launch arguments
    declare_keyboard_camera_topic = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/color/image_raw',
        description='Camera image topic for keyboard detection'
    )
    
    declare_keyboard_camera_info = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera/color/camera_info',
        description='Camera info topic for intrinsics'
    )
    
    declare_use_sim = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulated arm and camera'
    )
    
    # Keyboard localization node
    keyboard_localization_node = Node(
        package='autonomy_autonomous_typing',
        executable='autonomous_typing_node',
        name='keyboard_localization',
        output='screen',
        parameters=[
            {
                'camera_topic': LaunchConfiguration('camera_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'marker_dict': 'DICT_4X4_50',
                'marker_size': 0.03,  # 3cm markers
                'config_file': os.path.join(typing_pkg, 'config', 'keyboard.yaml'),
            }
        ]
    )
    
    # Autonomous typing main node
    typing_node = Node(
        package='autonomy_autonomous_typing',
        executable='autonomous_typing_node',
        name='autonomous_typing',
        output='screen',
        remappings=[
            ('keyboard_pose', 'keyboard_pose'),
            ('arm/joint_states', 'arm/joint_states'),
            ('arm/joint_command', 'arm/joint_command'),
        ]
    )
    
    # Optional: RViz visualization
    rviz_config = os.path.join(typing_pkg, 'config', 'typing.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='typing_rviz',
        arguments=['-d', rviz_config],
        condition=LaunchConfigurationNotEquals('use_sim', 'false'),
        on_exit=Shutdown()
    )
    
    return LaunchDescription([
        declare_keyboard_camera_topic,
        declare_keyboard_camera_info,
        declare_use_sim,
        keyboard_localization_node,
        typing_node,
        # rviz_node,  # Uncomment to enable visualization
    ])

