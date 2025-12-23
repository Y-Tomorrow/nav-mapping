#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 声明启动参数
    exploration_radius_arg = DeclareLaunchArgument(
        'exploration_radius',
        default_value='-1.0',
        description='探索半径（米），负数表示无限制'
    )
    
    frontier_distance_threshold_arg = DeclareLaunchArgument(
        'frontier_distance_threshold',
        default_value='0.5',
        description='前沿点聚类距离阈值（米）'
    )
    
    min_frontier_size_arg = DeclareLaunchArgument(
        'min_frontier_size',
        default_value='20',
        description='最小前沿点数量'
    )
    
    goal_timeout_arg = DeclareLaunchArgument(
        'goal_timeout',
        default_value='60.0',
        description='目标超时时间（秒）'
    )
    
    update_frequency_arg = DeclareLaunchArgument(
        'update_frequency',
        default_value='1.0',
        description='更新频率（Hz）'
    )
    
    stuck_threshold_arg = DeclareLaunchArgument(
        'stuck_threshold',
        default_value='0.1',
        description='卡死检测阈值（米）'
    )
    
    stuck_timeout_arg = DeclareLaunchArgument(
        'stuck_timeout',
        default_value='30.0',
        description='卡死超时时间（秒）'
    )

    # 启动仿真环境
    explore_launch_dir = os.path.join(
        get_package_share_directory('wpr_simulation2'),
        'launch'
    )
    
    explore_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(explore_launch_dir, 'explore.launch.py')
        )
    )

    # 启动建图
    slam_launch_dir = os.path.join(
        get_package_share_directory('slam_pkg'),
        'launch'
    )
    
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_launch_dir, 'slam.launch.py')
        )
    )

    # 注意：导航需要单独启动
    # 运行: ros2 launch nav_pkg nav.launch.py

    # 探索节点
    explore_node = Node(
        package='explore_pkg',
        executable='explore_node',
        name='explore_node',
        output='screen',
        parameters=[{
            'exploration_radius': LaunchConfiguration('exploration_radius'),
            'frontier_distance_threshold': LaunchConfiguration('frontier_distance_threshold'),
            'min_frontier_size': LaunchConfiguration('min_frontier_size'),
            'goal_timeout': LaunchConfiguration('goal_timeout'),
            'update_frequency': LaunchConfiguration('update_frequency'),
            'map_topic': '/map',
            'robot_base_frame': 'base_footprint',
            'map_frame': 'map',
            'nav_action_name': '/navigate_to_pose',
            'stuck_threshold': LaunchConfiguration('stuck_threshold'),
            'stuck_timeout': LaunchConfiguration('stuck_timeout'),
        }]
    )

    return LaunchDescription([
        exploration_radius_arg,
        frontier_distance_threshold_arg,
        min_frontier_size_arg,
        goal_timeout_arg,
        update_frequency_arg,
        stuck_threshold_arg,
        stuck_timeout_arg,
        explore_sim,
        slam,
        explore_node,
    ])

