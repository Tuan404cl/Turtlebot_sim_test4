#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Launch file khởi chạy toàn bộ hệ thống Phân vùng Bản đồ & Điều hướng Bằng Giọng nói
cho Robot TurtleBot 4 trên ROS 2 Humble & Nav2:
1. Gazebo Simulation (TurtleBot 4 Standard trong thế giới warehouse)
2. TF Relay & Mock Hardware Services
3. Localization (Map Server nạp bản đồ maps/map.yaml + AMCL định vị)
4. RViz2 hiển thị bản đồ, LaserScan, Costmaps, Path và nhãn 3D các khu vực (/semantic_regions_markers)
5. Tự động Undock robot khỏi trạm sạc
6. Nav2 Stack (Planner, Controller, Behavior, BT Navigator, Velocity Smoother)
7. Node điều hướng giọng nói: voice_navigation_node (subscribe /voice_command -> NavigateToPose)
8. Node nhận dạng giọng nói: voice_command_node (chuyển đổi giọng nói -> /voice_command)
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_practice = get_package_share_directory('turtlebot4_practice4')
    pkg_tb4_nav = get_package_share_directory('turtlebot4_navigation')

    sim_launch = PathJoinSubstitution([pkg_practice, 'launch', 'sim_robot.launch.py'])
    default_map = PathJoinSubstitution([pkg_practice, 'maps', 'map.yaml'])
    loc_params = PathJoinSubstitution([pkg_practice, 'config', 'localization_params.yaml'])
    nav2_params = PathJoinSubstitution([pkg_practice, 'config', 'nav2_params.yaml'])
    rviz_config = PathJoinSubstitution([pkg_practice, 'config', 'turtlebot4_nav2.rviz'])

    tb4_loc_launch = PathJoinSubstitution([pkg_tb4_nav, 'launch', 'localization.launch.py'])
    tb4_nav_launch = PathJoinSubstitution([pkg_tb4_nav, 'launch', 'nav2.launch.py'])
    fastdds_profile = PathJoinSubstitution([pkg_practice, 'config', 'fastdds_udp_only.xml'])

    set_fastdds = SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', fastdds_profile)

    arguments = [
        DeclareLaunchArgument('namespace', default_value='', description='Robot namespace'),
        DeclareLaunchArgument(
            'launch_sim',
            default_value='true',
            choices=['true', 'false'],
            description='Khởi chạy kèm Gazebo simulation',
        ),
        DeclareLaunchArgument('world', default_value='warehouse', description='Gazebo world'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
            description='Sử dụng simulation time',
        ),
        DeclareLaunchArgument(
            'map', default_value=default_map, description='Đường dẫn file map.yaml'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            choices=['true', 'false'],
            description='Mở RViz2 hiển thị bản đồ, các vùng ngữ nghĩa và Nav2',
        ),
        DeclareLaunchArgument(
            'enable_voice_studio',
            default_value='true',
            choices=['true', 'false'],
            description='Mở bảng điều khiển thu âm trực tiếp từ Microphone (Warehouse Voice Studio)',
        ),
        DeclareLaunchArgument(
            'continuous_mic',
            default_value='false',
            choices=['true', 'false'],
            description='Bật sẵn chế độ tự động lắng nghe liên tục qua Microphone (Auto-Listen VAD)',
        ),
    ]

    # 1. Gazebo Simulation
    simulation = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([sim_launch]),
                launch_arguments=[
                    ('world', LaunchConfiguration('world')),
                    ('namespace', LaunchConfiguration('namespace')),
                    ('rviz', 'false'),
                ],
            )
        ],
        scoped=True,
        condition=IfCondition(LaunchConfiguration('launch_sim')),
    )

    # 2. TF Relay & Helper Services (sau 3s)
    tf_relay_node = Node(
        package='turtlebot4_practice4',
        executable='tf_relay',
        name='tf_relay',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )
    delayed_tf_relay = TimerAction(period=3.0, actions=[tf_relay_node])

    # 3. Localization (Map Server + AMCL) ở giây thứ 10 (sau khi robot đã spawn xong tại t=4.5s)
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([tb4_loc_launch]),
        launch_arguments=[
            ('namespace', LaunchConfiguration('namespace')),
            ('map', LaunchConfiguration('map')),
            ('params', loc_params),
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
        ],
    )
    delayed_localization = TimerAction(period=10.0, actions=[localization])

    # 4. RViz2 ở giây thứ 10
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_voice_nav',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    delayed_rviz = TimerAction(period=10.0, actions=[rviz_node])

    # 5. Tự động Undock robot ở giây thứ 20
    undock_node = Node(
        package='turtlebot4_practice4',
        executable='auto_undock',
        name='auto_undocker',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )
    delayed_undock = TimerAction(period=20.0, actions=[undock_node])

    # 6. Nav2 Navigation ở giây thứ 34
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([tb4_nav_launch]),
        launch_arguments=[
            ('namespace', LaunchConfiguration('namespace')),
            ('params_file', nav2_params),
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
        ],
    )
    delayed_navigation = TimerAction(period=34.0, actions=[navigation])

    # 7. Node điều hướng bằng giọng nói (voice_navigation_node) ở giây thứ 46
    voice_nav_node = Node(
        package='turtlebot4_practice4',
        executable='voice_navigation_node',
        name='voice_navigation_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )
    delayed_voice_nav = TimerAction(period=46.0, actions=[voice_nav_node])

    # 8. Node thu âm và nhận dạng giọng nói trực tiếp từ Microphone (voice_command_node) ở giây thứ 50
    voice_studio_node = Node(
        package='turtlebot4_practice4',
        executable='voice_command_node',
        name='voice_command_studio',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_voice_studio')),
    )
    delayed_voice_studio = TimerAction(period=50.0, actions=[voice_studio_node])

    ld = LaunchDescription(arguments)
    ld.add_action(set_fastdds)
    ld.add_action(simulation)
    ld.add_action(delayed_tf_relay)
    ld.add_action(delayed_localization)
    ld.add_action(delayed_rviz)
    ld.add_action(delayed_undock)
    ld.add_action(delayed_navigation)
    ld.add_action(delayed_voice_nav)
    ld.add_action(delayed_voice_studio)
    return ld

