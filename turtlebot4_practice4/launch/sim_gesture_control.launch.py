#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Launch file khởi chạy hệ thống Điều khiển Hành vi TurtleBot 4 bằng Cử chỉ tay AI
(MediaPipe Gesture Recognizer + OpenCV + Gazebo Simulation + Map Server & AMCL trên RViz2)

Bao gồm:
1. Gazebo Simulation (TurtleBot 4 Standard trong thế giới warehouse rộng rãi)
2. TF Relay & Mock Hardware Services
3. Localization (Map Server nạp bản đồ maps/map.yaml đã lưu + AMCL định vị trên bản đồ)
4. RViz2 hiển thị Bản đồ đã lưu (/map), RobotModel, tia quét LiDAR (/scan) và quỹ đạo di chuyển
5. Auto Undock + Di chuyển robot ra vùng không gian mở trung tâm (AMCL tự động bám theo trên bản đồ)
6. Node nhận diện cử chỉ tay AI: gesture_recognition_node (publish /detected_gesture)
7. Node điều khiển hành vi robot: robot_behavior_node (subscribe /detected_gesture -> publish /cmd_vel)
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
    rviz_config = PathJoinSubstitution([pkg_practice, 'config', 'turtlebot4_nav2.rviz'])
    tb4_loc_launch = PathJoinSubstitution([pkg_tb4_nav, 'launch', 'localization.launch.py'])
    fastdds_profile = PathJoinSubstitution([pkg_practice, 'config', 'fastdds_udp_only.xml'])

    set_fastdds = SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', fastdds_profile)

    arguments = [
        DeclareLaunchArgument('namespace', default_value='', description='Robot namespace'),
        DeclareLaunchArgument(
            'launch_sim',
            default_value='true',
            choices=['true', 'false'],
            description='Khởi chạy Gazebo Simulation',
        ),
        DeclareLaunchArgument(
            'world',
            default_value='warehouse',
            description='Thế giới Gazebo (warehouse, depot, maze)',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
            description='Sử dụng simulation time',
        ),
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Đường dẫn file bản đồ map.yaml đã lưu trước đó',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            choices=['true', 'false'],
            description='Khởi chạy giao diện RViz2 hiển thị bản đồ và robot',
        ),
        DeclareLaunchArgument(
            'auto_undock',
            default_value='true',
            choices=['true', 'false'],
            description='Tự động Undock và đưa robot ra vùng rộng thoáng không vật cản',
        ),
        DeclareLaunchArgument(
            'enable_gesture_node',
            default_value='true',
            choices=['true', 'false'],
            description='Khởi chạy Node nhận diện cử chỉ tay (gesture_recognition_node)',
        ),
        DeclareLaunchArgument(
            'enable_behavior_node',
            default_value='true',
            choices=['true', 'false'],
            description='Khởi chạy Node điều khiển hành vi robot (robot_behavior_node)',
        ),
        DeclareLaunchArgument(
            'camera_id',
            default_value='0',
            description='Chỉ số Webcam laptop (/dev/videoX)',
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

    # 2. TF Relay & Mock Services (sau 3s)
    tf_relay_node = Node(
        package='turtlebot4_practice4',
        executable='tf_relay',
        name='tf_relay',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )
    delayed_tf_relay = TimerAction(period=3.0, actions=[tf_relay_node])

    # 3. Localization (Map Server + AMCL) nạp bản đồ đã lưu ở giây thứ 10 (khi robot đã spawn trên dock 0,0,0)
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

    # 4. RViz2 hiển thị bản đồ (map.yaml), RobotModel, LaserScan (/scan) và AMCL ParticleCloud (sau 10s)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_gesture_map',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    delayed_rviz = TimerAction(period=10.0, actions=[rviz_node])

    # 5. Auto Undock + Di chuyển ra khoảng trống trung tâm (sau 16s để AMCL đã khớp xong bản đồ tại dock)
    undock_node = Node(
        package='turtlebot4_practice4',
        executable='auto_undock',
        name='auto_undocker',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'move_to_open_space': True},
            {'open_space_distance': 0.90},
        ],
        condition=IfCondition(LaunchConfiguration('auto_undock')),
    )
    delayed_undock = TimerAction(period=16.0, actions=[undock_node])

    # 6. Node điều khiển hành vi TurtleBot 4: robot_behavior_node (sau 18s)
    behavior_node = Node(
        package='turtlebot4_practice4',
        executable='robot_behavior_node',
        name='robot_behavior_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(LaunchConfiguration('enable_behavior_node')),
    )
    delayed_behavior_node = TimerAction(period=18.0, actions=[behavior_node])

    # 7. Node nhận diện cử chỉ tay bằng AI: gesture_recognition_node (sau 20s)
    gesture_node = Node(
        package='turtlebot4_practice4',
        executable='gesture_recognition_node',
        name='gesture_recognition_node',
        output='screen',
        parameters=[
            {'camera_id': LaunchConfiguration('camera_id')},
            {'use_sim_time': False},
        ],
        condition=IfCondition(LaunchConfiguration('enable_gesture_node')),
    )
    delayed_gesture_node = TimerAction(period=20.0, actions=[gesture_node])

    ld = LaunchDescription(arguments)
    ld.add_action(set_fastdds)
    ld.add_action(simulation)
    ld.add_action(delayed_tf_relay)
    ld.add_action(delayed_localization)
    ld.add_action(delayed_rviz)
    ld.add_action(delayed_undock)
    ld.add_action(delayed_behavior_node)
    ld.add_action(delayed_gesture_node)
    return ld
