#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Launch file khởi chạy toàn bộ ngăn xếp Autonomous Navigation (Nav2) cho TurtleBot 4
Phần 4: Navigation với Nav2
Bao gồm:
- Gazebo simulation (TurtleBot 4 trong warehouse)
- Map Server & AMCL (Localization với bản đồ đã tạo)
- Nav2 Controller, Planner, Recoveries, BT Navigator
- RViz2 với công cụ Nav2 Goal
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, SetEnvironmentVariable, TimerAction
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

    # FastDDS chỉ dùng UDP (tắt Shared Memory). Nếu dùng SHM, sau một thời gian AMCL ngừng
    # nhận /scan hoặc TF map->odom không tới được controller_server -> lỗi
    # "Transform data too old when converting from map to odom" và robot quay vòng (recovery spin).
    fastdds_profile = PathJoinSubstitution([pkg_practice, 'config', 'fastdds_udp_only.xml'])
    set_fastdds = SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', fastdds_profile)

    arguments = [
        DeclareLaunchArgument('namespace', default_value='',
                              description='Robot namespace'),
        DeclareLaunchArgument('launch_sim', default_value='true',
                              choices=['true', 'false'],
                              description='Khởi chạy kèm Gazebo simulation'),
        DeclareLaunchArgument('world', default_value='warehouse',
                              description='Gazebo world'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              choices=['true', 'false'],
                              description='Sử dụng simulation time'),
        DeclareLaunchArgument('map', default_value=default_map,
                              description='Đường dẫn file map.yaml'),
        DeclareLaunchArgument('rviz', default_value='true',
                              choices=['true', 'false'],
                              description='Mở RViz2 hiển thị Nav2 Goal, Costmaps, Path'),
        DeclareLaunchArgument('auto_navigate', default_value='true',
                              choices=['true', 'false'],
                              description='Tự động gửi 03 Navigation Goals khi Nav2 sẵn sàng'),
        DeclareLaunchArgument('spawn_objects', default_value='true',
                              choices=['true', 'false'],
                              description='Tự động thêm mô hình người, ghế, bàn quanh các điểm Goal'),
        DeclareLaunchArgument('enable_yolo', default_value='true',
                              choices=['true', 'false'],
                              description='Khởi chạy Node nhận diện vật thể YOLOv8 từ Camera OAK-D'),
    ]

    # 1. Gazebo Simulation (GroupAction scoped=True để tránh rviz argument ghi đè ra ngoài)
    simulation = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([sim_launch]),
                launch_arguments=[
                    ('world', LaunchConfiguration('world')),
                    ('namespace', LaunchConfiguration('namespace')),
                    ('rviz', 'false')
                ]
            )
        ],
        scoped=True,
        condition=IfCondition(LaunchConfiguration('launch_sim'))
    )

    # 1b. Thêm mô hình người, ghế, bàn quanh các điểm Goal ở giây thứ 8
    spawn_objects_node = Node(
        package='turtlebot4_practice4',
        executable='spawn_yolo_objects',
        name='spawn_yolo_objects',
        output='screen',
        parameters=[{'world': LaunchConfiguration('world')}],
        condition=IfCondition(LaunchConfiguration('spawn_objects'))
    )
    delayed_spawn_objects = TimerAction(
        period=8.0,
        actions=[spawn_objects_node]
    )

    # 2. TF Relay & Helper services (chạy sớm sau 3s)
    tf_relay_node = Node(
        package='turtlebot4_practice4',
        executable='tf_relay',
        name='tf_relay',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    delayed_tf_relay = TimerAction(
        period=3.0,
        actions=[tf_relay_node]
    )

    # 3. Localization (AMCL + Map Server) - khởi chạy ở giây thứ 10 khi robot đã spawn xong trên dock (0, 0, 0)
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([tb4_loc_launch]),
        launch_arguments=[
            ('namespace', LaunchConfiguration('namespace')),
            ('map', LaunchConfiguration('map')),
            ('params', loc_params),
            ('use_sim_time', LaunchConfiguration('use_sim_time'))
        ]
    )
    delayed_localization = TimerAction(
        period=10.0,
        actions=[localization]
    )

    # 4. RViz2 Navigation View - mở đồng thời cùng localization ở giây thứ 10 (hiển thị robot trên dock)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_nav2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    delayed_rviz = TimerAction(
        period=10.0,
        actions=[rviz_node]
    )

    # 5. Tự động Undock robot ở giây thứ 16 (khi AMCL và RViz đã theo dõi, robot lùi ra và AMCL track trực tiếp)
    undock_node = Node(
        package='turtlebot4_practice4',
        executable='auto_undock',
        name='auto_undocker',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    delayed_undock = TimerAction(
        period=16.0,
        actions=[undock_node]
    )

    # 5b. Khởi chạy Node YOLOv8 Object Detection ở giây thứ 20
    yolo_node = Node(
        package='turtlebot4_practice4',
        executable='yolo_detector',
        name='yolo_detector_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('enable_yolo'))
    )
    delayed_yolo = TimerAction(
        period=20.0,
        actions=[yolo_node]
    )

    # 6. Nav2 Navigation (Planner, Controller, Recoveries, BT Navigator) - chờ 32s khi robot đã undock xong hoàn toàn
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([tb4_nav_launch]),
        launch_arguments=[
            ('namespace', LaunchConfiguration('namespace')),
            ('params_file', nav2_params),
            ('use_sim_time', LaunchConfiguration('use_sim_time'))
        ]
    )
    delayed_navigation = TimerAction(
        period=32.0,
        actions=[navigation]
    )

    # 7. Tự động gửi 3 Navigation Goals khi Nav2 đã sẵn sàng (chờ 45s)
    nav_goals_node = Node(
        package='turtlebot4_practice4',
        executable='auto_nav_goals',
        name='auto_nav_goals',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('auto_navigate'))
    )
    delayed_nav_goals = TimerAction(
        period=45.0,
        actions=[nav_goals_node]
    )

    ld = LaunchDescription(arguments)
    ld.add_action(set_fastdds)
    ld.add_action(simulation)
    ld.add_action(delayed_spawn_objects)
    ld.add_action(delayed_undock)
    ld.add_action(delayed_tf_relay)
    ld.add_action(delayed_localization)
    ld.add_action(delayed_rviz)
    ld.add_action(delayed_yolo)
    ld.add_action(delayed_navigation)
    ld.add_action(delayed_nav_goals)
    return ld

