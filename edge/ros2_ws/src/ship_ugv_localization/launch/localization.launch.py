#!/usr/bin/env python3
"""
ship_ugv_localization/launch/localization.launch.py
-----------------------------------------------------
전체 로컬라이제이션 스택 기동:
  uwb_dwm1001_driver -> uwb_map_calibration -> heading_complementary_filter
  -> ekf_local -> ekf_global

주의: 센서 드라이버(엔코더 /wheel/odom, IMU /imu/data)는 아직 하드웨어 미조립으로
이 launch에 포함하지 않았다. 실제 로봇 연결 후 별도 드라이버 launch를 추가할 것.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    localization_share = get_package_share_directory('ship_ugv_localization')
    ekf_local_yaml = os.path.join(localization_share, 'config', 'ekf_local.yaml')
    ekf_global_yaml = os.path.join(localization_share, 'config', 'ekf_global.yaml')
    laser_filter_yaml = os.path.join(localization_share, 'config', 'laser_filter.yaml')

    uwb_driver_node = Node(
        package='uwb_dwm1001_driver',
        executable='uwb_ros2_publisher',
        name='uwb_dwm1001_driver',
        output='screen',
        parameters=[{
            'serial_port': '/dev/uwb_tag',   
            'baud_rate': 115200,
            'uwb_frame_id': 'uwb_frame',
        }],
    )

    uwb_calibration_node = Node(
        package='uwb_map_calibration',
        executable='calibration_node',
        name='uwb_map_calibration',
        output='screen',
    )

    heading_filter_node = Node(
        package='heading_complementary_filter',
        executable='complementary_filter_node',
        name='heading_complementary_filter',
        output='screen',
    )

    ekf_local_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local',
        output='screen',
        parameters=[ekf_local_yaml],
        remappings=[('odometry/filtered', '/odometry/local')],
    )

    ekf_global_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global',
        output='screen',
        parameters=[ekf_global_yaml],
        remappings=[('odometry/filtered', '/odometry/global')],
    )

    change_point_node = Node(
        package='ship_ugv_perception',
        executable='change_point',
        name='change_point_detector',
        output='screen',
    )

    im10a_yaml = os.path.join(
        get_package_share_directory('witmotion_ros'),
        'config', 'im10a.yml')

    imu_driver_node = Node(
        package='witmotion_ros',
        executable='witmotion_ros_node',
        name='witmotion',
        output='screen',
        parameters=[im10a_yaml],
    )

    imu_axis_correction_node = Node(
        package='heading_complementary_filter',
        executable='imu_axis_correction_node',
        name='imu_axis_correction',
        output='screen',
    )

    imu_static_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_link_to_imu_tf',
    output='screen',
    # 실측(2026-07-29): base_link 원점 기준 상방 0.055m (x,y는 중앙)
    arguments=['0', '0', '0.055', '0', '0', '0', 'base_link', 'imu'],
    )
    
    laser_static_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_link_to_laser_tf',
    output='screen',
    # 실측(2026-07-29): base_link 원점(바퀴축 중점,지면) 기준 전방 0.195m, 상방 0.2m
    arguments=['0.195', '0', '0.2', '3.14159', '0', '0', 'base_link', 'laser'],
    )
    
    wheel_odom_node = Node(
        package='wheel_odom_bridge',
        executable='wheel_odom_node',
        name='wheel_odom_bridge',
        output='screen',
        parameters=[{
            'serial_port': '/dev/wheel_mcu',
            'track_width_m': 0.22568,
            'wheel_radius_m': 0.0308,
            'ticks_per_rev': 330,   # 1320 -> 330으로 변경 (JGB37-520 실제 CPR 재검증)
            'right_trim': 0.98,   # 왼쪽으로 휘니 오른쪽을 살짝 줄여서 시작
        }],
    )

    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/lidar',       # udev 고정 심볼릭 링크 사용
            'serial_baudrate': 115200,
            'frame_id': 'laser',                # base_link_to_laser_tf와 일치
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Sensitivity',
        }],
    )

    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='laser_filter_chain',
        output='screen',
        parameters=[laser_filter_yaml],
        remappings=[
            ('scan', '/scan'),            # RPLIDAR 원본 입력
            ('scan_filtered', '/scan_filtered'),  # 필터링된 출력
        ],
    )

    return LaunchDescription([
        uwb_driver_node,
        uwb_calibration_node,
        imu_static_tf_node,
        imu_driver_node,
        imu_axis_correction_node,
        wheel_odom_node,
        heading_filter_node,
        ekf_local_node,
        ekf_global_node,
        change_point_node,
        laser_static_tf_node,
        rplidar_node,
        laser_filter_node,
    ])
