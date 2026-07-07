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

    uwb_driver_node = Node(
        package='uwb_dwm1001_driver',
        executable='uwb_ros2_publisher',
        name='uwb_dwm1001_driver',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyACM0',
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

    return LaunchDescription([
        uwb_driver_node,
        uwb_calibration_node,
        heading_filter_node,
        ekf_local_node,
        ekf_global_node,
        change_point_node,
    ])
