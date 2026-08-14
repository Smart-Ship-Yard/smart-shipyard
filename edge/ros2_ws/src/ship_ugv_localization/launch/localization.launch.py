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

    # ★ 배 측량은 매핑 랩과 같은 주행에서 이뤄져야 한다.
    #   Nav2 순찰을 돌리려면 배 중심이 이미 있어야 하는데, 중심을 구하려고
    #   도는 것이라 Nav2를 켠 뒤에 측량하면 닭-달걀이 되기 때문.
    #   mapping.launch.py가 이 launch를 전제로 돌므로 여기 두면 매핑 시 항상 켜진다.
    #   측량 입력(/event_detection/uvd)을 yolo_depth_publisher가 만들므로 같이 띄운다.
    yolo_depth_publisher_node = Node(
        package='ship_ugv_perception',
        executable='yolo_depth_publisher',
        name='yolo_depth_publisher',
        output='screen',
    )

    ship_survey_node = Node(
        package='ship_ugv_perception',
        executable='ship_survey_node',
        name='ship_survey_node',
        output='screen',
        parameters=[{
            # 배를 반대 방향으로 놓았으면 이 값만 3.14159로 바꿀 것.
            # 최소외접사각형의 180도 모호성(앞뒤 구분 불가)만 푸는 용도라
            # 대충 실측한 값이면 충분하다.
            'yaw_hint_rad': 0.0,
        }],
    )

    # ★ ship_survey_node의 재측량 트리거(/block_level/confirmed)를 이 노드가
    #   발행하므로, 얘가 안 떠 있으면 조립 단계가 바뀌어도 재측량이 영영 안 된다.
    #   그래서 측량 노드와 같은 launch에 둔다.
    websocket_client_node = Node(
        package='ship_ugv_perception',
        executable='websocket_client',
        name='websocket_client',
        output='screen',
        parameters=[{
            # ★ 서버 IP는 같은 공유기 내부 IP라 바뀔 수 있다 (edge/docs/설치가이드.md 참고).
            #   서버가 안 붙으면 제일 먼저 여기부터 확인할 것.
            'server_ws_url': 'ws://192.168.0.5:8000/ws/jetson',
            # 서버가 아직 안 떠 있으면 이 주기로 재접속을 계속 시도하는데,
            # 기본 5초면 매핑 중에 로그가 시끄러워서 10초로 늘렸다.
            # 대가: 서버가 켜진 뒤 붙기까지 최대 10초 걸린다 (위치 핑은 0.5초라 무관).
            'reconnect_interval_s': 10.0,
        }],
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
        yolo_depth_publisher_node,
        ship_survey_node,
        websocket_client_node,
        laser_static_tf_node,
        rplidar_node,
        laser_filter_node,
    ])
