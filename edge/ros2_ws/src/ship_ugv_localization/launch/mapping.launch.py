#!/usr/bin/env python3
"""
mapping.launch.py
-------------------
slam_toolbox를 매핑 모드로 띄우는 launch 파일.

★ 전제조건 ★
  localization.launch.py가 이미 실행 중이어야 한다
  (LiDAR, /scan_filtered, EKF, TF 트리 전부 이걸로 완성됨).
  이 launch는 그 위에 SLAM만 추가로 얹는다.

★ 이 launch가 발행하는 것 - 딱 하나뿐 ★
  /slam_toolbox/pose (geometry_msgs/PoseWithCovarianceStamped)
    slam_map 프레임 기준 로봇 위치. slam_map_alignment 노드가 이걸 구독해서
    map(UWB 절대좌표) <-> slam_map 사이의 관계를 계산하고 그 결과를
    map -> slam_map 정적 TF로 발행한다.

  이 launch는 TF를 전혀 발행하지 않는다 (slam_toolbox_mapping.yaml의
  publish_tf: false 때문). TF의 최종 권위자는 항상 EKF뿐이라는 프로젝트
  원칙을 그대로 따른다.

★ 사용 절차 ★
  터미널 1: ros2 launch ship_ugv_localization localization.launch.py
  터미널 2: ros2 launch ship_ugv_localization mapping.launch.py
  터미널 3: motion_controller로 로봇을 돌아다니게 하며 지도 작성
            (또는 손으로 밀거나, 나중에 Nav2 탐색 알고리즘 연결)
  다 돌았으면:
    ros2 service call /slam_map_alignment/align std_srvs/srv/Trigger
    ros2 run tf2_ros tf2_echo map slam_map   (정합 결과 확인)

  지도 저장(나중에 재사용하려면):
    ros2 run nav2_map_server map_saver_cli -f ~/my_map
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    localization_share = get_package_share_directory('ship_ugv_localization')
    slam_yaml = os.path.join(localization_share, 'config', 'slam_toolbox_mapping.yaml')

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_yaml],
        # slam_toolbox는 기본적으로 상대경로 토픽 'pose'로 발행한다.
        # slam_map_alignment이 기대하는 절대경로 이름으로 맞춰준다.
        remappings=[
            ('pose', '/slam_toolbox/pose'),
        ],
    )

    slam_map_alignment_node = Node(
        package='slam_map_alignment',
        executable='slam_map_alignment_node',
        name='slam_map_alignment',
        output='screen',
    )

    return LaunchDescription([
        slam_toolbox_node,
        slam_map_alignment_node,
    ])
