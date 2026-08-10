#!/usr/bin/env python3
"""
motion_control.launch.py
-------------------------
모션 컨트롤러만 단독으로 띄우는 launch.

전제조건: localization.launch.py가 이미 실행 중이어야 한다.
  (/odometry/local 피드백과 wheel_odom_bridge의 /cmd_vel 수신이 필요하므로)

사용:
  터미널 1: ros2 launch ship_ugv_localization localization.launch.py
  터미널 2: ros2 launch ship_ugv_motion_control motion_control.launch.py
  터미널 3: ros2 topic pub --once /motion/move_distance std_msgs/msg/Float64 "{data: 0.5}"
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    motion_controller_node = Node(
        package='ship_ugv_motion_control',
        executable='motion_controller_node',
        name='motion_controller',
        output='screen',
        parameters=[{
            # 안전을 위해 처음에는 느린 속도로 시작할 것.
            # 실차에서 잘 도는 것을 확인한 뒤 조금씩 올리는 순서를 권장.
            'max_linear_speed': 0.15,
            'max_angular_speed': 0.6,
            'min_linear_speed': 0.04,
            'min_angular_speed': 0.15,
            'distance_tolerance_m': 0.02,
            'angle_tolerance_deg': 2.0,
            'kp_heading_hold': 3.0,
        }],
    )

    return LaunchDescription([
        motion_controller_node,
    ])
