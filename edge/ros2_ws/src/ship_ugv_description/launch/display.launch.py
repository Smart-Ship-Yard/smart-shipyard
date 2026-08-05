#!/usr/bin/env python3
"""
display.launch.py
-------------------
URDF 확인용 launch. robot_state_publisher + joint_state_publisher_gui(바퀴
continuous 조인트 슬라이더용) + rviz2만 띄운다. 실물/시뮬 launch와 무관하게
"이 URDF가 의도대로 조립됐는지"만 눈으로 확인하는 용도.

사용:
  ros2 launch ship_ugv_description display.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    description_share = get_package_share_directory('ship_ugv_description')
    xacro_path = os.path.join(description_share, 'urdf', 'ship_ugv_core.urdf.xacro')
    rviz_config_path = os.path.join(description_share, 'rviz', 'urdf.rviz')
    robot_description = xacro.process_file(xacro_path).toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])
