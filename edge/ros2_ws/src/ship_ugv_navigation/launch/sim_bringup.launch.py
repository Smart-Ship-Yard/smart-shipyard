#!/usr/bin/env python3
"""
sim_bringup.launch.py — 시뮬 환경 기동 (Step 3)
================================================
Gazebo를 띄우고 로봇을 스폰한 뒤, **실물과 동일한 TF 트리**를 세운다.
Nav2는 여기 포함되지 않는다 (navigation.launch.py가 따로 담당).

    ros2 launch ship_ugv_navigation sim_bringup.launch.py

띄우는 것
---------
    gzserver + gzclient        Gazebo (worlds/demo_room.world)
    robot_state_publisher      URDF -> base_link 하위 TF 전부
    spawn_entity               로봇을 world에 배치
    ekf_local                  odom -> base_link   ★ 슬램 담당자 설정 그대로 사용
    fake_global_localization   map -> odom         ★ 시뮬 전용 (실물은 ekf_global)
    rviz2                      (선택) use_rviz:=true

완성되는 TF 트리 — 실물과 구조가 동일하다
-----------------------------------------
    map
     └─ odom                 fake_global_localization (실물: ekf_global)
         └─ base_link        ekf_local                (실물과 동일한 노드·설정)
             ├─ chassis_link · left/right_wheel · caster · imu · laser
             └─ (robot_state_publisher가 URDF 보고 발행)

    ※ 실물에서는 robot_state_publisher 대신 static_transform_publisher 2개가
      base_link->imu, base_link->laser를 발행한다. 시뮬은 URDF 전체를 써야
      Gazebo가 로봇 모양을 알 수 있으므로 robot_state_publisher를 쓴다.
      Nav2가 요구하는 프레임(base_link, laser)은 어느 쪽이든 동일하게 존재한다.

실물과의 차이 (실물 이식 시 이 부분만 갈아끼운다)
--------------------------------------------------
    | 항목            | 시뮬                        | 실물                          |
    |-----------------|-----------------------------|-------------------------------|
    | /cmd_vel 소비   | gazebo diff_drive 플러그인  | wheel_odom_bridge (PR #14)    |
    | /wheel/odom     | 동 플러그인                 | 동 노드 (아두이노 엔코더)     |
    | /imu/data       | gazebo imu 플러그인         | witmotion_ros                 |
    | /scan           | gazebo ray 플러그인         | rplidar_ros -> /scan_filtered |
    | map->odom       | fake_global_localization    | ekf_global (UWB 융합)         |
    | odom->base_link | ekf_local                   | ekf_local  ← 동일             |

인자
----
    world        : world 파일 경로 (기본 demo_room.world)
    use_rviz     : RViz 동시 실행 (기본 false)
    x, y, yaw    : 로봇 스폰 위치. 배와 겹치지 않게 기본값은 순찰 반지름 밖
    gui          : Gazebo GUI 표시 (기본 true. false면 헤드리스)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():
    nav_share = get_package_share_directory('ship_ugv_navigation')
    desc_share = get_package_share_directory('ship_ugv_description')
    loc_share = get_package_share_directory('ship_ugv_localization')
    gazebo_share = get_package_share_directory('gazebo_ros')

    # ---- 인자 선언 -------------------------------------------------
    args = [
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(nav_share, 'worlds', 'demo_room.world'),
            description='Gazebo world 파일 경로'),
        DeclareLaunchArgument('use_rviz', default_value='false',
                              description='RViz 동시 실행 여부'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Gazebo GUI 표시 (false면 헤드리스)'),
        # 스폰 위치: demo_room은 원점에 배가 있으므로 옆으로 비켜서 놓는다.
        DeclareLaunchArgument('x', default_value='0.9'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='1.5708'),  # +y 방향(반시계 접선)
    ]

    # ---- URDF 처리 -------------------------------------------------
    # 시뮬 전용 xacro가 core를 include한다. core는 수정하지 않는다.
    xacro_path = os.path.join(desc_share, 'urdf', 'ship_ugv_gazebo.xacro')
    robot_description = xacro.process_file(xacro_path).toxml()

    # ---- Gazebo ----------------------------------------------------
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gzserver.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            # ROS 연동 플러그인. 없으면 spawn_entity 서비스가 안 뜬다.
            'init': 'true', 'factory': 'true', 'force_system': 'false',
            # ★ /clock 발행률을 100 Hz로 올린다.
            #   기본 10 Hz면 use_sim_time을 쓰는 모든 노드의 타이머가 10 Hz로
            #   잘려 ekf_local(50 Hz 설정)이 실제로 10 Hz로만 돈다(실측).
            #   상세 이유는 config/gazebo_params.yaml 주석 참고.
            'params_file': os.path.join(nav_share, 'config', 'gazebo_params.yaml'),
        }.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    # ---- 로봇 상태 발행 + 스폰 --------------------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_ship_ugv',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'ship_ugv',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', '0.02',                      # 바닥에 살짝 띄워 스폰 (끼임 방지)
            '-Y', LaunchConfiguration('yaw'),
        ],
    )

    # ---- 로컬 오도메트리 (실물과 완전히 동일) ------------------------
    # 슬램 담당자의 ekf_local.yaml을 그대로 쓴다. use_sim_time만 덮어쓴다.
    # 이래야 시뮬에서 맞춘 것이 실물에서도 그대로 성립한다.
    ekf_local = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local',
        output='screen',
        parameters=[
            os.path.join(loc_share, 'config', 'ekf_local.yaml'),
            {
                'use_sim_time': True,

                # ★★ 임시 오버라이드 — 슬램 담당자 확인 후 제거할 것 ★★
                #
                # ekf_local.yaml 원본은 odom0_config의 vy가 false다. 그런데
                # 차동구동 로봇은 옆으로 이동할 수 없으므로(비홀로노믹 제약)
                # vy는 항상 0이며, 이를 필터에 알려주지 않으면 프로세스 노이즈가
                # vy 상태를 흔들고 그것이 계속 적분되어 odom 프레임이 옆으로
                # 무한정 흘러간다. robot_localization의 대표적인 함정이다.
                #
                # 실측 (2026-08-07, 이 시뮬):
                #   입력 /wheel/odom      vy = 0.0        (정상)
                #   출력 /odometry/local  vy = -0.043 m/s (정지 상태인데도)
                #   결과 odom->base_link 의 y가 3 m까지 밀림
                #   증상 RViz에서 로봇 모델과 라이다 스캔이 멀찍이 떨어져 보이고
                #        스캔이 로봇을 따라 돎 (map->odom이 급변하기 때문)
                #
                # ★ 이것은 시뮬 전용 문제다 (2026-08-07 실물 검증 완료)
                #   실물에서 같은 현상을 재봤으나 나타나지 않았다.
                #     정지 1~2분      -> odom->base_link (0.000, 0.000). 안 밀림
                #     제자리 360도 x2 -> 한 바퀴당 6~8 mm 누적 (정상 범위)
                #
                #   차이의 원인: 프로세스 노이즈는 불확실성(분산)을 키울 뿐
                #   추정값(평균)을 움직이지 않는다. vy가 0에서 벗어나려면 다른
                #   상태와의 상관을 통해 밀려야 하고, 그 상관은 로봇이 실제로
                #   회전할 때 생긴다. 시뮬 로봇은 접촉 불안정으로 계속 미세하게
                #   흔들려(초당 1.6 mm + 롤 0.02 rad/s) vy가 계속 여기됐지만,
                #   실물은 바닥에 안정적으로 서 있어 그 요인이 없다.
                #
                #   -> ekf_local.yaml 원본은 수정하지 않는다. 이 오버라이드는
                #      Gazebo 물리 아티팩트를 막기 위한 시뮬 전용 조치다.
                #
                #              x      y      z
                #              roll   pitch  yaw
                #              vx     vy     vz
                #              vroll  vpitch vyaw
                #              ax     ay     az
                'odom0_config': [False, False, False,
                                 False, False, False,
                                 True,  True,  False,   # ← vy를 True로
                                 False, False, True,
                                 False, False, False],
            },
        ],
        remappings=[('odometry/filtered', '/odometry/local')],
    )

    # ---- 글로벌 위치 (시뮬 전용) -------------------------------------
    # 실물의 ekf_global 자리. Gazebo 참값 -> map->odom TF.
    fake_global = Node(
        package='ship_ugv_navigation',
        executable='fake_global_localization',
        name='fake_global_localization',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ---- RViz (선택) -------------------------------------------------
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', PathJoinSubstitution(
            [FindPackageShare('ship_ugv_navigation'), 'rviz', 'sim.rviz'])],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription(args + [
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_entity,
        ekf_local,
        fake_global,
        rviz,
    ])
