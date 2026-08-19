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
import subprocess

from launch.actions import LogInfo
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
        parameters=[{
            # ★ 2026-08-15 실측 반영: 노드 기본값 0.01(rad^2)은 "yaw 오차 5.7도"라는
            #   주장인데, 좁은 방 앵커 기하에서 실제로는 25도 이상 틀어지는 것을
            #   여러 번 확인했다(180도 뒤집힌 경우도 있었다). 과신한 공분산은
            #   ekf_global 에서 AMCL(라이다-맵 매칭)의 정확한 yaw 를 눌러버린다.
            #   실측에 맞춰 낮춘다 — 여전히 AMCL 이 수렴하기 전이나
            #   AMCL 이 실패했을 때의 fallback 역할은 한다.
            #
            # ★ 2026-08-18 재조정 0.15 -> 1.0 (약 57도).
            #   0.15 로도 부족했다. AMCL 이 수렴해 공분산 6.5도(=아주 확신)를
            #   내는데도 EKF 가 29도나 어긋난 채로 heading 필터를 따라갔다.
            #   원인은 **업데이트 빈도**다 — heading 필터는 10 Hz 로 계속 밀어넣지만
            #   AMCL 은 로봇이 0.1 m 움직여야 한 번 갱신한다(update_min_d).
            #   정확도는 AMCL 이 높은데 횟수로 밀린다.
            #   그래서 heading 필터를 "약한 사전정보"로 낮춘다. 라이다-맵 매칭이
            #   가능한 상황에서는 그쪽이 언제나 더 정확하다.
            'yaw_variance': 1.0,
        }],
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

    # ★ yolo_depth_publisher 는 **여기서 띄우지 않는다** (2026-08-19 제거).
    #
    #   원래는 "측량 입력(/event_detection/uvd)을 yolo_depth_publisher 가 만드니
    #   같이 띄운다" 는 의도로 여기 있었다. 의도는 맞지만 실제로는 동작하지
    #   않고 있었다.
    #
    #   젯슨에는 systemd 서비스 `yolo-depth-publisher.service` 가 이미 있고,
    #   카메라(pyorbbecsdk)는 **한 프로세스만 열 수 있다.** 그래서 둘이 경쟁하면
    #   늦게 뜬 쪽이 죽는데, 늦게 뜨는 쪽이 항상 이 launch 였다:
    #       [ERROR] [yolo_depth_publisher-11]: process has died [exit code 1]
    #   런치 로그가 워낙 길어 이 한 줄은 아무도 못 봤고, "YOLO 잘 돌아가네" 하고
    #   있었지만 실제로 도는 것은 systemd 쪽이었다.
    #
    #   반대로 CPU 를 아끼려고 systemd 를 잠깐 끄면 이번엔 이쪽이 살아나면서
    #   터미널을 로그로 도배했다 (실측 120건/12초).
    #
    #   -> **systemd 가 YOLO 를 소유한다.** 부팅 시 자동으로 뜨고, 죽어도
    #      Restart=always 로 되살아나며, 로그가 journald 로 가서 이 터미널을
    #      더럽히지 않는다. 매핑 중 배 표면 측량도 그대로 동작한다.
    #
    #   대신 아래 기동 배너에서 "YOLO 가 실제로 떠 있는지" 를 확인한다.
    #   누가 서비스를 꺼놨으면 매핑 마스킹이 **조용히** 실패하기 때문이다.

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
            # 재접속 주기·로그 주기는 노드 기본값(3초 재시도 / 15초 로그)을 쓴다.
            # 예전에 로그가 시끄럽다고 이 값을 10초로 늘렸었는데, 그러면 로그가
            # 아니라 재연결 자체가 늦어져 서버가 살아난 뒤에도 최대 15초간
            # 데이터가 끊겼다. 위치 핑이 2 Hz 라 대시보드에서 눈에 띈다.
            # 지금은 노드가 재시도는 빠르게 하고 실패 로그만 묶어서 찍는다.
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
            # ★ 2026-08-18: 330 -> 1320 으로 되돌림.
            #   330 이었던 이유는 펌웨어가 A상 상승엣지만 세는 반쪽 디코딩이라
            #   쿼드러처 4배를 못 셌기 때문이다 (1320 / 4 = 330).
            #   펌웨어를 정식 x4 쿼드러처로 바꿨으므로 스펙값이 맞다.
            #   물리 치수(wheel_radius_m, track_width_m)는 그대로다 — 세는 방식만 바뀐 것.
            'ticks_per_rev': 1320,
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

    # ------------------------------------------------------------------
    # ★ 기동 배너 — 센서 장치 파일이 있는지 먼저 눈으로 알려준다 (2026-08-17 추가)
    #
    # 왜 필요한가: 라이다 USB 커넥터가 헐거워져 /dev/lidar 가 사라진 채로
    # 이 launch 를 띄웠더니, rplidar_node 만 1초 만에 조용히 죽고 나머지는
    # 정상 기동했다. 이 launch 는 노드 하나가 죽어도 전체가 안 죽으므로
    # (required 미설정 — 카메라가 없어도 주행은 되어야 하니 의도된 설계다)
    # 아무도 눈치채지 못했고, 그 상태로 13분을 매핑했지만 스캔이 없어
    # 지도가 한 장도 안 만들어졌다. align 을 부르고 나서야 알았다.
    #
    # 그래서 "없으면 크게 알린다". 죽이지는 않는다 — 장치 하나가 없어도
    # 나머지로 할 수 있는 일이 있고, 그 판단은 사람이 한다.
    DEVICES = [
        ('/dev/lidar',     'RPLIDAR',      '스캔 없음 -> 매핑·Nav2 불가'),
        ('/dev/uwb_tag',   'UWB 태그',     '위치추정 불가 (캘리브레이션 실패)'),
        ('/dev/imu',       'IMU',          'yaw 추정 열화'),
        ('/dev/wheel_mcu', 'Arduino Mega', '바퀴가 안 돈다 (/cmd_vel 소비자 없음)'),
    ]
    missing = [(p, name, impact) for p, name, impact in DEVICES if not os.path.exists(p)]

    banner = [LogInfo(msg='─── 센서 장치 확인 ' + '─' * 41)]
    for path, name, _ in DEVICES:
        mark = '✅' if os.path.exists(path) else '❌'
        banner.append(LogInfo(msg=f'  {mark} {name:<14} {path}'))
    if missing:
        banner.append(LogInfo(msg='━' * 60))
        for path, name, impact in missing:
            banner.append(LogInfo(msg=f'  ⚠️  {name} 없음 ({path}) — {impact}'))
        banner.append(LogInfo(
            msg='  ⚠️  USB 를 다시 꽂고 이 launch 를 재시작할 것. '
                '이 상태로 진행하면 조용히 실패한다'))
        banner.append(LogInfo(msg='━' * 60))
    else:
        banner.append(LogInfo(msg='  네 장치 모두 정상 — 그래도 매핑 전에 '
                                  '/scan_filtered 가 실제로 흐르는지 한 번 볼 것'))

    # ★ YOLO 는 이 launch 가 아니라 systemd 가 띄운다(위 주석 참고).
    #   그래서 "떠 있는지" 를 여기서 확인해 준다. 안 떠 있으면 매핑 중
    #   배 표면 측량이 **조용히** 실패한다 — 에러도 경고도 안 난다.
    yolo_up = subprocess.run(
        ['pgrep', '-f', 'yolo_depth_publisher'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0
    banner.append(LogInfo(msg=f"  {'✅' if yolo_up else '❌'} {'YOLO(카메라)':<14} "
                              f"systemd yolo-depth-publisher.service"))
    if not yolo_up:
        banner.append(LogInfo(msg='━' * 60))
        banner.append(LogInfo(
            msg='  ⚠️  YOLO 가 안 떠 있다 — 카메라 영상 송출과 배 표면 측량이 '
                '둘 다 조용히 실패한다'))
        banner.append(LogInfo(
            msg='  ⚠️  살리는 법:  sudo systemctl start yolo-depth-publisher'))
        banner.append(LogInfo(msg='━' * 60))
    banner.append(LogInfo(msg='─' * 60))

    return LaunchDescription(banner + [
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
        ship_survey_node,
        websocket_client_node,
        laser_static_tf_node,
        rplidar_node,
        laser_filter_node,
    ])
