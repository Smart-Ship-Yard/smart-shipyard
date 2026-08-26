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
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import subprocess
import time

from launch.actions import LogInfo
from launch_ros.actions import Node


def _check_yolo_freshness():
    """YOLO(systemd)가 현재 소스보다 오래된 코드로 돌고 있으면 재시작한다.

    ★ 왜 필요한가 (2026-08-26 실측 사고) ★
    YOLO 는 이 launch 가 아니라 systemd 가 띄운다(아래 주석 참고). 그런데
    파이썬은 프로세스를 시작할 때 코드를 한 번 읽고 끝이라, git pull +
    colcon build 를 해도 **이미 돌고 있는 YOLO 는 옛 코드 그대로**다.
    실제로 7시간 동안 팀원이 머지한 개선(클래스별 confidence, person crop)이
    하나도 안 걸린 채 시험하고 있었고, 아무도 몰랐다. 파라미터가 없어서
    ros2 param set 이 실패하고 나서야 발견했다.

    그래서 여기서 "소스 수정 시각 > 프로세스 시작 시각" 이면 재시작한다.
    - **최신이면 아무것도 안 한다** (매번 sudo 암호를 물으면 금방 안 쓰게 된다)
    - 재시작이 필요할 때만 sudo 가 암호를 묻는다
    - 실패해도 launch 를 죽이지 않는다. 크게 알리고 판단은 사람이 한다
      (장치 확인 배너와 같은 철학)

    반환: (mark, text) — 배너 한 줄로 쓸 표시와 설명.
    """
    # ★ realpath 로 심볼릭 링크를 먼저 푼다.
    #   ros2 launch 로 실행하면 __file__ 은 install/ 쪽 경로다:
    #       install/ship_ugv_localization/share/ship_ugv_localization/launch/...
    #   colcon --symlink-install 이라 그 파일은 src/ 를 가리키는 심볼릭 링크인데,
    #   abspath 는 링크를 풀지 않아 install/ 밑에서 소스를 찾다가 실패했다.
    #   (src/ 에서 직접 돌린 테스트만 해서 못 잡았다 — 2026-08-26)
    here = os.path.realpath(__file__)
    rel = os.path.join('ship_ugv_perception', 'ship_ugv_perception',
                       'yolo_depth_publisher.py')
    src = None
    d = os.path.dirname(here)
    for _ in range(6):          # launch -> 패키지 -> src -> ... 위로 훑는다
        for cand in (os.path.join(d, rel), os.path.join(d, 'src', rel)):
            if os.path.exists(cand):
                src = cand
                break
        if src:
            break
        d = os.path.dirname(d)
    if src is None:
        return '❔', '소스를 못 찾음 — 확인 생략 (경로 구조가 바뀌었는지 볼 것)'

    # 래퍼(ros2 run)가 아니라 실제 노드 프로세스를 찾는다.
    r = subprocess.run(['pgrep', '-f', 'lib/ship_ugv_perception/yolo_depth_publisher'],
                       capture_output=True, text=True)
    pids = [x for x in r.stdout.split() if x.isdigit()]
    if not pids:
        return '❌', 'YOLO 가 안 떠 있음 — sudo systemctl start yolo-depth-publisher'

    try:
        # /proc/<pid> 디렉터리의 mtime 이 곧 프로세스 시작 시각이다.
        started = os.path.getmtime('/proc/' + pids[0])
    except OSError:
        return '❔', '프로세스 시작 시각을 못 읽음 — 확인 생략'

    src_mtime = os.path.getmtime(src)
    if src_mtime <= started:
        return '✅', '소스와 실행본 일치 — systemd 구동이라 git pull 만으론 최신화 안 됨'

    fmt = '%H:%M:%S'
    old = time.strftime(fmt, time.localtime(started))
    new = time.strftime(fmt, time.localtime(src_mtime))
    print(f'\n⚠️  YOLO 가 옛 코드로 돌고 있다 (실행 {old} < 소스 {new}).')
    print('   최신 코드로 재시작한다 — 관리자 암호를 입력할 것.')
    print('   (중단하려면 Ctrl+C — 그 경우 launch 를 시작하지 않는다)')

    # ★ 성공할 때까지 다시 묻는다. sudo 는 자체적으로 3회까지 받고 실패로
    #   끝나므로, 그 위를 한 번 더 감싸 무한 재시도로 만든다. 설치할 때
    #   sudo 를 쓰는 것과 같은 감각으로 쓰라고.
    #
    # ★ 실패한 채로는 launch 를 진행하지 않는다.
    #   옛 코드로 도는 YOLO 는 "돌긴 도는데 최신 개선이 안 걸린" 상태라
    #   증상이 조용하다. 실제로 그 상태로 7시간을 시험하며 엉뚱한 원인을
    #   쫓았다. 경고만 띄우고 진행하면 그 사고가 그대로 재현되므로,
    #   여기서는 아예 시작을 막는다(장치 확인과 달리 조치가 확실하므로).
    while True:
        try:
            rc = subprocess.run(
                ['sudo', 'systemctl', 'restart', 'yolo-depth-publisher']).returncode
        except KeyboardInterrupt:
            print('\n\n중단됨 — YOLO 가 옛 코드인 채로는 시작하지 않는다.')
            print('   나중에 직접 하려면:  sudo systemctl restart yolo-depth-publisher')
            sys.exit(1)
        except Exception as e:
            print(f'\n재시작을 실행하지 못했다: {e}')
            sys.exit(1)
        if rc == 0:
            break
        print('\n암호가 틀렸거나 재시작에 실패했다. 다시 시도한다 '
              '(중단하려면 Ctrl+C).')

    return '🔄', (f'옛 코드 감지 -> 재시작함 ({old} -> 방금) — '
                  f'systemd 구동이라 git pull 만으론 최신화 안 됨')


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

    # ---- 저장된 캘리브레이션 되살리기 (2026-08-20 신설) ----
    #   ros2 launch ship_ugv_localization localization.launch.py calib:=shipyard_map_hall_v3
    #
    #   map<-uwb_frame 은 방에 고정된 값이다(UWB 앵커가 벽에 붙어 있다).
    #   그런데 노드 메모리에만 있어서, 배터리가 나가면 — 모터와 젯슨이 배터리
    #   하나를 같이 쓴다 — 같이 날아가고, 그 좌표계로 만든 맵까지 통째로
    #   못 쓰게 되어 매핑을 처음부터 다시 해야 했다. 불러오면 그럴 필요가 없다.
    #
    #   이 런치 파일은 OpaqueFunction 없이 즉시 조립하는 방식이라
    #   LaunchConfiguration 을 여기서 풀 수 없다. 그래서 argv 를 직접 본다.
    #   (사람이 직접 실행하는 파일이고, 다른 런치가 include 하지 않는다)
    calib_arg = next((a.split(':=', 1)[1] for a in sys.argv
                      if a.startswith('calib:=')), '')
    calib_file = ''
    if calib_arg:
        calib_file = calib_arg if calib_arg.endswith('.json') else os.path.join(
            get_package_share_directory('ship_ugv_navigation'),
            'maps', 'calibration_records', f'calib_{calib_arg}.json')

        # ★ 파일이 없으면 **여기서 멈춘다** (2026-08-27 실측 사고).
        #   맵 이름 끝에 점 하나가 더 붙었을 뿐인데
        #       calib:=shipyard_map_hall_v6.
        #   에러도 경고도 없이 그냥 캘리브레이션 없이 떴다. map<-uwb_frame
        #   이 항등변환(0,0,0)이 되어 UWB 원시 좌표가 그대로 map 좌표로
        #   쓰였고, 로봇이 자기를 맵 밖 4m 지점에 있다고 믿어 제자리만
        #   돌았다(Nav2 recovery 7회). 로봇이 눈에 띄게 이상해서 알아챘지
        #   조금만 어긋났으면 "오늘따라 부정확하네" 하고 넘어갔을 것이다.
        #
        #   조용히 잘못된 상태로 도는 것보다 안 뜨는 게 낫다. 장치 확인은
        #   경고만 하고 진행하지만(없는 장치로 할 수 있는 일이 남아 있다),
        #   이건 조치가 명확하고 잘못 진행하면 주행이 통째로 망가진다.
        if not os.path.exists(calib_file):
            d = os.path.dirname(calib_file)
            avail = sorted(f[len('calib_'):-len('.json')]
                           for f in os.listdir(d)
                           if f.startswith('calib_') and f.endswith('.json')) \
                    if os.path.isdir(d) else []
            print('\n' + '━' * 60)
            print('  ❌ 캘리브레이션 파일이 없다 — 시작하지 않는다')
            print(f'     찾은 이름 : calib_{calib_arg}.json')
            print(f'     찾은 경로 : {d}')
            near = [a for a in avail if a.strip('. ') == calib_arg.strip('. ')]
            if near:
                print(f'\n  💡 이름이 거의 같은 것이 있다 — 오타로 보인다:')
                print(f'        calib:={calib_arg}   ->   calib:={near[0]}')
            elif avail:
                print('\n  쓸 수 있는 이름:')
                for a in avail:
                    print(f'        calib:={a}')
            else:
                print('\n  저장된 캘리브레이션이 하나도 없다. 먼저 캘리브레이션을 할 것:')
                print('        ros2 service call /uwb_map_calibration/calibrate '
                      'std_srvs/srv/Trigger')
            print('━' * 60 + '\n')
            sys.exit(1)

    uwb_calibration_node = Node(
        package='uwb_map_calibration',
        executable='calibration_node',
        name='uwb_map_calibration',
        output='screen',
        parameters=[{'load_calibration_file': calib_file}],
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

    # ---- YOLO 배 측량을 켤지 (2026-08-20: 기본 끔) ----
    #   ros2 launch ship_ugv_localization localization.launch.py survey:=true
    #
    #   왜 껐나: 이 노드가 내놓는 배 중심·방향·크기가 못 쓸 정도로 부정확했다.
    #     실측 0.80 x 0.17 m 인 배를 1.68 x 0.35 m 로 쟀다(2026-08-20).
    #     방향은 노이즈고, 중심도 같은 배를 두 번 재서 크게 달랐다.
    #   왜 문제인가: 이 값 하나가 **두 곳의 기준 좌표계**가 된다.
    #     ① Nav2 keepout 마스크 위치  ② 프론트엔드 화면 전체
    #        (ShipyardTwinDashboard.jsx 의 mapXYToShipLocalMeters 가 배 pose 를
    #         기준으로 로봇·이벤트·블록을 전부 변환한다)
    #     즉 배 pose 가 틀리면 자율주행도 화면도 같이 틀어진다.
    #   대신: 매핑할 때 사람이 줄자로 한 번 재서 넣는다.
    #     finalize_map.py <맵이름> --center X Y
    #     방향은 "캘리브레이션 때 로봇을 배와 나란히" 약속으로 0도 고정.
    #   ※ 이벤트 좌표(/event_detection/map_point)와 block_level 은 그대로 YOLO 를
    #     쓴다. 그쪽은 1회 관측이라 오차가 누적되지 않고, 대안도 없다.
    survey_on = any(a in ('survey:=true', 'survey:=True', 'survey:=1')
                    for a in sys.argv)

    # ---- 실측 배 중심을 켜질 때마다 서버로 보낸다 (2026-08-20 신설) ----
    #   매핑한 날에는 publish_ship_pose.py 를 사람이 치지만, 며칠 뒤에 맵만
    #   불러와 자율주행을 돌리는 날에는 아무도 안 친다. 그러면 프론트엔드
    #   화면에 배 위치가 안 가고, 그 화면은 배 pose 를 기준 좌표계로 쓰므로
    #   로봇도 이벤트도 전부 어긋난 자리에 그려진다.
    #   ※ 이 파일은 install/ 에서도 src 로의 심볼릭 링크라, realpath 로
    #     소스 트리를 되짚으면 측정값 파일을 확실히 찾을 수 있다.
    #     (share 경유로 찾으면 colcon build 를 해야만 보인다)
    measured_file = os.path.normpath(os.path.join(
        os.path.dirname(os.path.realpath(__file__)),
        '..', '..', 'ship_ugv_navigation', 'config',
        'ship_center_measured.json'))

    ship_pose_pub_node = Node(
        package='ship_ugv_navigation',
        executable='ship_pose_publisher',
        name='ship_pose_publisher',
        output='screen',
        parameters=[{'measured_file': measured_file}],
    )
    banner_survey = ('켬 — YOLO 로 배를 측량한다 (부정확할 수 있음)' if survey_on
                     else '끔 — 배 위치는 finalize_map.py --center X Y 로 실측해 넣는다')

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

    # ★ 장치보다 먼저 — YOLO 가 최신 코드로 돌고 있는지 (필요하면 재시작)
    yolo_mark, yolo_text = _check_yolo_freshness()

    banner = [LogInfo(msg='─── 센서 장치 확인 ' + '─' * 41)]
    banner.append(LogInfo(msg=f"  {yolo_mark} {'YOLO 코드':<14} {yolo_text}"))
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
    banner.append(LogInfo(msg=f"  {'✅' if survey_on else '⏸️'} {'배 측량':<14} "
                              f"{banner_survey}"))
    banner.append(LogInfo(
        msg=f"  {'✅' if os.path.exists(measured_file) else '❌'} "
            f"{'배 중심 실측값':<11} "
            + ('있음 — 켜지면 자동으로 서버에 보낸다'
               if os.path.exists(measured_file)
               else '없음 — 프론트엔드에 배 위치가 안 간다. '
                    'scripts/measure_ship_center.py 로 잴 것')))
    banner.append(LogInfo(
        msg=f"  {'✅' if calib_file else '⏸️'} {'캘리브 불러오기':<12} "
            + (os.path.basename(calib_file) if calib_file
               else '끔 — 새로 잰다 (되살리려면 calib:=<맵이름>)')))
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
        *( [ship_survey_node] if survey_on else [ship_pose_pub_node] ),
        websocket_client_node,
        laser_static_tf_node,
        rplidar_node,
        laser_filter_node,
    ])
