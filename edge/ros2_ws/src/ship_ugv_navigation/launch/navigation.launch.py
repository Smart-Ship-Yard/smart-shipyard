#!/usr/bin/env python3
"""
navigation.launch.py — Nav2 자율주행 스택 기동 (Step 5)
========================================================
시뮬·실물 **공용**이다. 설정 파일을 손으로 고칠 일이 없도록,
장소에 따라 달라지는 것은 전부 space 인자 하나로 묶어두었다.

    # 시뮬 (노트북) — sim_bringup.launch.py 를 먼저 띄운 뒤
    ros2 launch ship_ugv_navigation navigation.launch.py \
        use_sim_time:=true space:=wide map:=demo_room use_rviz:=true

    # 내 방 맵으로 시뮬
    ros2 launch ship_ugv_navigation navigation.launch.py \
        use_sim_time:=true space:=narrow map:=shipyard_map_JG_room_v2 use_rviz:=true

    # 실물 (젯슨) — localization.launch.py 를 켜둔 상태에서
    ros2 launch ship_ugv_navigation navigation.launch.py \
        space:=wide map:=shipyard_map_JG_room_v2

★ space 인자가 무엇을 바꾸나 — 4가지가 한 번에 바뀐다
------------------------------------------------------
    항목                       narrow      wide      어디에서
    ------------------------------------------------------------------
    local_costmap 크기         2 x 2 m     5 x 5 m   space_*.yaml
    local  inflation_radius    0.10 m      0.25 m    space_*.yaml
    global inflation_radius    0.10 m      0.25 m    space_*.yaml
    복구 행동 spin             쓰지 않음   사용      navigate_*.xml

    설정 파일 구조:
        config/nav2_params.yaml    공통. 장소와 무관한 값 전부
        config/space_narrow.yaml   위에 겹쳐 로드 (나중 파일이 이긴다)
        config/space_wide.yaml     "

★ 맵은 패키지 안에 들어 있다 — 경로를 손으로 적지 않는다
---------------------------------------------------------
    맵 파일은 ship_ugv_navigation/maps/ 에 커밋되어 있고 colcon build 시
    패키지 share 디렉토리로 설치된다. 노트북이든 젯슨이든 경로가 같다.
    map 인자에는 **이름만** 넣는다 (확장자·경로 불필요).

        map:=demo_room         -> <pkg share>/maps/demo_room.yaml
        map:=shipyard_map_JG_room_v2   -> <pkg share>/maps/shipyard_map_JG_room_v2.yaml

    패키지 밖의 맵을 쓰려면 .yaml 로 끝나는 전체 경로를 주면 된다.

    ⚠️ 젯슨 전제: ship_ugv_navigation 패키지는 아직 main 에 없다.
       이 브랜치를 머지한 뒤 젯슨에서 pull + colcon build 해야 맵이 생긴다.

띄우는 것 (전부 lifecycle 노드)
-------------------------------
    map_server          저장된 맵을 /map 으로 발행
    controller_server   경로 추종 (RPP) -> /cmd_vel_nav
    planner_server      전역 경로 계획 (NavFn)
    behavior_server     복구 행동 (spin, backup, wait 등록)
    bt_navigator        행동트리 실행 (space 가 고른 xml)
    velocity_smoother   /cmd_vel_nav -> 가속 제한 -> /cmd_vel
    lifecycle_manager   위 6개를 순서대로 configure/activate

이 launch 가 제공하지 않는 것 (먼저 떠 있어야 한다)
----------------------------------------------------
    map->odom TF        시뮬 fake_global_localization / 실물 ekf_global
    odom->base_link TF  ekf_local (양쪽 공통)
    base_link->laser TF 시뮬 robot_state_publisher / 실물 static_transform_publisher
    /scan               시뮬 gazebo ray / 실물 rplidar + laser_filters
    /cmd_vel 소비자     시뮬 gazebo diff_drive / 실물 wheel_odom_bridge

★ /cmd_vel 경로 — velocity_smoother 를 반드시 거치게 한다
----------------------------------------------------------
    controller_server --(/cmd_vel_nav)--> velocity_smoother --(/cmd_vel)--> 로봇

    Nav2 기본은 controller_server 가 /cmd_vel 을 직접 쏘지만, 그러면
    nav2_params.yaml 의 가속 제한(max_accel 0.3 / max_decel -0.5)이 적용되지
    않는다. 실물 펌웨어에도 wheel_odom_bridge 에도 클램프가 없으므로
    **속도·가속 제한을 걸 수 있는 곳은 velocity_smoother 뿐이다.**
    그래서 remapping 으로 중간에 끼워 넣는다.

    ※ behavior_server(후진)만은 /cmd_vel 을 직접 쓴다. Nav2 설계상 복구 행동은
      스스로 속도를 제어한다. 후진 속도가 0.05 m/s 라 문제되지 않는다.

★ YAML 안에서 채울 수 없어 여기서 주입하는 값
-----------------------------------------------
    default_nav_to_pose_bt_xml  패키지 설치 경로를 YAML 이 알 수 없다
    yaml_filename               맵은 실행할 때 정해진다
    use_sim_time                시뮬/실물 구분 (9곳 전부)
    topic (obstacle_layer.scan) 시뮬 /scan vs 실물 /scan_filtered

인자
----
    space        : narrow | wide  (기본 wide)
    use_sim_time : 시뮬이면 true (기본 false = 실물)
    map          : 맵 이름 (기본 demo_room). 또는 .yaml 로 끝나는 전체 경로
    scan_topic   : 비우면 use_sim_time 에 따라 자동 (/scan 또는 /scan_filtered)
    params_file  : 공통 파라미터 (기본 config/nav2_params.yaml)
    autostart    : lifecycle 자동 활성화 (기본 true)
    use_rviz     : Nav2 전용 RViz(nav.rviz) 동시 실행 (기본 false)
    log_level    : 노드 로그 레벨 (기본 info)
"""

import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, LogInfo, OpaqueFunction,
                            ExecuteProcess, RegisterEventHandler)
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


# space 인자 하나가 고르는 것들. 여기 외에는 장소 의존 분기가 없다.
SPACE_PRESETS = {
    'narrow': {'overlay': 'space_narrow.yaml', 'bt': 'navigate_no_spin.xml',
               'desc': '좁은 방 — inflation 0.10, 코스트맵 2x2, spin 미사용'},
    'wide':   {'overlay': 'space_wide.yaml',   'bt': 'navigate_with_spin.xml',
               'desc': '넓은 곳 — inflation 0.25, 코스트맵 5x5, spin 사용'},
}

# lifecycle_manager 가 이 순서대로 configure -> activate 한다.
# map_server 가 가장 먼저여야 global_costmap 의 static_layer 가 맵을 받는다.
LIFECYCLE_NODES = [
    'map_server',
    'controller_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'velocity_smoother',
]


def resolve_map(value: str, pkg_share: str):
    """맵 이름 또는 경로를 실제 yaml 경로로 바꾼다.

    이름만 준 경우 패키지의 maps/ 안에서 찾는다. 노트북과 젯슨의 설치 경로가
    같아지므로 명령어를 양쪽에서 그대로 쓸 수 있다.
    반환값: (경로, 문제설명 또는 None)
    """
    maps_dir = os.path.join(pkg_share, 'maps')
    available = sorted(
        os.path.splitext(os.path.basename(p))[0]
        for p in glob.glob(os.path.join(maps_dir, '*.yaml')))

    if value.endswith('.yaml') or os.sep in value:
        path = os.path.abspath(os.path.expanduser(value))
    else:
        path = os.path.join(maps_dir, value + '.yaml')

    if os.path.isfile(path):
        return path, None
    return path, (f'맵을 찾을 수 없다: {path}   '
                  f'(패키지에 있는 맵: {", ".join(available) or "없음"})')


def launch_setup(context, *args, **kwargs):
    """인자를 실제 값으로 확정한 뒤 노드를 만든다.

    OpaqueFunction 을 쓰는 이유: space 프리셋 선택, 맵 이름 해석, scan_topic
    자동 결정처럼 조건이 들어가는 처리를 읽기 쉽게 쓰고, 무엇이 선택됐는지
    기동 로그로 남기기 위함이다. Substitution 만으로는 분기가 금방 읽기
    어려워지고, 잘못 골라도 눈에 띄지 않는다.
    """
    pkg_share = get_package_share_directory('ship_ugv_navigation')

    space = LaunchConfiguration('space').perform(context).strip().lower()
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    autostart = LaunchConfiguration('autostart').perform(context)
    log_level = LaunchConfiguration('log_level').perform(context)
    scan_topic = LaunchConfiguration('scan_topic').perform(context)

    is_sim = use_sim_time.lower() in ('true', '1', 'yes')
    problems = []

    # ---- space 프리셋 ---------------------------------------------------
    # 오타를 조용히 넘기면 엉뚱한 설정으로 시연하게 된다. 기본값으로 되돌리되
    # 반드시 눈에 띄게 알린다.
    if space not in SPACE_PRESETS:
        problems.append(
            f"space:={space} 는 없는 값이다. narrow 또는 wide 만 쓸 수 있다. "
            f"wide 로 진행한다")
        space = 'wide'
    preset = SPACE_PRESETS[space]
    overlay_file = os.path.join(pkg_share, 'config', preset['overlay'])
    bt_xml = os.path.join(pkg_share, 'behavior_trees', preset['bt'])

    # ---- 맵 ---------------------------------------------------------------
    map_yaml, map_problem = resolve_map(
        LaunchConfiguration('map').perform(context), pkg_share)
    if map_problem:
        problems.append(map_problem)

    # ---- scan_topic 자동 결정 -------------------------------------------
    # 실물은 laser_filters 가 후방 180도를 잘라낸 /scan_filtered 를 써야 한다.
    # 원본 /scan 을 쓰면 차체 뒷부분이 장애물로 찍혀 계속 갇힌다.
    # 잊기 쉬운 값이라 use_sim_time 으로부터 자동 유도한다.
    if not scan_topic:
        scan_topic = '/scan' if is_sim else '/scan_filtered'

    for label, path in (('params_file', params_file),
                        ('space overlay', overlay_file),
                        ('bt_xml', bt_xml)):
        if not os.path.isfile(path):
            problems.append(f'{label} 파일 없음: {path}')

    # ---- 파라미터 주입 --------------------------------------------------
    # RewrittenYaml 은 키 "이름"을 깊이에 상관없이 찾아 바꾼다.
    #   use_sim_time -> 9곳 전부 (의도한 동작)
    #   topic        -> local/global 코스트맵의 스캔 2곳뿐 (다른 topic 키 없음)
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml,
            'default_nav_to_pose_bt_xml': bt_xml,
            'topic': scan_topic,
            'scan_topic': scan_topic,   # amcl 은 'topic' 이 아니라 'scan_topic' 키를 쓴다
        },
        convert_types=True,
    )

    # ---- keepout 마스크 -------------------------------------------------
    # 라이다 스캔 평면보다 낮은 대상(모형 배)은 코스트맵에 안 잡혀 로봇이
    # 가로질러 갈 수 있다. finalize_map.py 가 만들어 둔 마스크가 있으면
    # KeepoutFilter 로 그 자리를 코스트맵에 직접 박는다.
    #
    # 마스크가 없으면 필터를 켜지 않는다. 켜 두면 KeepoutFilter 가
    # /costmap_filter_info 를 기다리다 코스트맵이 활성화되지 못해 Nav2 가 안 뜬다.
    map_name = os.path.splitext(os.path.basename(map_yaml))[0]
    mask_yaml = os.path.join(pkg_share, 'masks', f'keepout_{map_name}.yaml')
    keepout_on = os.path.isfile(mask_yaml)

    # 있든 없든 반드시 로그로 알린다. 조용히 건너뛰면 나중에 마스크를 만들어 두고도
    # "왜 keepout 이 안 먹지" 하며 원인을 못 찾는다.
    if keepout_on:
        banner_keepout = f'사용 — masks/keepout_{map_name}.yaml'
    else:
        banner_keepout = (f'사용 안 함 — masks/keepout_{map_name}.yaml 없음. '
                          f'라이다에 잡히는 대상만 회피한다')

    # ★ 순서가 중요하다. 공통 -> 장소별 순으로 줘야 장소별이 이긴다.
    #   overlay 에 적힌 값만 덮어써지고 나머지는 공통 파일 값이 그대로 남는다.
    node_params = [configured_params, overlay_file]
    if keepout_on:
        node_params.append(os.path.join(pkg_share, 'config', 'keepout_on.yaml'))

    common = dict(output='screen',
                  arguments=['--ros-args', '--log-level', log_level])

    nodes = [
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=node_params, **common),

        # ★ /cmd_vel 이 아니라 /cmd_vel_nav 로 내보낸다 (velocity_smoother 경유)
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', parameters=node_params,
             remappings=[('cmd_vel', 'cmd_vel_nav')], **common),

        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', parameters=node_params, **common),

        # 복구 행동은 /cmd_vel 을 직접 쓴다 (Nav2 설계)
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', parameters=node_params, **common),

        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', parameters=node_params, **common),

        # ★ 여기서 속도·가속 제한이 걸린 뒤 로봇으로 나간다
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', parameters=node_params,
             remappings=[('cmd_vel', 'cmd_vel_nav'),
                         ('cmd_vel_smoothed', 'cmd_vel')], **common),

    ]

    # keepout 을 쓸 때만 서버 두 개를 띄운다.
    #   filter_mask_server        마스크 이미지를 OccupancyGrid 로 발행 (map_server 재사용)
    #   costmap_filter_info_server 그 마스크를 어떻게 해석할지 알려줌
    # 둘 다 lifecycle 노드라 관리 목록에도 넣어야 activate 된다.
    lifecycle_nodes = list(LIFECYCLE_NODES)
    if keepout_on:
        lifecycle_nodes = (['filter_mask_server', 'costmap_filter_info_server']
                           + lifecycle_nodes)
        nodes += [
            Node(package='nav2_map_server', executable='map_server',
                 name='filter_mask_server',
                 parameters=[{'use_sim_time': is_sim,
                              'yaml_filename': mask_yaml,
                              'topic_name': 'keepout_mask',
                              'frame_id': 'map'}], **common),
            Node(package='nav2_map_server', executable='costmap_filter_info_server',
                 name='costmap_filter_info_server',
                 parameters=[{'use_sim_time': is_sim,
                              # type 0 = keepout. base/multiplier 는 마스크 값을
                              # 코스트로 바꾸는 1차식이며 keepout 은 그대로 쓴다.
                              'type': 0,
                              'filter_info_topic': '/costmap_filter_info',
                              'mask_topic': 'keepout_mask',
                              'base': 0.0,
                              'multiplier': 1.0}], **common),
        ]

    # ---- AMCL 라이다 로컬라이제이션 (2026-08-15 추가) --------------------
    # ★ TF 는 발행하지 않는다 (nav2_params.yaml 의 tf_broadcast: false).
    #   /amcl_pose 만 내고, ekf_global 이 그것을 pose1 센서 입력으로 먹는다.
    #   map->odom 발행자는 계속 ekf_global 하나뿐이라 TF 이중 발행이 없다.
    #   근거와 배경은 nav2_params.yaml 의 amcl 블록 주석 참고.
    #
    # 시뮬에서는 켜지 않는다 — fake_global_localization 이 참값을 주므로
    # AMCL 이 할 일이 없고, 오히려 실물과 다른 구조가 되어 이식이 어긋난다.
    amcl_arg = LaunchConfiguration('amcl').perform(context).strip().lower()
    if amcl_arg == '':
        amcl_on = not is_sim          # 기본값: 실물이면 켠다
        amcl_how = '(실물이라 자동으로 켬)'
    else:
        amcl_on = amcl_arg in ('true', '1', 'yes')
        amcl_how = '(amcl 인자로 지정)'

    if amcl_on:
        # map_server 다음, controller_server 앞에 둔다 — 맵을 받아야 초기화된다.
        lifecycle_nodes.insert(lifecycle_nodes.index('map_server') + 1, 'amcl')
        nodes.append(Node(package='nav2_amcl', executable='amcl', name='amcl',
                          parameters=node_params, **common))
        # AMCL 은 초기 위치를 모르면 수렴하지 못한다. ekf_global(UWB) 추정치를
        # 씨앗으로 한 번 넣어 주고 스스로 종료하는 노드.
        nodes.append(Node(package='ship_ugv_navigation', executable='amcl_seed_node',
                          name='amcl_seed_node',
                          parameters=[{'use_sim_time': is_sim}], **common))
        banner_amcl = f'사용 {amcl_how} — TF 미발행, /amcl_pose -> ekf_global pose1'
    else:
        banner_amcl = f'사용 안 함 {amcl_how}'

    # ---- wheel_odom_bridge 의 heading_hold 끄기 (2026-08-17) --------------
    # ★ 왜: wheel_odom_bridge 는 /cmd_vel 의 w 가 0 근처(|w| < 0.02)면
    #   "직진 의도"로 보고 **발행자의 조향을 버리고 자기 PI 제어로 덮어쓴다.**
    #   teleop·motion_controller 에게는 유용한 기능이지만(직진이 안 휘게 해준다),
    #   Nav2 는 자기 컨트롤러가 이미 조향을 닫은 루프로 제어하고 있다.
    #   그 위에 두 번째 제어 루프가 얹히면
    #     ① Nav2 의 미세 조향(w=0.015 같은 값)이 통째로 버려져 경로 추종이 나빠지고
    #     ② 두 루프가 서로 싸운다.
    #   실제로 2026-08-17 에 이 heading_hold 가 상한 없는 w 를 만들어
    #   **모터가 두 번 폭주했다** (wheel_odom_node.py 의 안전 수정 주석 참고).
    #
    # 노드를 재시작하지 않고 파라미터만 바꾸는 이유: wheel_odom_bridge 는
    # localization.launch.py 소속이라 재시작하면 UWB 캘리브레이션까지 날아간다.
    # 그래서 bridge 쪽에서 이 파라미터를 매 콜백마다 다시 읽도록 해 두었다.
    nodes.append(ExecuteProcess(
        cmd=['ros2', 'param', 'set', '/wheel_odom_bridge',
             'enable_heading_hold', 'false'],
        output='screen'))

    # ★ 그리고 Nav2 가 꺼질 때 반드시 되돌린다 (2026-08-17).
    #   되돌리지 않으면 이런 함정이 생긴다:
    #     Nav2 를 한 번 켰다 끔 -> heading_hold 가 false 로 남음
    #     -> 그 상태로 재캘리브레이션하면 직진 보정 없이 주행해 크게 휨
    #     -> 원인을 찾기 매우 어렵다 (아무 에러도 안 나고 조용히 휜다)
    #   wheel_odom_bridge 는 localization.launch.py 소속이라 이 launch 를
    #   껐다고 재시작되지 않는다. 그래서 여기서 명시적으로 복원해야 한다.
    nodes.append(RegisterEventHandler(OnShutdown(on_shutdown=[
        LogInfo(msg='Nav2 종료 — wheel_odom_bridge 의 heading_hold 를 다시 켠다 '
                    '(teleop·캘리브레이션 직진 보정용)'),
        ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/wheel_odom_bridge',
                 'enable_heading_hold', 'true'],
            output='screen'),
    ])))

    nodes.append(
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation',
             parameters=[{'use_sim_time': is_sim,
                          'autostart': autostart.lower() in ('true', '1', 'yes'),
                          'node_names': lifecycle_nodes}], **common))

    # ---- 순찰 노드 (Step 6) ---------------------------------------------
    # 순찰 원의 중심·반지름은 **맵마다 다르다.** 그래서 맵 이름에 묶어
    # config/patrol_<맵이름>.yaml 을 자동으로 찾는다. map 인자 하나가
    # 맵과 순찰 원을 같이 정하므로 따로 지정할 일이 없다.
    patrol_on = LaunchConfiguration('patrol').perform(context).lower() in (
        'true', '1', 'yes')
    if patrol_on:
        patrol_cfg = os.path.join(pkg_share, 'config', f'patrol_{map_name}.yaml')
        if os.path.isfile(patrol_cfg):
            patrol_params = [patrol_cfg, {'use_sim_time': is_sim}]
            banner_patrol = f'patrol_{map_name}.yaml'
        else:
            # 없는 맵이면 노드 기본값으로 뜬다. 엉뚱한 원을 돌게 되므로 크게 알린다.
            problems.append(
                f'{os.path.basename(patrol_cfg)} 가 없다. 순찰 원을 모르는 상태로 뜬다. '
                f'check_patrol_space.py 로 중심·반지름을 구해 파일을 만들 것')
            patrol_params = [{'use_sim_time': is_sim}]
            banner_patrol = '(없음 — 노드 기본값)'
        nodes.append(Node(
            package='ship_ugv_navigation', executable='patrol_mission_node',
            name='patrol_mission_node', parameters=patrol_params, **common))
    else:
        banner_patrol = '사용 안 함 (patrol:=true 로 켠다)'

    # ---- 이벤트 게이트 (Step 7) -----------------------------------------
    # 기본값은 "patrol 을 따라간다". 순찰을 켜면 이벤트 정지도 함께 켜지는 것이
    # 시연에서 원하는 동작이고, 튜닝 중에만 events:=false 로 끄면 된다.
    events_arg = LaunchConfiguration('events').perform(context).strip().lower()
    if events_arg == '':
        events_on = patrol_on
        how = '(patrol 을 따라감)'
    else:
        events_on = events_arg in ('true', '1', 'yes')
        how = '(직접 지정)'
    if events_on:
        nodes.append(Node(
            package='ship_ugv_navigation', executable='event_gate_node',
            name='event_gate_node', parameters=[{'use_sim_time': is_sim}],
            **common))
        banner_events = f'사용 {how} — fire/fallen_person/no_helmet 에 정지'
    else:
        banner_events = f'사용 안 함 {how}'

    if LaunchConfiguration('use_rviz').perform(context).lower() in ('true', '1', 'yes'):
        nodes.append(Node(
            package='rviz2', executable='rviz2', name='rviz2_nav',
            parameters=[{'use_sim_time': is_sim}],
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'nav.rviz')],
            output='screen'))

    banner = [
        LogInfo(msg='─── Nav2 설정 ' + '─' * 46),
        LogInfo(msg=f'  space        : {space}   ({preset["desc"]})'),
        LogInfo(msg=f'  use_sim_time : {use_sim_time}   ({"시뮬" if is_sim else "실물"})'),
        LogInfo(msg=f'  scan_topic   : {scan_topic}'),
        LogInfo(msg=f'  map          : {map_yaml}'),
        LogInfo(msg=f'  덮어쓰기     : {preset["overlay"]}'),
        LogInfo(msg=f'  행동트리     : {preset["bt"]}'),
        LogInfo(msg=f'  순찰         : {banner_patrol}'),
        LogInfo(msg=f'  이벤트 정지  : {banner_events}'),
        LogInfo(msg=f'  keepout      : {banner_keepout}'),
        LogInfo(msg=f'  amcl         : {banner_amcl}'),
        LogInfo(msg='  cmd_vel 경로 : controller -> /cmd_vel_nav -> smoother -> /cmd_vel'),
        LogInfo(msg='─' * 60),
    ]
    banner += [LogInfo(msg=f'  ⚠️  {p}') for p in problems]

    return banner + nodes


def generate_launch_description():
    pkg_share = get_package_share_directory('ship_ugv_navigation')

    args = [
        DeclareLaunchArgument(
            'space', default_value='wide',
            description='narrow(좁은 방) 또는 wide(넓은 곳). '
                        'inflation·코스트맵 크기·spin 사용 여부가 한 번에 바뀐다'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='시뮬이면 true. 실물은 false (기본)'),
        DeclareLaunchArgument(
            'map', default_value='demo_room',
            description='맵 이름 (패키지 maps/ 안). 실물은 shipyard_map_JG_room_v2. '
                        '또는 .yaml 로 끝나는 전체 경로'),
        DeclareLaunchArgument(
            'scan_topic', default_value='',
            description='비우면 자동 (시뮬 /scan, 실물 /scan_filtered)'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
            description='공통 Nav2 파라미터 파일'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='lifecycle 노드를 자동으로 activate 할지'),
        DeclareLaunchArgument(
            'patrol', default_value='false',
            description='순찰 노드 동시 실행. 원의 중심·반지름은 map 이름에 묶인 '
                        'config/patrol_<맵이름>.yaml 에서 자동으로 읽는다'),
        DeclareLaunchArgument(
            'events', default_value='',
            description='이벤트 정지 노드(event_gate_node) 실행. '
                        '비우면 patrol 값을 따라간다. 튜닝 중 끄려면 events:=false'),
        DeclareLaunchArgument(
            'amcl', default_value='',
            description='AMCL 라이다 로컬라이제이션(TF 미발행, /amcl_pose 를 '
                        'ekf_global 에 공급). 비우면 실물에서만 자동으로 켠다'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='Nav2 전용 RViz(nav.rviz) 동시 실행'),
        DeclareLaunchArgument(
            'log_level', default_value='info',
            description='노드 로그 레벨 (debug/info/warn/error)'),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
