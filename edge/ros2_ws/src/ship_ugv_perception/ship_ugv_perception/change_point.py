#!/usr/bin/env python3
"""
change_point.py (리팩터링판 + 2차 중복 제거 필터 추가)
----------------------------
Depth camera 이벤트 감지 (u, v, depth)를 map 좌표계의 절대 위치로 변환한다.

[2026-07-08 추가] map 좌표 기준 중복 제거 (2차 필터)
------------------------------------------------------
같은 클래스의 이벤트가 map 좌표상 일정 반경(dedup_radius_m) 안에서 이미
보고된 적이 있으면 재발행하지 않는다. yolo_depth_publisher.py의 track ID
기반 1차 필터(같은 프레임 흐름 안에서의 중복 방지)와 별개로, 로봇이
이동하며 같은 지점을 다시 지나치는 경우까지 커버하기 위한 것.
일정 시간(event_ttl_s) 동안 재감지가 없으면 목록에서 제거해, 같은 위치에서
실제로 새로 발생한 이벤트(예: 꺼졌던 불이 다시 남)는 다시 보고될 수 있게 한다.
"""

import json
import math
import os
import tempfile

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,
                       QoSReliabilityPolicy)
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401  (PointStamped 변환을 위해 필요한 등록)


def angle_diff(a, b):
    """두 각도의 차이를 -pi~pi 로 정규화해 절댓값으로 준다."""
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def clear_verdict(dist_to_vantage_m, yaw_diff_rad, last_seen_s, now_s,
                  revisit_radius_m, revisit_yaw_tol_rad, revisit_grace_s,
                  left_vantage, arrived_at_s, event_in_fov, fov_seen_s, dt_s,
                  max_departure_m=None, min_departure_m=0.0, min_unseen_s=0.0):
    """치워졌는지 판정. 반환 (verdict, left_vantage, arrived_at_s, fov_seen_s).

    ★ 2026-08-27 실측 사고: "구역 안에 머문 시간" 으로 세면 안 된다.
      로봇은 불을 **지나친 뒤에** 멈춘다(정지까지 1초쯤 걸린다). 그 자리에서
      카메라(우측 고정)는 불을 안 보는 쪽을 향하고 있는데, 사용자가 팝업
      확인을 누를 때까지 10초쯤 서 있는다. 그러면 grace 3초가 그냥 채워져
      "지켜봤는데 안 보임" 이 성립하고, 불이 멀쩡히 있는데 핑이 지워졌다.
      지워지면 목록에서도 빠지므로 다음 검출이 **새 이벤트로 재등록되어
      로봇이 또 멈춘다** — 실측: 등록 02:32:50 -> 치워짐 02:33:03(13초)
      -> 5cm 옆에 재등록 02:33:04 + 재정지. 사용자는 손도 안 댔다.

      그래서 **카메라가 실제로 그 이벤트 방향을 향한 시간(fov_seen_s)** 만
      센다. 등 돌리고 서 있는 시간은 0초로 친다. in_camera_fov() 는 예전에
      이 목적으로 만들어 놓고도 **아무 데서도 부르지 않는 죽은 코드**였다.

    ★ 2026-08-29 실측 사고: 등록 6초 만에 치워짐 판정이 났다.

      한 바퀴가 50초인데 6초 만에 "그 자리를 떠났다 돌아왔다" 가 성립했다.
      실제로 일어난 일:

          불 등록 -> event_gate 가 로봇을 세움
            -> 로봇이 seen_from 에서 약 0.35 m 지점에 멈춤
            -> revisit_radius(0.35) 경계를 들락날락
            -> 들어올 때마다 arrived_at 이 새로 찍힘
            -> last_seen(=등록 시각) < arrived_at 은 항상 참
            -> 각도는 맞으니 FOV 시간이 3초 차서 확정

      left_vantage 가 "0.35 m 만 벗어나면 떠난 것" 으로 판정하는 것이
      문제였다. 정지하며 경계를 넘나드는 것을 한 바퀴로 착각한다.

      또 다른 사례는 가림이었다 — 18.5초 안 보였다고 치웠는데 8초 뒤
      같은 자리(4 cm)에서 다시 잡혔다.

      -> 두 조건을 더한다:
         ① min_departure_m  등록 이후 seen_from 에서 이만큼은 실제로
            멀어진 적이 있어야 한다. 정지 중 경계 진동으로는 못 채운다
            (한 바퀴 돌면 2 m 벌어진다).
         ② min_unseen_s     이만큼 연속으로 안 보여야 한다. 오판 사례가
            18.5초였고 정상 치워짐은 39~52초라 25초면 정상은 안 막는다.
    """
    tick = dt_s if event_in_fov else 0.0

    # ★ 조건 두 개 추가 (2026-08-29 실측 사고). 아래 docstring 참고.
    #   ① 진짜로 멀리 갔다 와야 한다   ② 충분히 오래 안 보여야 한다
    went_far = (max_departure_m is not None
                and max_departure_m >= min_departure_m)
    unseen_long = (now_s - last_seen_s) >= min_unseen_s

    if dist_to_vantage_m > revisit_radius_m:
        # 구역 밖 — 이번 방문이 끝난 시점이다. 여기서만 판정한다.
        if (left_vantage
                and arrived_at_s is not None
                and fov_seen_s >= revisit_grace_s
                and last_seen_s < arrived_at_s
                and went_far
                and unseen_long):
            return 'clear', True, None, 0.0
        return 'wait', True, None, 0.0
    if not left_vantage:
        return 'wait', left_vantage, None, 0.0
    if arrived_at_s is None:
        if yaw_diff_rad > revisit_yaw_tol_rad:
            return 'wait', left_vantage, None, 0.0
        return 'wait', left_vantage, now_s, tick
    return 'wait', left_vantage, arrived_at_s, fov_seen_s + tick


def quat_to_yaw(x, y, z, w):
    """평면(2D) 로봇 가정 — 쿼터니언에서 Z축 회전(요)만 뽑아낸다."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def in_camera_fov(robot_yaw, cam_yaw, hfov, robot_x, robot_y, event_x, event_y):
    """지금 이 위치·헤딩에서 카메라가 이벤트 방향을 실제로 보고 있는지.

    ★ 2026-08-24 (주현 진단, 근본 원인) ★
    clear 판정이 거리만 보고 카메라가 실제로 그쪽을 보고 있었는지는
    확인하지 않았다. camera_yaw_deg=-90(로봇 우측 고정)인 상태로 순찰
    왕복/원형 구간을 반대 방향으로 지나가면 카메라는 이벤트 반대쪽을
    보고 있는데도, 거리만으로 "지켜봤는데 없었다"고 오판해 지워버리고
    다음 바퀴에 같은 걸 다시 보면 새 이벤트로 재등록 -> 재정지가
    반복됐다. 반경/시간(dedup_radius_m, clear_radius_m, clear_watch_s)을
    아무리 넓혀도 카메라가 구조적으로 못 보는 구간 자체는 없어지지
    않으므로 근본 해결이 안 됐다(지난 3커밋: 8428374, 057d5f3, 16cbf9d).
    """
    camera_facing = robot_yaw + cam_yaw
    bearing = math.atan2(event_y - robot_y, event_x - robot_x)
    diff = math.atan2(math.sin(bearing - camera_facing), math.cos(bearing - camera_facing))
    return abs(diff) <= hfov / 2.0


class ChangePointDetector(Node):

    def __init__(self):
        super().__init__('change_point_detector')

        # ---- 파라미터 ----
        self.declare_parameter('detection_topic', '/event_detection/uvd')
        self.declare_parameter('output_topic', '/event_detection/map_point')
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('base_frame_id', 'base_link')
        # ★ 실측: base_link(바퀴축 중점) 기준 카메라 RGB 렌즈(광학중심) 위치 (2026-08-13 재실측)
        #   x: 휠 축에서 전방 13.5cm
        #   y: 차체 오른쪽 면 장착 → ROS(+y=좌측) 규약상 음수. chassis_width(0.178)/2와 일치
        #   z: 지상고(URDF 0.075) + 차체 밑면에서 3.5cm
        self.declare_parameter('camera_offset_x', 0.135)
        self.declare_parameter('camera_offset_y', -0.089)
        self.declare_parameter('camera_offset_z', 0.110)
        # ★ 카메라 장착 회전각 (실측: 로봇 정면 기준 오른쪽을 보도록 장착됨).
        #   yaw=0이면 카메라 정면=로봇 정면. 오른쪽을 보면 로봇 기준
        #   시계방향으로 돌아간 것이므로 REP-103 규약상 음수 각도.
        self.declare_parameter('camera_yaw_deg', -90.0)
        self.declare_parameter('camera_hfov_deg', 74.0)  # Astra+ RGB FOV
        self.declare_parameter('image_width', 640)
        self.declare_parameter('depth_is_radial', False)
        self.declare_parameter('tf_timeout_s', 0.3)

        # ★ 2차 필터 파라미터
        # ── dedup_radius_m 변천사 (0.5 -> 1.0 -> 0(임시) -> 0.5) ──
        #   커밋 8428374 에서 0.5 -> 1.0 으로 키웠다. 근거는 "같은 불인데
        #   로봇 각도만 바뀌면 새 이벤트로 재등록된다" 였다. 그런데 그건
        #   **오진이었다**. 실측 로그를 다시 보면 재등록 직전에 항상 clear
        #   가 먼저 있었다:
        #       18:26:23  fire@-0.20,-1.14  cleared
        #       18:28:42  fire@-0.29,-1.04  새 이벤트 (겨우 14cm 옆)
        #   14cm 면 0.5m 반경으로도 매칭됐을 거리다. 즉 반경이 좁아서가
        #   아니라 clear 가 먼저 목록에서 지워버려서 재등록된 것이고,
        #   그 clear 오판의 진짜 원인은 아래 in_camera_fov 게이트로 잡았다.
        #
        #   반대로 1.0m 는 실제 피해를 냈다: 불을 0.9m 옮겼는데 1.0m 반경
        #   안이라 "같은 불"로 매칭돼 재발행이 안 됐고, 정지도 팝업도 안
        #   떴다(2026-08-24 실측). 하나의 반경이 "각도 바뀌어도 같은 불로
        #   봐라"와 "옮기면 다른 불로 봐라"를 동시에 만족시킬 수 없다.
        #   -> 근본 원인이 FOV 게이트로 해결됐으므로 0.5 로 되돌렸다.
        #
        # ★ 0.5 -> 0.35 (2026-08-27 저녁). 실측으로 안전 구간이 정해졌다.
        #     같은 불 제자리 흔들림  중앙값 0.04m / 최대 0.10m
        #       (순찰 원 정반대편에서 봐도 4cm — 시점 편향은 사실상 없다)
        #     이동거리 측정 오차     -0.06m ~ +0.11m
        #       (자 0.70 -> 계산 0.64 / 자 0.75 -> 계산 0.86)
        #   여기서 두 제약이 나온다:
        #     너무 작으면 제자리 흔들림을 "옮겼다"로 오판  -> 반경 > 0.10m
        #     너무 크면 진짜 이동을 "같은 불"로 무시
        #       0.7m 이동이 최악 0.59m 로 읽히므로          -> 반경 < 0.59m
        #   안전 구간 0.15~0.55m 안에서 0.5 는 위쪽에 치우쳐 있었다
        #   (흔들림 여유 0.40m vs 이동감지 여유 0.09m). 0.35 면 0.25/0.24 로
        #   균형이 맞고, 0.5~0.6m 만 옮겨도 잡힌다.
        #   오탐 위험은 오히려 준다 — 새 이벤트 2회 확인이 "오측정 둘이 이
        #   반경 안에서 서로 일치할 것"을 요구하므로 반경이 작을수록 엄격해진다.
        #
        # ★ 0.35 -> 0.5 로 되돌림 (2026-08-29). 위 0.35 판단이 틀렸다.
        #   그때 근거로 삼은 "흩어짐 최대 0.196 m" 는 110초 표본이었는데,
        #   더 오래 돌려보니 **0.39 m 짜리 계통 오차**가 가끔 난다. 랜덤이
        #   아니다 — 두 표본이 0.06 m 안에서 서로 일치해 2회 확인까지
        #   통과했다. 특정 시야각에서 depth 를 잘못 읽는 것으로 보인다.
        #
        #       등록  fire@0.47,-1.02        (실측 군집 중심과 일치)
        #       0.39 m 떨어진 곳에 또 등록   fire@0.75,-1.28
        #       -> 실제 불은 하나인데 핑이 두 개, 로봇도 한 번 더 정지
        #
        #   지금 센서 정확도로는 **0.45 m 이동 감지와 중복 방지를 동시에
        #   만족할 수 없다.** 0.39 m 오차가 나는데 0.45 m 이동을 구분하려는
        #   것 자체가 무리다. 시연 시나리오가 0.7 m 이상 이동이므로
        #   중복 방지 쪽을 택한다.
        #
        #   0.5 를 고른 이유 (0.6 이 아니라):
        #       관측 최대 계통오차 0.39 m  -> 0.5 면 흡수 (여유 0.11)
        #       실제 두 불 사이   0.74 m  -> 0.5 면 구분 (여유 0.24)
        #       0.6 이면 두 불 구분 여유가 0.14 뿐이라 위험하다.
        #
        #   ⚠️ 대가: 0.5 m 미만 이동은 감지 못 한다. 시험할 때 **0.7 m
        #   이상** 옮길 것.
        self.declare_parameter('dedup_radius_m', 0.5)   # 같은 이벤트로 볼 거리 반경
        self.declare_parameter('event_ttl_s', 600.0)    # 이 시간 이상 재감지 없으면 "새 이벤트"로 취급

        # ★ 2026-08-21 신설 — 능동 클리어링.
        #   순찰 중 fire 를 정지-확인-재개했는데, 반바퀴도 못 가서 **같은 자리의
        #   같은 불**로 다시 정지하는 사고가 있었다. 원인은 event_gate_node 와
        #   websocket_client 가 이 노드를 거치지 않고 원본 /event_detection/uvd
        #   를 각자 직접 구독해서, 여기서 이미 만들어둔 위치 기반 중복 제거가
        #   실제 정지/서버전송 경로에는 전혀 적용되지 않고 있었다는 것이다.
        #   (이 노드는 그동안 ship_survey_node 용으로만 쓰였다)
        #   -> event_gate_node·websocket_client 를 이 노드의 출력
        #      (/event_detection/map_point, 이미 중복 제거된 스트림)을 보도록
        #      바꿨다. 그러면 같은 자리의 같은 불은 애초에 다시 발행되지 않는다.
        #
        #   그런데 "TTL 600초 뒤에 조용히 잊는다" 만으로는 부족하다. 로봇이
        #   불을 치운 자리를 곧장 다시 지나가도 프론트 화면의 빨간 핑이 9분
        #   넘게 남아 있게 된다. 그래서 **로봇이 그 자리를 다시 지나가며
        #   지켜봤는데 안 보이면** 즉시 event_cleared 를 쏴서 핑을 지운다.
        self.declare_parameter('clear_topic', '/event_detection/cleared')
        # ★ 2026-08-24: 0.6m/3.0s 로는 부족했다. 실측: 계속 잡히던 fire가
        #   순찰 각도 바뀌는 중 ~2.45초간 재검출이 비어 clear 오판 -> 24초 뒤
        #   같은 불이 "새 이벤트"로 재등록되는 사고가 실측됨. dedup_radius_m
        #   과 같은 이유(헤딩 오차로 인한 위치 흔들림)로 반경을 키우고,
        #   실측된 순간적 미검출 구간(약 2.5초)보다 넉넉하게 watch 시간도
        #   늘린다.
        #
        #   ★ 2026-08-24 2차: 1.2m/5.0s 로 넓혀도 주기만 20초->4분대로
        #   늘었을 뿐 완전히 없어지지 않았다(실측: 같은 자리 fire가 4분
        #   19초 뒤 다시 clear->재등록). 데모 전까지는 "안 멈추는 것"보다
        #   "이미 확인한 걸 가끔 또 세운다"가 훨씬 안전하다는 판단으로,
        #   재발 빈도를 더 낮추기 위해 한 번 더 넉넉하게 키운다. 여전히
        #   0이 되진 않지만(로봇이 그 자리를 몇 분 이상 안 지나가면 결국
        #   반경 안에서 몇 초는 재검출 없이 지나갈 수 있다), 데모 시간
        #   내에는 거의 안 걸릴 정도로 늘린다.
        # ★ 2026-08-24 최종 (사용자 제안). "마지막으로 본 지 60초" 방식은
        #   배 가림(약 30초)을 오판하지 않으려다 최악 2.2바퀴(110초)가
        #   걸렸다. 대신 **처음 그 불을 본 로봇 위치**를 기억해두고, 그
        #   자리로 돌아왔는데 안 보이면 치워진 것으로 본다. 그 자리에서는
        #   분명히 보였으니 가림을 추측할 필요가 없다 -> 한 바퀴 안에 판정.
        self.declare_parameter('revisit_radius_m', 0.35)    # 이만큼 가까우면 "그 자리로 돌아왔다"
        self.declare_parameter('revisit_yaw_tol_deg', 45.0) # 헤딩도 비슷해야 카메라가 같은 곳을 본다
        self.declare_parameter('revisit_grace_s', 3.0)      # 그 자리에서 이만큼 안 보이면 확정
        # ★ 2026-08-29 — 등록 6초 만에 오판하던 것을 막는다
        #   (자세한 이유는 clear_verdict docstring 참고)
        self.declare_parameter('min_departure_m', 0.8)   # 실제로 이만큼 멀어졌다 와야 함
        self.declare_parameter('min_unseen_s', 25.0)     # 이만큼 연속으로 안 보여야 함

        # ★ 배 중심에서 이보다 먼 검출은 버린다 (2026-08-29 신설).
        #
        #   max_depth_m 은 **로봇~대상** 거리를 재므로 이걸 못 막는다.
        #   실측: 유령 4건이 배 중심에서 1.6~2.0 m 였는데, 로봇이 순찰 원의
        #   그쪽 지점에 있을 때 **로봇~유령 거리는 약 1 m** 였다. 즉 depth 는
        #   정상 범위라 통과했다. 유령은 "멀리 있는 것" 이 아니라 "로봇 옆인데
        #   배 반대편에 있는 것" 이다.
        #
        #   전체 통계: fire 등록 181건 중 49건(27%)이 배에서 1.2 m 초과였다.
        #   그동안 프론트가 막아왔지만 젯슨은 그대로 서버에 보내고 로봇도
        #   멈췄다. 1차 방어선을 여기 둔다.
        #
        #   ⚠️ 프론트의 MAX_EVENT_DIST_FROM_SHIP_M 과 **짝이다. 같이 바꿀 것.**
        #   순찰 반지름을 키우거나 대상을 배에서 멀리 두면 둘 다 올려야 한다.
        #
        #   배 위치를 모르면 **거르지 않는다**(fail open). 위험을 놓치는 것보다
        #   유령을 통과시키는 편이 안전하다.
        self.declare_parameter('max_dist_from_ship_m', 1.2)

        # ★ AMCL 이 붙기 전에는 이벤트를 등록하지 않는다 (2026-08-29).
        #
        #   change_point 는 로컬라이제이션 런치에 들어 있어 Nav2 보다 10~60초
        #   먼저 뜬다. 그 창에는 AMCL 이 없어서 heading 을 보정하는 것이
        #   아무것도 없다(IMU+UWB 상보필터만). 실측:
        #
        #       같은 불, 같은 자리
        #         AMCL 없을 때  배기준 앞뒤 -0.493  ->  S5 선미
        #         AMCL 있을 때  배기준 앞뒤 +0.484  ->  S1 선수
        #
        #   heading 이 49도만 틀어져도 뱃머리와 선미가 뒤집힌다. 그렇게 만든
        #   좌표가 기억 파일에 저장되면 계속 남는다.
        #
        #   미루는 것이 안전한 이유: AMCL 이 없다는 것은 Nav2 가 없다는 뜻이고,
        #   그러면 **로봇이 움직이지 않는다.** 순찰을 안 하는 동안 이벤트를
        #   안 잡아도 놓치는 위험이 없다. 반대로 그때 잡으면 틀린 좌표가
        #   기억에 박힌다.
        #
        #   텔레옵처럼 Nav2 없이 굴려야 하면 False 로 둘 것.
        self.declare_parameter('require_amcl', True)
        # ★ 새 이벤트는 이만큼 연속으로 같은 자리에서 봐야 등록한다 (2026-08-27).
        #   단발 오측정 하나가 곧바로 로봇을 세우는 것을 막는다.
        self.declare_parameter('new_event_confirm_frames', 2)
        self.declare_parameter('new_event_confirm_window_s', 2.0)
        # ★ 재시작해도 이벤트 기억을 잃지 않게 파일에 남긴다 (2026-08-27).
        #   지우려면 이 파일을 rm 하면 된다(시작 로그에 경로가 찍힌다).
        self.declare_parameter(
            'state_file',
            os.path.join(os.path.expanduser('~'), '.ros',
                         'change_point_events.json'))
        # ★ 2026-08-24: confidence 필터. 실측 0.468짜리 한 프레임 오탐이
        #   그대로 새 이벤트(정지 유발)로 등록된 사고가 있었다.
        #   websocket_client 가 이미 쓰는 기준(0.5)과 맞춘다.
        self.declare_parameter('min_confidence', 0.5)
        # ★ 2026-08-24: depth 상한. 이게 없으면 쓰레기 depth 가 그대로 map
        #   좌표로 투영돼 맵 밖(x=5.13, 경계는 4.95)까지 유령 이벤트가 찍혔다.
        #   진짜 불은 배 위에 있어 로봇~불 거리 상한이
        #       순찰 반지름(1.0) + 불이 배 중심에서 떨어진 거리
        #   이고, 불을 배 중심에서 1.0m 까지 옮겨도 2.0m 다.
        #   순찰 반지름을 키우거나 불을 배에서 더 멀리 두려면 같이 올릴 것.
        self.declare_parameter('max_depth_m', 2.0)
        self.declare_parameter('clear_check_hz', 2.0)

        self.map_frame = self.get_parameter('map_frame_id').value
        self.base_frame = self.get_parameter('base_frame_id').value
        self.cam_offset = (
            self.get_parameter('camera_offset_x').value,
            self.get_parameter('camera_offset_y').value,
            self.get_parameter('camera_offset_z').value,
        )
        self.cam_yaw = math.radians(self.get_parameter('camera_yaw_deg').value)
        self.hfov = math.radians(self.get_parameter('camera_hfov_deg').value)
        self.image_width = self.get_parameter('image_width').value
        self.depth_is_radial = self.get_parameter('depth_is_radial').value
        self.tf_timeout = Duration(seconds=self.get_parameter('tf_timeout_s').value)

        self.dedup_radius = self.get_parameter('dedup_radius_m').value
        self.event_ttl = Duration(seconds=self.get_parameter('event_ttl_s').value)
        self.revisit_radius = self.get_parameter('revisit_radius_m').value
        self.revisit_yaw_tol = math.radians(
            self.get_parameter('revisit_yaw_tol_deg').value)
        self.revisit_grace = self.get_parameter('revisit_grace_s').value
        self.min_departure = float(self.get_parameter('min_departure_m').value)
        self.min_unseen = float(self.get_parameter('min_unseen_s').value)
        self.max_dist_from_ship = float(
            self.get_parameter('max_dist_from_ship_m').value)
        self._ship_center = None          # 모르면 거르지 않는다
        self.require_amcl = bool(self.get_parameter('require_amcl').value)
        self._amcl_ready = not self.require_amcl
        self.new_confirm = int(self.get_parameter('new_event_confirm_frames').value)
        self.new_window = float(self.get_parameter('new_event_confirm_window_s').value)
        self._pending = []   # 아직 확정 안 된 새 이벤트 후보
        self.min_confidence = self.get_parameter('min_confidence').value
        self.max_depth = self.get_parameter('max_depth_m').value

        # ★ 2차 필터: 이미 보고한 이벤트 기록
        # 각 항목: {'class_id': str, 'x': float, 'y': float, 'last_seen': rclpy.time.Time}
        self.reported_events = []
        self.state_file = self.get_parameter('state_file').value
        self._restore_state()

        # ---- TF ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- 통신 ----
        self.create_subscription(
            String, self.get_parameter('detection_topic').value,
            self._detection_cb, 10)
        # ★ 서버 -> 젯슨 중계 채널. reset_events 만 우리 관심사다 (2026-08-29).
        #   프론트의 [초기화] 버튼이 여기까지 온다. 유령 핑이 생겼을 때
        #   프로세스를 다 껐다 켜는 대신 기억만 비우려는 것이다.
        self.declare_parameter('inbound_topic', '/server/inbound')
        self.create_subscription(
            String, self.get_parameter('inbound_topic').value,
            self._inbound_cb, 10)

        if self.require_amcl:
            self.create_subscription(
                PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, 10)

        # 배 중심 — latch 발행이라 나중에 떠도 마지막 값을 받는다
        self.create_subscription(
            String, '/ship_survey/pose', self._ship_pose_cb,
            QoSProfile(depth=1,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=QoSReliabilityPolicy.RELIABLE))

        self.pub = self.create_publisher(
            String, self.get_parameter('output_topic').value, 10)

        # ★ 지금 살아있는 이벤트 목록 (2026-08-29).
        #   /map_point 는 **새 이벤트만** 나가므로, 재시작으로 복원한 것은
        #   아무도 모른다. 실제로 젯슨이 4건을 기억하는데 서버에는 1건만
        #   재통보됐다 — 나머지 3건은 "다시 안 멈추는 자리" 인데 대시보드에
        #   없어서 대조조차 못 한다.
        #   latch 로 내보내 websocket_client 가 재연결 때 그대로 알리게 한다.
        #   event_gate 는 이 토픽을 보지 않으므로 로봇 정지에는 영향이 없다.
        self.active_pub = self.create_publisher(
            String, '/event_detection/active',
            QoSProfile(depth=1,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=QoSReliabilityPolicy.RELIABLE))
        self.clear_pub = self.create_publisher(
            String, self.get_parameter('clear_topic').value, 10)
        clear_hz = max(0.1, self.get_parameter('clear_check_hz').value)
        self._clear_dt = 1.0 / clear_hz
        self.create_timer(self._clear_dt, self._check_clear)

        self.get_logger().info(
            "change_point_detector 시작: map->base_link TF 조회 기반 + "
            f"위치 기반 중복 제거(반경 {self.dedup_radius}m, TTL {self.event_ttl.nanoseconds/1e9:.0f}s)"
        )
        self.get_logger().info(f"이벤트 기억 파일: {self.state_file} "
                               "(깨끗이 시작하려면 이 파일을 지울 것)")
        # 복원한 목록도 바로 알린다 — 이게 없으면 websocket_client 가
        # 이번 세션에 새로 본 것만 알아 재통보가 빠진다.
        self._publish_active()

    # ------------------------------------------------------------------
    #  ★ 이벤트 기억을 재시작 너머로 잇는다 (2026-08-27).
    #
    #  이게 없으면 change_point 가 뜰 때마다 목록이 비어서, **같은 자리에
    #  그대로 있는 불을 새 이벤트로 다시 등록하고 로봇을 또 세운다.** 게다가
    #  예전 이벤트는 "치워짐" 을 보낼 주체가 사라져 프론트 핑이 고아가 된다.
    #  오늘 재시작할 때마다 둘 다 겪었다.
    #
    #  TTL 을 넘긴 항목은 복원하지 않는다 — "이만큼 못 봤으면 잊는다" 는
    #  기존 규칙이 재시작 여부와 무관하게 그대로 적용되는 것뿐이다.
    def _restore_state(self):
        try:
            with open(self.state_file, encoding='utf-8') as f:
                saved = json.load(f)
        except FileNotFoundError:
            return
        except Exception as e:
            self.get_logger().warn(f"이벤트 기억 파일을 못 읽었다(빈 상태로 시작): {e}")
            return

        now = self.get_clock().now()
        restored, dropped = [], 0
        for ev in saved:
            try:
                # ★ clock_type 을 반드시 맞춘다 (2026-08-28 실측 버그).
                #   Time(nanoseconds=...) 의 기본은 SYSTEM_TIME 인데 노드의
                #   get_clock().now() 는 ROS_TIME 이다. 둘을 빼면
                #       TypeError: Can't subtract times with different clock types
                #   가 나고, 그걸 아래 except 가 "깨진 항목" 으로 삼켜서
                #   **모든 이벤트가 조용히 버려졌다.** 기억 파일이 한 번도
                #   복원된 적이 없었고, 재시작할 때마다 같은 불이 새 이벤트로
                #   다시 등록돼 프론트에 핑이 겹쳐 쌓였다.
                last_seen = Time(nanoseconds=int(ev['last_seen_ns']),
                                 clock_type=now.clock_type)
                if (now - last_seen) >= self.event_ttl:
                    dropped += 1
                    continue
                # ★ 판정 진행상황은 되살리지 않는다 (2026-08-29 실측 사고).
                #   종료 전 값을 그대로 되살리면 켜자마자 치워짐이 나간다:
                #     now - last_seen  = 꺼져 있던 시간(수 분) >= min_unseen_s
                #     max_departure    = 지난 세션에 쌓인 큰 값 >= min_departure_m
                #     fov_seen         = 3초 이상이면 그대로 통과
                #   -> 첫 _check_clear 에서 모든 조건이 참이 되어, 불이 눈앞에
                #      있는데도 시동 몇 초 만에 "치워졌다" 를 보냈다.
                #      min_unseen_s 도 min_departure_m 도 무력화된다.
                #
                #   되살릴 것은 "이 자리를 이미 보고했다" 는 사실뿐이다.
                #   치워졌는지는 **이번 세션에 다시 확인해야 한다.**
                #   last_seen 을 지금으로 두면 로봇이 한 바퀴 돌아 실제로
                #   못 볼 때까지 기다린다 — 불이 진짜 없으면 그때 치워진다.
                restored.append({
                    'class_id': ev['class_id'],
                    'event_id': ev['event_id'],
                    'x': float(ev['x']), 'y': float(ev['y']),
                    'last_seen': now,        # 판정 시계를 지금부터 다시
                    'seen_from': tuple(ev['seen_from']),
                    'seen_yaw': float(ev['seen_yaw']),
                    'left_vantage': False,   # 이번 세션에 다시 벗어나야 함
                    'arrived_at': None,
                    'fov_seen': 0.0,
                    'max_departure': 0.0,    # 이번 세션에 다시 멀어졌다 와야 함
                })
            except Exception:
                dropped += 1
        self.reported_events = restored
        if restored:
            ids = ', '.join(e['event_id'] for e in restored)
            self.get_logger().warn(
                f"이전 세션 이벤트 {len(restored)}건 복원 — 이 자리들은 다시 "
                f"멈추지 않는다: {ids}")
            self.get_logger().warn(
                "   치워졌는지는 이번 세션에 다시 확인한다 "
                "(한 바퀴 돌며 못 보면 그때 치워짐 발행)")
        if dropped:
            self.get_logger().info(f"오래되거나 깨진 항목 {dropped}건은 버렸다")

    def _publish_active(self):
        """살아있는 이벤트 목록을 latch 로 알린다 (위 active_pub 주석 참고)."""
        try:
            out = [{
                'class_id': e['class_id'],
                'event_id': e['event_id'],
                'map_x': e['x'], 'map_y': e['y'],
                'confidence': e.get('confidence', 0.0),
            } for e in self.reported_events]
            m = String()
            m.data = json.dumps(out)
            self.active_pub.publish(m)
        except Exception as e:
            self.get_logger().warn(f"활성 목록 발행 실패(무시): {e}")

    def _save_state(self):
        """목록이 바뀔 때마다 저장. 이벤트는 드물어서 비용은 무시할 만하다."""
        try:
            data = [{
                'class_id': e['class_id'], 'event_id': e['event_id'],
                'x': e['x'], 'y': e['y'],
                'last_seen_ns': e['last_seen'].nanoseconds,
                'seen_from': list(e['seen_from']), 'seen_yaw': e['seen_yaw'],
                'left_vantage': e['left_vantage'], 'arrived_at': e['arrived_at'],
                'fov_seen': e.get('fov_seen', 0.0),
                'confidence': e.get('confidence', 0.0),
                'max_departure': e.get('max_departure', 0.0),
            } for e in self.reported_events]
            os.makedirs(os.path.dirname(self.state_file), exist_ok=True)
            # 원자적 교체 — 쓰는 도중에 죽어도 파일이 깨지지 않는다.
            # (이 노드는 오늘 실제로 두 번 죽었다.)
            fd, tmp = tempfile.mkstemp(dir=os.path.dirname(self.state_file))
            with os.fdopen(fd, 'w', encoding='utf-8') as f:
                json.dump(data, f)
            os.replace(tmp, self.state_file)
        except Exception as e:
            self.get_logger().warn(f"이벤트 기억을 저장 못 했다(계속 진행): {e}")

    # ------------------------------------------------------------------
    def _amcl_cb(self, _msg):
        """AMCL 이 한 번이라도 위치를 내면 heading 이 보정된 것으로 본다."""
        if self._amcl_ready:
            return
        self._amcl_ready = True
        self.get_logger().warn(
            "✅ AMCL 확인 — 이제부터 이벤트를 등록한다 "
            "(그 전에는 heading 보정이 없어 좌표를 믿을 수 없다)")

    # ------------------------------------------------------------------
    def _inbound_cb(self, msg):
        """서버에서 온 것을 그대로 받는다. reset_events 만 우리 관심사다.

        ★ 왜 필요한가 (2026-08-29)

        유령 핑이나 잘못된 위치의 이벤트가 생기면 지금은 로컬라이제이션을
        껐다 켜야 했다. 그런데 기억 파일까지 지워야 진짜 깨끗해지고, 그 사이
        순찰도 끊긴다. 관제사가 화면에서 한 번에 정리할 수 있어야 한다.

        기억을 비우면 그 자리의 불을 **다시 새 이벤트로 보고하고 로봇도 다시
        멈춘다.** 이는 안전한 방향의 실수다(놓치는 것이 아니라 한 번 더 알림).
        그래서 이 명령이 잘못 와도 위험하지 않다.
        """
        try:
            d = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if not isinstance(d, dict) or d.get('event_type') != 'reset_events':
            return

        n = len(self.reported_events)
        ids = ', '.join(e['event_id'] for e in self.reported_events)
        self.reported_events = []
        self._pending = []
        self._save_state()
        self._publish_active()
        self.get_logger().warn(
            f"🧹 이벤트 기억 초기화 — {n}건 지움" + (f": {ids}" if ids else ""))
        self.get_logger().warn(
            "   이제부터 같은 자리의 위험도 새 이벤트로 보고하고 다시 멈춘다")

    # ------------------------------------------------------------------
    def _ship_pose_cb(self, msg):
        """배 중심 좌표. 배에서 먼 검출을 거르는 데만 쓴다."""
        try:
            xy = json.loads(msg.data)['map_xy']
            self._ship_center = (float(xy[0]), float(xy[1]))
        except (ValueError, TypeError, KeyError, IndexError) as e:
            self.get_logger().warn(f"ship_pose 파싱 실패(거르기 비활성): {e}")
            return
        self.get_logger().info(
            f"배 중심 수신 ({self._ship_center[0]:.2f}, "
            f"{self._ship_center[1]:.2f}) — 여기서 "
            f"{self.max_dist_from_ship:.1f}m 초과 검출은 버린다")

    # ------------------------------------------------------------------
    def _find_matching_event(self, class_id, map_x, map_y):
        """같은 클래스이면서 반경 안에 있는 기존 이벤트를 찾아 반환 (없으면 None)."""
        for ev in self.reported_events:
            if ev['class_id'] != class_id:
                continue
            dist = math.hypot(map_x - ev['x'], map_y - ev['y'])
            if dist < self.dedup_radius:
                return ev
        return None

    def _cleanup_old_events(self, now):
        """일정 시간 이상 재감지가 없었던 이벤트는 목록에서 제거."""
        self.reported_events = [
            ev for ev in self.reported_events
            if (now - ev['last_seen']) < self.event_ttl
        ]

    # ------------------------------------------------------------------
    def _check_clear(self):
        """처음 그 불을 본 로봇 위치로 돌아왔는데 안 보이면 "치워졌다"고 알린다.

        판정 근거는 clear_verdict 참고. 핵심은 **로봇 자신의 위치**를 쓰는
        것이다 — 그 자리에서는 불이 분명히 보였으므로(그래서 검출됐다),
        같은 자리에서 안 보이면 배에 가려진 게 아니라 진짜 없는 것이다.

        판정이 나면 목록에서 지운다. 같은 자리에 나중에 새로 불을 놓으면
        완전히 새 이벤트로 다시 보고돼야 하기 때문이다.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time(), timeout=self.tf_timeout)
        except Exception:
            return   # 위치추정이 아직 없다 — 다음 주기에 다시 시도
        rx = transform.transform.translation.x
        ry = transform.transform.translation.y
        q = transform.transform.rotation
        robot_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

        # ★ last_seen 갱신을 주기적으로 파일에 반영한다. 등록/치워짐 때만
        #   저장하면 저장된 last_seen 이 등록 시각에 머물러, 눈앞에 계속
        #   보이는 불도 재시작 후 TTL 초과로 탈락한다.
        now_s = self.get_clock().now().nanoseconds / 1e9
        if now_s - getattr(self, '_last_save_s', 0.0) >= 10.0:
            self._last_save_s = now_s
            if self.reported_events:
                self._save_state()

        survivors = []
        for ev in self.reported_events:
            # ★ 판정 전에 따로 붙잡아 둔다 (2026-08-27 실측 사고).
            #   clear_verdict 는 '치워짐' 을 확정하면서 arrived_at 을 None 으로
            #   되돌린다(다음 방문을 위해). 그런데 아래 로그가 그 값을 그대로
            #   빼서 TypeError 로 **노드가 통째로 죽었다.** 치워짐이 처음
            #   성공하는 순간 죽으므로, 그 뒤로는
            #     - map_point 가 안 나가 로봇이 불을 봐도 안 멈추고
            #     - cleared 가 안 나가 프론트 핑이 영영 안 지워진다.
            #   증상이 인지 문제처럼 보여서 원인을 두 번 놓쳤다.
            watched_since = ev['arrived_at']
            # ★ fov_seen 도 clear 확정 시 0.0 으로 되돌아온다. arrived_at 과
            #   똑같이 **덮어쓰기 전에** 붙잡아야 로그가 진짜 값을 찍는다.
            #   (2026-08-27: 이걸 빠뜨려 판정 근거가 항상 '0.0초' 로 찍혔다.
            #    믿을 수 없는 진단 로그는 없느니만 못하다.)
            fov_before = ev['fov_seen']
            in_fov = in_camera_fov(robot_yaw, self.cam_yaw, self.hfov,
                                   rx, ry, ev['x'], ev['y'])
            dist_v = math.hypot(rx - ev['seen_from'][0], ry - ev['seen_from'][1])
            # 등록 이후 실제로 얼마나 멀어졌었는지 기억한다 (되돌아오지 않음)
            if dist_v > ev.get('max_departure', 0.0):
                ev['max_departure'] = dist_v
            verdict, ev['left_vantage'], ev['arrived_at'], ev['fov_seen'] = clear_verdict(
                dist_v,
                angle_diff(robot_yaw, ev['seen_yaw']),
                ev['last_seen'].nanoseconds / 1e9, now_s,
                self.revisit_radius, self.revisit_yaw_tol, self.revisit_grace,
                ev['left_vantage'], ev['arrived_at'],
                in_fov, ev['fov_seen'], self._clear_dt,
                ev.get('max_departure', 0.0), self.min_departure,
                self.min_unseen)

            if verdict != 'clear':
                survivors.append(ev)
                continue

            out = {
                'class_id': ev['class_id'],
                'event_id': ev['event_id'],
                'map_x': ev['x'],
                'map_y': ev['y'],
            }
            msg = String(); msg.data = json.dumps(out)
            self.clear_pub.publish(msg)
            watched = ('%.1f초' % (now_s - watched_since)
                       if watched_since is not None else '(체류시간 불명)')
            # 왜 지웠는지 숫자로 남긴다 — 오판이 나면 이 줄만 보면 된다
            self.get_logger().info(
                f"  근거: 카메라가 그쪽을 본 시간 {fov_before:.1f}초"
                f"(>={self.revisit_grace}초 필요), 구역 체류 {watched}, "
                f"마지막 검출은 {now_s - ev['last_seen'].nanoseconds/1e9:.1f}초 전"
                f"(>={self.min_unseen:.0f}초 필요), "
                f"최대 이탈거리 {ev.get('max_departure', 0.0):.2f}m"
                f"(>={self.min_departure:.1f}m 필요)")
            self.get_logger().info(
                f"[{ev['class_id']}] 치워짐 확인 — event_id={ev['event_id']} "
                f"(처음 본 자리로 돌아와 {watched} 지켜봤는데 안 보임)")
            # survivors 에 안 넣는다 -> 목록에서 제거됨

        if len(survivors) != len(self.reported_events):
            self.reported_events = survivors
            self._save_state()
            self._publish_active()
        else:
            self.reported_events = survivors

    # ------------------------------------------------------------------
    def _detection_cb(self, msg: String):
        try:
            det = json.loads(msg.data)
            u = float(det['u'])
            v = float(det['v'])
            depth = float(det['depth'])
            class_id = det.get('class_id', 'unknown')
            confidence = det.get('confidence', None)
        except (json.JSONDecodeError, KeyError, ValueError) as e:
            self.get_logger().warn(f"감지 메시지 파싱 실패: {e}")
            return

        if depth <= 0.0:
            self.get_logger().debug("depth<=0, 무효 감지 스킵")
            return

        if depth > self.max_depth:
            self.get_logger().debug(
                f"[{class_id}] depth {depth:.2f}m > {self.max_depth}m — "
                f"쓰레기 depth 로 보고 버린다")
            return

        # ★ 2026-08-24: confidence 필터. 실측 0.468짜리 한 프레임 오탐이
        #   그대로 새 이벤트(정지 유발)로 등록된 사고가 있었다. YOLO
        #   confidence 는 프레임마다 크게 흔들리는 게 이미 확인된
        #   문제라(같은 세션 실측 0.026~0.201), 낮은 신뢰도 감지는 아예
        #   위치 계산·매칭 단계까지 안 보낸다.
        if confidence is not None and confidence < self.min_confidence:
            self.get_logger().debug(
                f"[{class_id}] confidence {confidence:.2f} < {self.min_confidence}, 무시")
            return

        # --- 1) (u, v, depth) -> 카메라 좌표계 ---
        focal_px = (self.image_width / 2.0) / math.tan(self.hfov / 2.0)
        cx = self.image_width / 2.0
        angle = math.atan2(u - cx, focal_px)

        if self.depth_is_radial:
            x_cam = depth * math.sin(angle)
            z_cam = depth * math.cos(angle)
        else:
            z_cam = depth
            x_cam = depth * math.tan(angle)

        # --- 2) 카메라 좌표계 -> base_link 좌표계 ---
        # 카메라가 로봇 정면과 다른 방향(cam_yaw)을 보도록 장착된 경우를 위해
        # 2D 회전을 먼저 적용한 뒤 오프셋을 더한다.
        # 카메라 기준 "전방"은 z_cam, "좌측"은 -x_cam (OpenCV: x=우측이므로 좌측=-x_cam)
        cam_local_x = z_cam
        cam_local_y = -x_cam
        cos_yaw = math.cos(self.cam_yaw)
        sin_yaw = math.sin(self.cam_yaw)
        rotated_x = cam_local_x * cos_yaw - cam_local_y * sin_yaw
        rotated_y = cam_local_x * sin_yaw + cam_local_y * cos_yaw

        local_x = rotated_x + self.cam_offset[0]
        local_y = rotated_y + self.cam_offset[1]
        local_z = self.cam_offset[2]

        # --- 3) base_link -> map 변환 (TF 조회) ---
        point_in_base = PointStamped()
        point_in_base.header.frame_id = self.base_frame
        point_in_base.header.stamp = msg.header.stamp if hasattr(msg, 'header') else self.get_clock().now().to_msg()
        point_in_base.point.x = local_x
        point_in_base.point.y = local_y
        point_in_base.point.z = local_z

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame,
                Time(),
                timeout=self.tf_timeout)
        except Exception as e:
            self.get_logger().warn(f"TF 조회 실패 ({self.map_frame}<-{self.base_frame}): {e}")
            return

        point_in_map = tf2_geometry_msgs.do_transform_point(point_in_base, transform)
        map_x = point_in_map.point.x
        map_y = point_in_map.point.y

        # ★ 이 검출을 낸 순간의 로봇 위치·방향. 새 이벤트를 만들 때
        #   "이 자리에서는 불이 분명히 보였다"는 기준점으로 저장한다
        #   (clear_verdict 참고).
        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y
        rq = transform.transform.rotation
        robot_yaw = quat_to_yaw(rq.x, rq.y, rq.z, rq.w)

        # --- ★ AMCL 전에는 좌표를 믿을 수 없다 (위 require_amcl 주석 참고) ---
        if not self._amcl_ready:
            self.get_logger().warn(
                f"[{class_id}] AMCL 아직 없음 — 등록 보류 "
                "(Nav2 를 켜면 heading 이 보정되고 그때부터 등록한다)",
                throttle_duration_sec=10.0)
            return

        # --- ★ 배에서 먼 검출은 버린다 (위 max_dist_from_ship_m 주석 참고) ---
        if self._ship_center is not None and self.max_dist_from_ship > 0:
            d_ship = math.hypot(map_x - self._ship_center[0],
                                map_y - self._ship_center[1])
            if d_ship > self.max_dist_from_ship:
                self.get_logger().info(
                    f"[{class_id}] 배 중심에서 {d_ship:.2f}m — "
                    f"{self.max_dist_from_ship:.1f}m 초과라 버림 "
                    f"(map={map_x:.2f},{map_y:.2f})",
                    throttle_duration_sec=5.0)
                return

        # --- ★ 2차 필터: map 좌표 기준 중복 제거 ---
        now = self.get_clock().now()
        self._cleanup_old_events(now)

        existing = self._find_matching_event(class_id, map_x, map_y)
        if existing is not None:
            # 이미 보고된 이벤트 → 재발행하지 않고, "최근에 봤다"는 시각만 갱신
            existing['last_seen'] = now
            self.get_logger().debug(
                f"[{class_id}] 중복 이벤트로 판단 (기존 위치와 "
                f"{math.hypot(map_x - existing['x'], map_y - existing['y']):.2f}m 이내) - 재발행 안 함"
            )
            return

        # --- ★ 3차 필터: 새 이벤트는 연속 확인 후에만 등록 (2026-08-27) ---
        #   같은 불을 110초간 12번 재보니 흩어짐은 중앙값 0.04m / 최대 0.10m 로
        #   아주 안정적이었다(순찰 원 정반대편에서 봐도 4cm). 그런데 가끔
        #   depth 가 튀어 0.5~1.3m 벗어난 단발 오측정이 나오고, 그 하나가
        #   곧바로 새 이벤트로 등록되어 **로봇을 세웠다**. 실측 오측정:
        #       정상 (-0.21,-1.16) -> 오측정 (-0.71,-0.97) / (-0.51,-2.24)
        #   yolo 쪽 연속 3프레임 확인은 **화면 좌표** 기준이라 depth 튐을
        #   못 거른다. 그래서 map 좌표로 한 번 더 확인한다.
        #   dedup_radius 를 키우는 건 답이 아니다 — 정상 측정이 4cm 밖에
        #   안 흩어지므로, 키우면 진짜 이동만 놓치게 된다.
        now_f = now.nanoseconds / 1e9
        self._pending = [q for q in self._pending
                         if now_f - q['first'] <= self.new_window]
        cand = next((q for q in self._pending
                     if q['class_id'] == class_id
                     and math.hypot(map_x - q['x'], map_y - q['y']) < self.dedup_radius),
                    None)
        if cand is None:
            self._pending.append({'class_id': class_id, 'x': map_x, 'y': map_y,
                                  'first': now_f, 'count': 1})
            cand = self._pending[-1]
        else:
            cand['count'] += 1
            # 평균으로 다듬어 등록 좌표를 안정시킨다
            k = cand['count']
            cand['x'] += (map_x - cand['x']) / k
            cand['y'] += (map_y - cand['y']) / k
        if cand['count'] < self.new_confirm:
            self.get_logger().info(
                f"[{class_id}] 새 이벤트 후보 {cand['count']}/{self.new_confirm} "
                f"map=({map_x:.2f}, {map_y:.2f}) - 한 번 더 봐야 등록한다")
            return
        map_x, map_y = cand['x'], cand['y']
        self._pending.remove(cand)

        # ★ event_id 는 이 위치에 처음 보고된 좌표로 고정한다(반올림 좌표).
        #   재검출마다 map_x/map_y 가 몇 cm씩 흔들려도 프론트가 같은 핑으로
        #   식별할 수 있어야 하기 때문이다. 프론트엔드에 이 형식 그대로
        #   전달하기로 합의했다: "<class_id>@<x>,<y>" (소수 2자리).
        event_id = f"{class_id}@{map_x:.2f},{map_y:.2f}"

        # 새 이벤트로 확정 → 기록하고 발행
        self.reported_events.append({
            'class_id': class_id,
            'event_id': event_id,
            'x': map_x,
            'y': map_y,
            'confidence': confidence,
            'last_seen': now,
            'seen_from': (robot_x, robot_y),   # 이 불을 처음 본 로봇 위치
            'seen_yaw': robot_yaw,             # 그때 로봇이 보던 방향
            'left_vantage': False,             # 그 자리를 한 번 벗어났는지
            'arrived_at': None,                # 이번에 그 자리에 도착한 시각
            'fov_seen': 0.0,                   # 이번 방문에서 카메라가 실제로 그쪽을 본 시간
            'max_departure': 0.0,              # 등록 이후 seen_from 에서 벌어진 최대 거리
        })

        position_uncertainty_m = self._estimate_position_uncertainty()

        out = {
            'stamp': self.get_clock().now().to_msg().sec,
            'class_id': class_id,
            'event_id': event_id,
            'confidence': confidence,
            'map_x': map_x,
            'map_y': map_y,
            'depth': depth,
            'position_uncertainty_m': position_uncertainty_m,
        }
        out_msg = String()
        out_msg.data = json.dumps(out)
        self.pub.publish(out_msg)

        self.get_logger().info(
            f"[{class_id}] 새 이벤트 발행: map=({map_x:.2f}, {map_y:.2f}) "
            f"event_id={event_id}"
        )
        self._save_state()
        self._publish_active()

    # ------------------------------------------------------------------
    def _estimate_position_uncertainty(self) -> float:
        return 0.15  # meters, placeholder


def main(args=None):
    rclpy.init(args=args)
    node = ChangePointDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
