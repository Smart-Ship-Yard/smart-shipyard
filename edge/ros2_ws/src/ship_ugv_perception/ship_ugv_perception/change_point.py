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

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401  (PointStamped 변환을 위해 필요한 등록)


def clear_verdict(dist_m, last_seen_s, now_s,
                  clear_radius_m, clear_watch_s, first_seen_s, min_age_s):
    """치워짐 판정의 순수 핵심부. ROS 없이 검증하려고 뽑아냈다.

    반환: 'clear' 또는 'wait'.

    판정은 딱 세 가지만 본다:
      · 로봇이 판정할 만큼 가까운가        (dist_m <= clear_radius_m)
      · 이벤트가 판정할 만큼 늙었는가      (now - first_seen >= min_age_s)
      · 마지막으로 본 지 충분히 지났는가   (now - last_seen >= clear_watch_s)

    ★ 왜 이렇게 단순해졌나 (2026-08-24) ★
    예전에는 range_entered_at("이번 관찰을 언제 시작했나")을 따로 들고
    다니며 last_seen 과 비교했다. 그런데 그 값은 **한 번 켜지면 다시
    켜지지 않아서**, 그 뒤로 검출이 한 번이라도 있으면
    last_seen >= range_entered_at 이 영원히 참이 되어 clear 가 절대
    발동하지 못했다(실측: 불을 치워도 cleared 가 안 나감). 유일한 재무장
    경로가 "FOV 이탈 시 리셋"이었는데 그건 헤딩 오차에 따라 불규칙해서
    clear_watch_s 를 채우지 못했다.

    "마지막으로 본 지 얼마나 됐나"만 보면 재무장이 필요 없다. 불이
    그대로 있으면 검출될 때마다 last_seen 이 갱신돼 판정이 계속 밀리고,
    치우면 갱신이 멈춰 자연히 시간이 쌓인다. 배에 가려 안 보이는
    구간(실측 약 30초)은 clear_watch_s(60초)보다 짧아 그대로 넘어간다.

    그 이전에 쓰던 has_left_once("반경을 한 번은 벗어나야 판정 시작")도
    같은 이유로 걷어냈다. 그 조건은 순찰 기하에 의존해서, 반지름 1.00 에
    불이 중심 0.30m 면 로봇~불 거리가 0.70~1.30m 라 clear_radius 2.0 을
    영영 못 벗어나 clear 가 아예 발동 못 했다.
    """
    if dist_m > clear_radius_m:
        return 'wait'
    if (now_s - first_seen_s) < min_age_s:
        return 'wait'
    if (now_s - last_seen_s) < clear_watch_s:
        return 'wait'
    return 'clear'


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
        #   -> 근본 원인이 FOV 게이트로 해결됐으므로 0.5 로 되돌린다.
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
        self.declare_parameter('clear_radius_m', 2.0)    # 이 반경 안이면 "지나간다"로 본다
        #   ★ 2026-08-24 4차 (실측으로 결론). 5초로 줄였더니 한 바퀴마다
        #   오판이 났다. 원인은 **배가 불을 가리는 시간**이다:
        #       20:53:40 검출 끊김(배 뒤로 들어감) -> 20:54:10 재개
        #       = 가림 구간 약 30초, 한 바퀴 약 50초
        #   in_camera_fov 는 각도만 보므로 가림을 모른다. 가림과 진짜
        #   없어짐은 **더 긴 시간으로만** 구분된다 — 가려진 것은 다음
        #   바퀴에 반드시 다시 보이고, 치운 것은 영영 안 보인다.
        #   그래서 실측 가림 시간(30초)의 2배인 60초로 잡는다.
        #   대가: 불을 진짜 치웠을 때 핑이 사라지기까지 최대 1분쯤 걸린다.
        #   그 대신 "가려졌다고 핑이 깜빡 사라지는" 일이 없어진다.
        self.declare_parameter('clear_watch_s', 60.0)    # 그 안에서 이만큼 재감지가 없으면 지운다
        # ★ 2026-08-24: confidence 필터가 아예 없어서 저신뢰도(실측 0.468)
        #   한 프레임짜리 오탐도 그대로 새 이벤트로 등록 -> 불필요한 정지를
        #   유발했다. websocket_client 가 이미 쓰는 기준(0.5)과 맞춘다.
        self.declare_parameter('min_confidence', 0.5)
        # ★ 2026-08-24: depth 상한이 아예 없어서 쓰레기 depth 가 그대로
        #   map 좌표로 투영되고 있었다. 실측(21:42):
        #       로봇(-0.78,-0.42) -> 불 추정(+5.13,-2.52), 계산상 6.27m
        #       맵 경계가 x:-4.60~4.95 인데 x=5.13 은 맵 밖이다.
        #   같은 시간대 로봇 위치(ekf_global)는 18개 샘플 전부 순찰중심에서
        #   0.93~1.14m 로 궤도에 정확히 붙어 있었다 — 즉 로컬라이제이션은
        #   멀쩡하고, YOLO 가 멀리 있는 무언가를 fire 로 잡아(conf 0.64~0.93)
        #   ROI 중앙값 depth 가 5~6m 로 나온 것이 원인이다.
        #   event_gate_node 는 max_trigger_depth_m=2.5 가 있어 정지는 안
        #   했지만, 프론트·DB 로는 다 나가 맵 전체에 유령 핑이 찍혔다.
        #   순찰 반지름 1.0 + 불이 배 위(중심에서 ~0.4m)면 로봇~불 거리는
        #   0.6~1.4m 다. 2.5m 면 넉넉하고, event_gate_node 와 같은 값이라
        #   "정지는 안 하는데 핑만 찍히는" 불일치도 사라진다.
        #   ★ 2026-08-24 2차: 2.5m 도 헐거웠다. 실측:
        #       22:07:32  로봇(-0.31,-1.87) -> 불 추정(+0.56,+0.52)  2.55m
        #       22:08:18  로봇(+0.33,-1.96) -> 불 추정(-0.35,+0.54)  2.59m
        #     로봇이 방 아래쪽에서 위쪽을 보며 2.5m 거리의 무언가를 fire 로
        #     오검출해 2.5m 상한을 아슬아슬하게 통과했다.
        #   진짜 불은 배 위에 있다. 로봇~불 거리의 상한은
        #       순찰 반지름(1.0) + 불이 배 중심에서 떨어진 거리
        #   이고, 불을 배 중심에서 1.0m 까지 옮긴다 해도 2.0m 다.
        #   그래서 2.0 으로 조인다. 순찰 반지름을 키우거나 불을 배에서 더
        #   멀리 두고 시험하려면 이 값도 같이 올려야 한다.
        self.declare_parameter('max_depth_m', 2.0)
        # ★ 2026-08-24: 이벤트가 생긴 지 이만큼은 지나야 클리어 판정 대상이
        #   된다. 예전의 has_left_once(순찰 기하에 의존해 영영 발동 못 하던
        #   조건)를 대체한다 — clear_verdict 주석 참고.
        #   30초는 clear_watch_s 와 합쳐 체감 30~45초가 되어 너무 길었다.
        #   10초면 정지-확인 순간을 덮으면서 체감 지연은 최대 15초다.
        self.declare_parameter('min_event_age_s', 10.0)
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
        self.clear_radius = self.get_parameter('clear_radius_m').value
        self.clear_watch = Duration(seconds=self.get_parameter('clear_watch_s').value)
        self.min_confidence = self.get_parameter('min_confidence').value
        self.min_event_age = self.get_parameter('min_event_age_s').value
        self.max_depth = self.get_parameter('max_depth_m').value

        # ★ 2차 필터: 이미 보고한 이벤트 기록
        # 각 항목: {'class_id': str, 'x': float, 'y': float, 'last_seen': rclpy.time.Time}
        self.reported_events = []

        # ---- TF ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- 통신 ----
        self.create_subscription(
            String, self.get_parameter('detection_topic').value,
            self._detection_cb, 10)
        self.pub = self.create_publisher(
            String, self.get_parameter('output_topic').value, 10)
        self.clear_pub = self.create_publisher(
            String, self.get_parameter('clear_topic').value, 10)
        clear_hz = max(0.1, self.get_parameter('clear_check_hz').value)
        self.create_timer(1.0 / clear_hz, self._check_clear)

        self.get_logger().info(
            "change_point_detector 시작: map->base_link TF 조회 기반 + "
            f"위치 기반 중복 제거(반경 {self.dedup_radius}m, TTL {self.event_ttl.nanoseconds/1e9:.0f}s)"
        )

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
        """이미 보고한 이벤트 자리를 지켜보다가, 안 보인 지 clear_watch_s
        이상 지나면 "치워졌다"고 알린다.

        판정 조건은 clear_verdict 참고. 여기서는 그 앞에 두 가지 게이트를
        더 둔다:
          · 카메라가 실제로 그 방향을 보고 있을 때만 판정한다
            (in_camera_fov — 등 돌린 채 "안 보이네"라고 하면 안 된다)
          · 판정이 나면 목록에서 지운다. 같은 자리에 나중에 새로 불을
            놓으면 완전히 새 이벤트로 다시 보고돼야 하기 때문이다.

        ★ in_camera_fov 는 각도만 본다 — 배에 가려 안 보이는 것(occlusion)은
        모른다. 그래서 가림 구간(실측 약 30초)을 clear_watch_s(60초)로
        덮는다. 자세한 근거는 docs/이벤트_중복제거_튜닝.md 참고.
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

        now = self.get_clock().now()
        now_s = now.nanoseconds / 1e9
        clear_watch_s = self.clear_watch.nanoseconds / 1e9
        survivors = []
        for ev in self.reported_events:
            dist = math.hypot(rx - ev['x'], ry - ev['y'])

            if not in_camera_fov(robot_yaw, self.cam_yaw, self.hfov,
                                 rx, ry, ev['x'], ev['y']):
                survivors.append(ev)
                continue

            verdict = clear_verdict(
                dist, ev['last_seen'].nanoseconds / 1e9, now_s,
                self.clear_radius, clear_watch_s,
                ev['first_seen'].nanoseconds / 1e9, self.min_event_age)

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
            self.get_logger().info(
                f"[{ev['class_id']}] 치워짐 확인 — event_id={ev['event_id']} "
                f"(안 보인 지 {now_s - ev['last_seen'].nanoseconds/1e9:.0f}초)")
            # survivors 에 안 넣는다 -> 목록에서 제거됨

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
            'last_seen': now,
            'first_seen': now,          # min_event_age_s 판정용
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
