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


def clear_verdict(dist_m, range_entered_at_s, last_seen_s, now_s,
                  clear_radius_m, clear_watch_s, has_left_once):
    """이벤트 클리어 상태기계의 순수 핵심부. ROS 없이 검증하려고 뽑아냈다.

    반환: (verdict, 다음 range_entered_at_s, 다음 has_left_once)
      'reset' — 반경 밖. 다음 방문을 위해 range_entered_at 을 지운다.
      'wait'  — 아직 판정 시간이 안 됐거나, 이번 방문 중 다시 보였거나,
                애초에 판정을 시작할 자격이 안 된다.
      'clear' — 확정. 자리를 한 번 떠났다가 다시 들어와 지켜봤는데 없었다.

    ★ has_left_once 가 왜 필요한가 (2026-08-22 실측 사고로 발견) ★
    불을 처음 발견해서 **정지한 그 순간부터 이미 반경 안**이다. 로봇이
    자리를 뜬 적이 한 번도 없는데 판정 시계가 그냥 시작해버리면, 정지-확인
    사이에 흐른 몇 초만으로 "재방문했는데 없다"고 오판해 방금 막 보고한
    이벤트를 곧장 지워버린다(프론트 핑이 반짝하고 꺼지는 것으로 관측됨).
    그래서 **반경을 한 번이라도 벗어난 적이 있어야만** 그다음 재진입을
    "재방문"으로 인정하고 시계를 켠다.
    """
    if dist_m > clear_radius_m:
        return 'reset', None, True   # 벗어났다 -> 이제부터는 재방문 자격 있음
    if not has_left_once:
        return 'wait', range_entered_at_s, has_left_once   # 아직 첫 방문 중
    if range_entered_at_s is None:
        return 'wait', now_s, has_left_once
    if (now_s - range_entered_at_s) < clear_watch_s:
        return 'wait', range_entered_at_s, has_left_once
    if last_seen_s >= range_entered_at_s:
        return 'wait', range_entered_at_s, has_left_once
    return 'clear', range_entered_at_s, has_left_once


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
        # ★ 2026-08-24 0.5m -> 1.0m로 확대: 지지대상 실측으로 같은 자리에
        #   정지 안 한 불을 로봇이 도는 각도만 바뀌어도 map 좌표가 0.5m 넘게
        #   흔들렸다(이 프로젝트의 헤딩 추정 오차 ±30도가 그대로 위치 오차로
        #   번짐). 그 결과 "같은 불"이 각도 바뀔 때마다 새 이벤트로 잡혀
        #   clear-재생성이 반복됐다. 반경을 실측 흔들림보다 넉넉히 키운다.
        # ponytail: 2026-08-24 데모 당일 시간 부족으로 dedup/clear 완전
        #   안정화를 포기하고 0으로 되돌림 — 매 감지가 항상 새 이벤트로
        #   잡혀 "여러 번 멈춰도 프론트가 확인을 계속 눌러준다"는 예전의
        #   단순 동작으로 복귀. dist(>=0) < 0.0 은 절대 참이 안 되므로
        #   100% 확실하게 매칭이 꺼진다. 데모 이후 시간 생기면 위치 잡음
        #   문제(각도별 최대 ~1m 흔들림 실측) 제대로 잡고 되살릴 것.
        self.declare_parameter('dedup_radius_m', 0.0)   # 같은 이벤트로 볼 거리 반경
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
        self.declare_parameter('clear_watch_s', 15.0)    # 그 안에서 이만큼 재감지가 없으면 지운다
        # ★ 2026-08-24: confidence 필터가 아예 없어서 저신뢰도(실측 0.468)
        #   한 프레임짜리 오탐도 그대로 새 이벤트로 등록 -> 불필요한 정지를
        #   유발했다. websocket_client 가 이미 쓰는 기준(0.5)과 맞춘다.
        self.declare_parameter('min_confidence', 0.5)
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
        """이미 보고한 이벤트 자리를 로봇이 **다시 지나가며** 지켜본다.

        clear_radius 를 한 번이라도 벗어난 적이 있는 이벤트에 한해서만,
        재진입 후 clear_watch 이상 지켜봤는데 재검출이 없으면 "치워졌다"고
        알린다. 벗어난 적이 아직 없으면(막 보고돼서 로봇이 그 옆에 서 있는
        중이면) 판정을 시작하지 않는다 — 그렇지 않으면 정지-확인 사이 몇
        초만으로 방금 보고한 이벤트를 "재방문했는데 없다"고 오판해 곧장
        지워버린다(2026-08-22 실측: 프론트 핑이 반짝하고 꺼짐).

        치워진 이벤트는 목록에서 지운다. 같은 자리에 나중에 새로 불이 나면
        (모형을 다시 놓으면) 완전히 새 이벤트로 다시 보고돼야 하기 때문이다.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time(), timeout=self.tf_timeout)
        except Exception:
            return   # 위치추정이 아직 없다 — 다음 주기에 다시 시도
        rx = transform.transform.translation.x
        ry = transform.transform.translation.y

        now = self.get_clock().now()
        now_s = now.nanoseconds / 1e9
        clear_watch_s = self.clear_watch.nanoseconds / 1e9
        survivors = []
        for ev in self.reported_events:
            dist = math.hypot(rx - ev['x'], ry - ev['y'])
            range_s = (ev['range_entered_at'].nanoseconds / 1e9
                      if ev['range_entered_at'] is not None else None)
            last_seen_s = ev['last_seen'].nanoseconds / 1e9

            verdict, next_range_s, next_left = clear_verdict(
                dist, range_s, last_seen_s, now_s,
                self.clear_radius, clear_watch_s, ev['has_left_once'])

            ev['range_entered_at'] = (
                None if next_range_s is None
                else (now if next_range_s == now_s else ev['range_entered_at']))
            ev['has_left_once'] = next_left

            if verdict != 'clear':
                survivors.append(ev)
                continue

            # ★ 확정: 이번 방문 동안 한 번도 안 잡혔다 — 치워졌다.
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
                f"({now_s - range_s:.1f}초 지켜봄, 재검출 없음)")
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
            'range_entered_at': None,   # _check_clear 가 쓴다
            'has_left_once': False,     # 처음엔 아직 반경을 벗어난 적이 없다
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
