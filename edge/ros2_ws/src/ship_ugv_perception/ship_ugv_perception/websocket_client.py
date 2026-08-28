#!/usr/bin/env python3
"""
websocket_client.py
--------------------
젯슨 <-> 백엔드 서버 WebSocket 노드. (스펙: 젯슨-서버 통신 스펙 v1.3)

[2026-07-17 스펙 v1.3 반영]
- ① position에 yaw(로봇이 바라보는 방향, 라디안) 추가
  -> /odometry/global의 orientation 쿼터니언에서 yaw 추출.
- ② 위험 이벤트에 map_xy(객체의 map 절대 좌표) 추가
- ekf_global은 디버깅용으로 유지.
- ③ block_level, ④ ship_pose는 변경 없음.

[2026-08-21 위험 이벤트 경로 변경 — 재정지 사고 수정]
- 예전에는 이 노드가 /event_detection/uvd(원본 검출)를 직접 받아 카메라
  좌표(depth_xyz)를 자체적으로 TF 변환해 map_xy 를 만들었다. 위치 기준
  중복 제거가 전혀 없어서, 같은 대상을 로봇이 다시 지나칠 때마다 서버로
  또 전송됐다(정지-확인-재개 후 반바퀴도 못 가 재정지하는 사고로 드러남).
- change_point_detector(change_point.py)가 이미 이 TF 변환 + map 좌표
  기준 중복 제거를 갖고 있었으나, 그동안 ship_survey_node 용으로만
  쓰이고 있었다. 이제 위험 이벤트도 그 출력(/event_detection/map_point,
  이미 중복 제거됨)을 받는다. 그래서 이 노드는 더 이상 TF 를 직접
  조회하지 않는다(카메라 오프셋 파라미터·TF 코드 삭제).
- 새 메시지 event_cleared: change_point_detector 가 로봇이 그 자리를
  다시 지나가며 확인했는데 대상이 없어졌을 때 /event_detection/cleared
  로 알린다. 여기서 서버로 그대로 넘겨 프론트엔드가 해당 핑을 지운다.

[수신 루프 추가]
- 기존에는 송신만 하던 노드였으나, 서버 -> 젯슨 메시지를 받는 수신 루프를 추가함.
- 받은 메시지는 이 노드에서 해석하지 않고, 그대로 /server/inbound
  (std_msgs/String) 토픽으로 발행만 한다. 내용 판단/분기는 Nav2 쪽 노드가
  담당한다. 서버->젯슨 메시지 종류가 앞으로 늘어나도 이 노드는 손댈 필요 없음.
- 송신 로직(_ws_worker, _enqueue 등)과 YOLO 관련 부분은 전혀 건드리지 않음.
  같은 WebSocket 연결 객체에서 별도 스레드(_recv_loop)로 수신만 전담.

공통 규칙: person/helmet은 전송 안 함, conf 0.5 미만 버림, 같은 상황 연사 금지,
timestamp는 서버가 붙이므로 생략.

필요 패키지: pip install websockets --user
"""

import json
from collections import Counter, deque
import math
import queue
import re
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String, Int32
from nav_msgs.msg import Odometry

try:
    from websockets.sync.client import connect as ws_connect
    from websockets.exceptions import ConnectionClosed
except ImportError:
    ws_connect = None
    ConnectionClosed = Exception


DANGER_CLASS_MAP = {
    'fallen_person': 'fallen_person',
    'person_fallen': 'fallen_person',
    'fire': 'fire',
    'no_helmet': 'no_helmet',
    'ship_defect': 'ship_defect',
}


def extract_level(class_name: str):
    match = re.search(r'(\d+)', str(class_name))
    return int(match.group(1)) if match else None


def yaw_from_quaternion(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class WebSocketClient(Node):

    def __init__(self):
        super().__init__('websocket_client')

        self.declare_parameter('server_ws_url', 'ws://192.168.0.5:8000/ws/jetson')
        self.declare_parameter('uvd_topic', '/event_detection/uvd')
        self.declare_parameter('ekf_odom_topic', '/odometry/global')
        self.declare_parameter('ship_pose_input_topic', '/ship_survey/pose')
        self.declare_parameter('position_ping_interval_s', 0.5)
        self.declare_parameter('min_confidence', 0.5)
        # ★ 재시도 간격과 로그 주기는 별개다.
        #   간격을 늘려 로그를 줄이면 서버가 살아난 뒤에도 그만큼 데이터가 끊긴다.
        #   빠르게 재시도하되(3초) 실패 로그만 묶어서 찍는다(15초).
        self.declare_parameter('reconnect_interval_s', 3.0)
        self.declare_parameter('fail_log_interval_s', 15.0)
        self.declare_parameter('block_id', 'B1')
        self.declare_parameter('block_level_stability_s', 3.0)
        # ★ 다수결 투표로 바꿨다 (2026-08-27). 아래 _handle_block_level 참고.
        self.declare_parameter('block_level_vote_ratio', 0.6)
        self.declare_parameter('block_level_min_samples', 6)

        # ★ 2026-08-21: 카메라 오프셋·TF 파라미터를 지웠다. map_xy 변환은
        #   change_point_detector 가 전담하고, 이 노드는 그 결과만 받는다.
        self.declare_parameter('map_point_topic', '/event_detection/map_point')
        self.declare_parameter('cleared_topic', '/event_detection/cleared')
        self.declare_parameter('snapshot_topic', '/event_detection/snapshot')

        # ★ 수신 루프 관련
        self.declare_parameter('inbound_topic', '/server/inbound')
        self.declare_parameter('recv_timeout_s', 1.0)

        # ★ 확정된 조립 단계를 ROS 토픽으로도 내보낸다.
        #   ship_survey_node가 "단계 바뀌었으니 배를 다시 측량하라"는 신호로 쓴다.
        #   같은 판정 로직을 저쪽에 또 짜지 않는 이유: 프레임마다 흔들리는 YOLO를
        #   3초 안정화로 거르는 로직(_handle_block_level)이 이미 여기 있고, 두 벌이
        #   되면 서버가 아는 단계와 측량이 아는 단계가 어긋나는 순간이 생긴다.
        self.declare_parameter('block_level_topic', '/block_level/confirmed')

        self.server_url = self.get_parameter('server_ws_url').value
        uvd_topic = self.get_parameter('uvd_topic').value
        ekf_topic = self.get_parameter('ekf_odom_topic').value
        ship_pose_topic = self.get_parameter('ship_pose_input_topic').value
        self.ping_interval = self.get_parameter('position_ping_interval_s').value
        self.min_confidence = self.get_parameter('min_confidence').value
        self.reconnect_interval = self.get_parameter('reconnect_interval_s').value
        self.fail_log_interval = self.get_parameter('fail_log_interval_s').value
        self._last_fail_log = 0.0    # 0 이면 다음 실패를 즉시 찍는다
        self._fail_count = 0
        self.block_id = self.get_parameter('block_id').value
        self.block_level_stability = Duration(
            seconds=self.get_parameter('block_level_stability_s').value)

        inbound_topic = self.get_parameter('inbound_topic').value
        self.recv_timeout = self.get_parameter('recv_timeout_s').value

        if ws_connect is None:
            self.get_logger().error(
                "websockets 미설치. 'pip install websockets --user' 후 재실행."
            )

        self.latest_ekf_global = None
        self.latest_yaw = None
        self._ekf_lock = threading.Lock()
        self._ping_count = 0

        self._level_window = deque()      # (시각, level) — 최근 창
        self._level_vote_ratio = float(
            self.get_parameter('block_level_vote_ratio').value)
        self._level_min_samples = int(
            self.get_parameter('block_level_min_samples').value)
        self._level_confirmed = None
        self._level_lock = threading.Lock()

        self.send_queue = queue.Queue()
        # ★ 마지막 배 위치. 재연결할 때 다시 보내려고 들고 있는다(2026-08-20).
        #   ship_pose 는 측량이 끝나는 그 순간 1회만 발행된다. 그래서 그 뒤에
        #   백엔드가 재시작하고 DB가 비어 있으면 배 위치를 영영 못 받는다.
        #   block_level 은 이미 재연결마다 다시 보내고 있었다. 같이 맞춘다.
        #   (참조 대입 하나라 CPython 에서는 락 없이도 원자적이다)
        self._last_ship_pose = None

        # ★ 위치 핑은 큐에 넣지 않는다 (2026-08-29).
        #   서버가 꺼져 있는 동안에도 0.5초마다 만들어지므로 큐에 넣으면
        #   1시간 끊김 = 7,200건이 쌓였다가 재연결 순간 한꺼번에 쏟아진다.
        #   서버와 프론트가 과거 좌표를 순서대로 재생하게 된다.
        #   **위치는 최신 하나만 의미가 있으므로** 슬롯 하나로 덮어쓴다.
        #   (이벤트는 하나도 버리면 안 되므로 큐를 그대로 쓴다)
        self._pending_position = None

        # ★ 지금 현장에 살아있는 위험 이벤트 거울 (2026-08-29).
        #   서버가 재시작하면 프론트가 빈 화면으로 시작하는데, 로봇은
        #   change_point 가 이미 기억하고 있어 **재발행을 하지 않는다.**
        #   그래서 불이 눈앞에 있는데 화면에는 아무것도 없는 상태가
        #   **무기한** 이어진다 (event_ttl 은 재검출마다 갱신되므로 만료되지
        #   않는다). 재연결할 때 다시 알려서 화면을 진실과 맞춘다.
        #   조립 단계·배 위치가 이미 쓰는 방식과 같다.
        #
        #   change_point 는 건드리지 않는다 — 로봇이 멈추는 판단
        #   (/map_point -> event_gate)에는 영향이 없어야 하기 때문이다.
        self._active_events = {}          # event_id -> payload
        self._active_lock = threading.Lock()

        # ★ 서버 -> 젯슨 수신 메시지를 그대로 중계할 퍼블리셔 (해석하지 않음)
        self.inbound_pub = self.create_publisher(String, inbound_topic, 10)

        # ★ 확정된 조립 단계 퍼블리셔. 단계가 바뀔 때만 발행하므로 latched로 둬야
        #   나중에 뜨는 ship_survey_node가 현재 단계를 즉시 알 수 있다.
        level_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.block_level_pub = self.create_publisher(
            Int32, self.get_parameter('block_level_topic').value, level_qos)

        self.create_subscription(String, uvd_topic, self._uvd_cb, 10)
        self.create_subscription(
            String, self.get_parameter('map_point_topic').value,
            self._map_point_cb, 10)
        self.create_subscription(
            String, self.get_parameter('cleared_topic').value,
            self._cleared_cb, 10)
        # ★ 이벤트 스냅샷 (2026-08-27). yolo_depth_publisher 가 **새 이벤트에
        #   대해서만** 인코딩해 보내준다. 프론트가 핑을 눌렀을 때 "그때 무슨
        #   일이 있었나" 를 보여줄 사진 한 장이다.
        self.create_subscription(
            String, self.get_parameter('snapshot_topic').value,
            self._snapshot_cb, 10)
        self.create_subscription(Odometry, ekf_topic, self._ekf_cb, 10)
        # ★ ship_pose는 ship_survey_node가 측량 끝날 때 딱 1번만 발행하고, 그 시점은
        #   매핑 랩 도중이라 이 노드보다 먼저 끝나 있을 수 있다. 기본 QoS(VOLATILE)로
        #   구독하면 나중에 떠서 그 1회를 통째로 놓치고, 배 위치가 서버에 영영 안 간다.
        #   (발행자만 TRANSIENT_LOCAL이면 부족하고, 구독자도 맞춰야 과거 값을 받는다.)
        ship_pose_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            String, ship_pose_topic, self._ship_pose_cb, ship_pose_qos)

        self.create_timer(self.ping_interval, self._position_ping_cb)

        self._stop_event = threading.Event()
        self._ws_thread = threading.Thread(target=self._ws_worker, daemon=True)
        self._ws_thread.start()

        self.get_logger().info(
            f"websocket_client 시작 (스펙 v1.3 + 수신 루프): server={self.server_url}, "
            f"ekf_topic={ekf_topic}, ping={self.ping_interval}s, "
            f"min_conf={self.min_confidence}, inbound_topic={inbound_topic}"
        )

    def _ekf_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        with self._ekf_lock:
            self.latest_ekf_global = [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
            ]
            self.latest_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def _get_ekf_state(self):
        with self._ekf_lock:
            ekf = list(self.latest_ekf_global) if self.latest_ekf_global else None
            return ekf, self.latest_yaw

    def _position_ping_cb(self):
        ekf_global, yaw = self._get_ekf_state()
        if ekf_global is None or yaw is None:
            return

        # ★ 큐가 아니라 슬롯에 덮어쓴다 (위 _pending_position 주석 참고)
        self._pending_position = {
            'event_type': 'position',
            'ekf_global': ekf_global,
            'yaw': yaw,
        }

        self._ping_count += 1
        if self._ping_count % 10 == 0:
            self.get_logger().info(f"[위치핑] ekf_global={ekf_global} yaw={yaw:.3f}")

    def _uvd_cb(self, msg: String):
        """원본 검출 스트림 — 이제 block_level(조립 단계) 판정에만 쓴다.

        위험 이벤트(fire 등)는 위치 기준 중복 제거를 거친
        /event_detection/map_point 로 옮겼다(_map_point_cb 참고). 이 콜백이
        원본을 그대로 위험 이벤트로 넘기면 중복 제거가 무의미해진다.
        """
        try:
            det = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"/uvd 파싱 실패: {e}")
            return

        class_id = str(det.get('class_id', ''))
        if class_id in DANGER_CLASS_MAP:
            return   # map_point 경로가 처리한다

        level = extract_level(class_id)
        if level is not None:
            self._handle_block_level(class_id, level)

    def _snapshot_cb(self, msg: String):
        """이벤트 스냅샷을 서버로 그대로 넘긴다.

        서버는 이 base64 를 파일로 저장하고 DB 에는 경로만 남긴다
        (docs 협의: 이벤트 문서에 이미지를 통째로 넣으면 재접속 복원 때마다
        이미지가 전부 다시 흐른다).
        """
        try:
            d = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not d.get('image_b64'):
            return
        self._enqueue({
            'event_type': 'event_snapshot',
            'block_id': self.block_id,
            'event_id': d.get('event_id'),
            'cls': d.get('class_id'),
            'image_b64': d['image_b64'],
        })
        self.get_logger().info(
            f"[스냅샷 큐] {d.get('class_id')} event_id={d.get('event_id')} "
            f"{len(d['image_b64'])/1024:.0f} KB")

    def _map_point_cb(self, msg: String):
        """change_point_detector 가 위치 기준 중복 제거를 마친 위험 이벤트.

        같은 자리의 같은 클래스는 change_point_detector 가 이미 걸러서
        여기까지 오지 않는다. 그래서 여기서는 신뢰도만 다시 확인하고
        그대로 서버에 큐잉하면 된다 — TF 변환도 여기서 다시 안 한다
        (change_point_detector 가 map_x/map_y 를 이미 계산해 줬다).
        """
        try:
            det = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"/map_point 파싱 실패: {e}")
            return

        class_id = str(det.get('class_id', ''))
        event_type = DANGER_CLASS_MAP.get(class_id)
        if event_type is None:
            return   # ship_defect 등 여기서 다루지 않는 클래스

        self._handle_danger_event(event_type, det)

    def _handle_danger_event(self, event_type, det):
        confidence = float(det.get('confidence') or 0.0)
        if confidence < self.min_confidence:
            return

        try:
            map_xy = [float(det['map_x']), float(det['map_y'])]
        except (KeyError, TypeError, ValueError):
            self.get_logger().warn(
                f"[{event_type}] map_x/map_y 없음 - change_point_detector "
                "최신 버전인지 확인")
            return

        ekf_global, _yaw = self._get_ekf_state()

        # ★ event_id: change_point_detector 가 이 위치에 매긴 안정된 식별자.
        #   프론트엔드가 event_cleared 로 어느 핑을 지울지 이 값으로 맞춘다.
        payload = {
            'event_type': event_type,
            'confidence': confidence,
            'map_xy': map_xy,
            'event_id': det.get('event_id'),
            'ekf_global': ekf_global,
        }
        # ★ 거울에 넣는다 — 재연결 때 다시 알리려고
        #   (위 _active_events 주석 참고)
        _eid = payload.get('event_id')
        if _eid:
            with self._active_lock:
                self._active_events[_eid] = dict(payload)
        self._enqueue(payload)
        self.get_logger().info(
            f"[위험이벤트 큐] {event_type} conf={confidence:.2f} "
            f"map_xy=({map_xy[0]:.2f},{map_xy[1]:.2f}) "
            f"event_id={det.get('event_id')}")

    def _cleared_cb(self, msg: String):
        """change_point_detector 가 "치워짐"을 확인했을 때. 서버로 그대로 넘겨
        프론트엔드가 해당 위치의 빨간 핑을 지우게 한다.

        프론트엔드와 합의한 봉투 형식 그대로 보낸다:
        {"event_type": "event_cleared", "block_id", "cls", "map_xy", "event_id"}
        """
        try:
            det = json.loads(msg.data)
            payload = {
                'event_type': 'event_cleared',
                'block_id': self.block_id,
                'cls': str(det['class_id']),
                'map_xy': [float(det['map_x']), float(det['map_y'])],
                'event_id': det.get('event_id'),
            }
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as e:
            self.get_logger().warn(f"/cleared 파싱 실패: {e}")
            return

        # 치워졌으므로 거울에서도 뺀다
        _eid = payload.get('event_id')
        if _eid:
            with self._active_lock:
                self._active_events.pop(_eid, None)
        self._enqueue(payload)
        self.get_logger().info(
            f"[치워짐 큐] {payload['cls']} event_id={payload['event_id']}")

    def _handle_block_level(self, class_id, level):
        """최근 창에서 **다수결**로 조립 단계를 확정한다.

        ★ 왜 다수결인가 (2026-08-27)

        예전에는 "마지막 값이 block_level_stability_s 동안 계속 같아야" 확정했다.
        값이 하나만 달라도 시계가 리셋되는 구조라, 검출이 조금만 흔들려도
        영영 확정이 안 되거나, 반대로 틀린 값이 우연히 연속으로 나오면
        그게 그대로 확정됐다.

        실물에서 조립 단계 검출은 이렇게 흔들린다(실측):
            깨끗한 정면      level3 0.434 (+ level1 0.036 딸려 나옴)
            배가 40% 잘림    level3 0.629 + level4 0.345
            배가 50% 잘림    level4 0.561   <- 뒤집힘
            모션블러 7px     검출 없음

        그래서 한 표씩 모아 다수결로 본다. 의견이 갈리면(득표율이
        block_level_vote_ratio 미만) **아무것도 확정하지 않는다** — 틀린 값을
        내보내는 것보다 조용한 편이 낫다.

        (잘린 화면을 아예 안 받도록 하는 것은 yolo_depth_publisher 의
         reject_level_touching_edge 가 맡는다. 여기까지 오는 표는 이미
         "온전히 담긴 배" 에서 나온 것이다.)
        """
        now = self.get_clock().now()

        with self._level_lock:
            self._level_window.append((now, level))
            cutoff = now - self.block_level_stability
            while self._level_window and self._level_window[0][0] < cutoff:
                self._level_window.popleft()

            if len(self._level_window) < self._level_min_samples:
                return

            counts = Counter(lv for _, lv in self._level_window)
            winner, votes = counts.most_common(1)[0]
            ratio = votes / len(self._level_window)
            if ratio < self._level_vote_ratio:
                self.get_logger().info(
                    f"[조립단계] 의견이 갈림 {dict(counts)} — 확정 보류",
                    throttle_duration_sec=10.0)
                return

            if self._level_confirmed == winner:
                return
            self._level_confirmed = winner

        payload = {
            'event_type': 'block_level',
            'block_id': self.block_id,
            'level': winner,
        }
        self._enqueue(payload)
        self.block_level_pub.publish(Int32(data=winner))
        self.get_logger().info(
            f"[조립단계 확정] level={winner} "
            f"({votes}/{len(self._level_window)}표, {100*ratio:.0f}%) "
            "-> 전송 + 재측량 트리거")

    def _ship_pose_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            map_xy = data['map_xy']
            yaw = float(data['yaw'])
            block_id = data.get('block_id', self.block_id)
        except (json.JSONDecodeError, KeyError, ValueError) as e:
            self.get_logger().warn(f"ship_pose 입력 파싱 실패: {e}")
            return

        payload = {
            'event_type': 'ship_pose',
            'block_id': block_id,
            'map_xy': [float(map_xy[0]), float(map_xy[1])],
            'yaw': yaw,
        }
        self._last_ship_pose = payload
        self._enqueue(payload)
        self.get_logger().info(f"[배위치] block_id={block_id} map_xy={map_xy} yaw={yaw:.3f}")

    def _enqueue(self, payload: dict):
        self.send_queue.put(payload)

    # ------------------------------------------------------------------
    # ★ 수신 루프: 서버 -> 젯슨 메시지를 해석 없이 그대로 /server/inbound로 발행.
    def _recv_loop(self, ws):
        while not self._stop_event.is_set():
            try:
                message = ws.recv(timeout=self.recv_timeout)
            except TimeoutError:
                continue
            except (ConnectionClosed, Exception) as e:
                self.get_logger().info(f"[수신 루프] 연결 종료/오류로 중단: {e}")
                return

            if isinstance(message, bytes):
                try:
                    message = message.decode('utf-8')
                except UnicodeDecodeError:
                    self.get_logger().warn("[수신 루프] 바이너리 메시지 디코딩 실패, 무시")
                    continue

            out_msg = String()
            out_msg.data = message
            self.inbound_pub.publish(out_msg)
            self.get_logger().info(f"[수신] /server/inbound 로 중계: {message[:200]}")

    def _ws_worker(self):
        while not self._stop_event.is_set():
            ws = None
            recv_thread = None
            try:
                self.get_logger().info(f"서버 연결 시도: {self.server_url}")
                ws = ws_connect(self.server_url, open_timeout=5)
                self.get_logger().info("서버 연결 성공")
                # 억제 상태를 푼다. 다음 장애 때 첫 실패가 즉시 찍히도록.
                self._last_fail_log = 0.0
                self._fail_count = 0

                recv_thread = threading.Thread(
                    target=self._recv_loop, args=(ws,), daemon=True)
                recv_thread.start()

                with self._level_lock:
                    current_level = self._level_confirmed
                if current_level is not None:
                    self._enqueue({
                        'event_type': 'block_level',
                        'block_id': self.block_id,
                        'level': current_level,
                    })
                    self.get_logger().info(f"[조립단계 재통보] level={current_level} 재전송")

                last_pose = self._last_ship_pose
                if last_pose is not None:
                    self._enqueue(dict(last_pose))
                    self.get_logger().info(
                        f"[배위치 재통보] map_xy={last_pose['map_xy']} 재전송")

                # ★ 지금 살아있는 위험 이벤트도 다시 알린다 (2026-08-29).
                #   서버가 재시작하면 프론트는 빈 화면인데 change_point 는
                #   이미 기억하고 있어 재발행하지 않는다. 그대로 두면 불이
                #   눈앞에 있는데 화면에는 아무것도 없는 상태가 **무기한**
                #   이어진다(event_ttl 은 재검출마다 갱신되므로 만료 안 됨).
                #   replay 를 실어 보내 프론트가 팝업 없이 핑만 그리게 한다.
                with self._active_lock:
                    revive = [dict(v) for v in self._active_events.values()]
                for ev in revive:
                    ev['replay'] = True
                    self._enqueue(ev)
                if revive:
                    self.get_logger().info(
                        f"[위험이벤트 재통보] {len(revive)}건 재전송: "
                        + ', '.join(str(e.get('event_id')) for e in revive))

                while not self._stop_event.is_set():
                    # 이벤트가 먼저다. 큐가 비었을 때만 최신 위치를 보낸다.
                    try:
                        payload = self.send_queue.get(timeout=0.2)
                    except queue.Empty:
                        payload = self._pending_position
                        self._pending_position = None
                        if payload is None:
                            continue

                    try:
                        ws.send(json.dumps(payload))
                    except (ConnectionClosed, Exception) as send_err:
                        self.get_logger().warn(f"전송 실패, 재큐잉: {send_err}")
                        self.send_queue.put(payload)
                        raise

            except Exception as e:
                # ★ 실패 로그는 억제하되 재시도는 늦추지 않는다.
                #   재시도 간격을 늘려 로그를 줄이면 서버가 살아난 뒤에도 그만큼
                #   데이터가 끊긴다. 위치핑이 2 Hz 라 대시보드에서 로봇이 멈춘
                #   것처럼 보인다. 줄여야 할 것은 로그지 가용성이 아니다.
                #
                #   첫 실패는 반드시 즉시 찍는다. 억제 때문에 "서버가 꺼져 있다"를
                #   한참 뒤에 알게 되면 안 된다.
                now = time.time()
                self._fail_count += 1
                if now - self._last_fail_log >= self.fail_log_interval:
                    extra = (f" (최근 {self.fail_log_interval:.0f}초간 "
                             f"{self._fail_count}회 실패)"
                             if self._fail_count > 1 else "")
                    self.get_logger().warn(
                        f"WebSocket 오류: {e} - "
                        f"{self.reconnect_interval}초 후 재시도{extra}")
                    self._last_fail_log = now
                    self._fail_count = 0
            finally:
                if ws is not None:
                    try:
                        ws.close()
                    except Exception:
                        pass
                if recv_thread is not None:
                    recv_thread.join(timeout=2.0)

            time.sleep(self.reconnect_interval)

    def destroy_node(self):
        self._stop_event.set()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebSocketClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
