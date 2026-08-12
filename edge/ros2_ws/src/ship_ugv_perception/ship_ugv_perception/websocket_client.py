#!/usr/bin/env python3
"""
websocket_client.py
--------------------
젯슨 <-> 백엔드 서버 WebSocket 노드. (스펙: 젯슨-서버 통신 스펙 v1.3)

[2026-07-17 스펙 v1.3 반영]
- ① position에 yaw(로봇이 바라보는 방향, 라디안) 추가
  -> /odometry/global의 orientation 쿼터니언에서 yaw 추출.
- ② 위험 이벤트에 map_xy(객체의 map 절대 좌표) 추가
  -> depth_xyz(카메라 기준)를 tf2로 map 프레임 변환 (change_point.py와 동일한
     카메라 장착 오프셋 + REP-103 좌표축 변환 로직 사용).
  -> TF(map->base_link) 조회 실패 시(EKF 미가동 등) 해당 이벤트 전송 보류.
- depth_xyz, ekf_global은 디버깅용으로 유지.
- ③ block_level, ④ ship_pose는 변경 없음.

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
import math
import queue
import re
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401  (PointStamped 변환 등록용)

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
        self.declare_parameter('reconnect_interval_s', 5.0)
        self.declare_parameter('block_id', 'B1')
        self.declare_parameter('block_level_stability_s', 3.0)

        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('camera_offset_x', 0.15)
        self.declare_parameter('camera_offset_y', 0.0)
        self.declare_parameter('tf_timeout_s', 0.3)

        # ★ 수신 루프 관련
        self.declare_parameter('inbound_topic', '/server/inbound')
        self.declare_parameter('recv_timeout_s', 1.0)

        self.server_url = self.get_parameter('server_ws_url').value
        uvd_topic = self.get_parameter('uvd_topic').value
        ekf_topic = self.get_parameter('ekf_odom_topic').value
        ship_pose_topic = self.get_parameter('ship_pose_input_topic').value
        self.ping_interval = self.get_parameter('position_ping_interval_s').value
        self.min_confidence = self.get_parameter('min_confidence').value
        self.reconnect_interval = self.get_parameter('reconnect_interval_s').value
        self.block_id = self.get_parameter('block_id').value
        self.block_level_stability = Duration(
            seconds=self.get_parameter('block_level_stability_s').value)

        self.map_frame = self.get_parameter('map_frame_id').value
        self.base_frame = self.get_parameter('base_frame_id').value
        self.cam_offset_x = self.get_parameter('camera_offset_x').value
        self.cam_offset_y = self.get_parameter('camera_offset_y').value
        self.tf_timeout = Duration(seconds=self.get_parameter('tf_timeout_s').value)

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

        self._level_candidate = None
        self._level_candidate_since = None
        self._level_confirmed = None
        self._level_lock = threading.Lock()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.send_queue = queue.Queue()

        # ★ 서버 -> 젯슨 수신 메시지를 그대로 중계할 퍼블리셔 (해석하지 않음)
        self.inbound_pub = self.create_publisher(String, inbound_topic, 10)

        self.create_subscription(String, uvd_topic, self._uvd_cb, 10)
        self.create_subscription(Odometry, ekf_topic, self._ekf_cb, 10)
        self.create_subscription(String, ship_pose_topic, self._ship_pose_cb, 10)

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

        self._enqueue({
            'event_type': 'position',
            'ekf_global': ekf_global,
            'yaw': yaw,
        })

        self._ping_count += 1
        if self._ping_count % 10 == 0:
            self.get_logger().info(f"[위치핑] ekf_global={ekf_global} yaw={yaw:.3f}")

    def _camera_xyz_to_map_xy(self, depth_xyz):
        x_cam, y_cam, z_cam = depth_xyz

        local_x = z_cam + self.cam_offset_x
        local_y = -x_cam + self.cam_offset_y

        point_in_base = PointStamped()
        point_in_base.header.frame_id = self.base_frame
        point_in_base.header.stamp = self.get_clock().now().to_msg()
        point_in_base.point.x = local_x
        point_in_base.point.y = local_y
        point_in_base.point.z = 0.0

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame,
                Time(),
                timeout=self.tf_timeout)
        except Exception as e:
            self.get_logger().warn(
                f"TF 조회 실패 ({self.map_frame}<-{self.base_frame}): {e}")
            return None

        point_in_map = tf2_geometry_msgs.do_transform_point(point_in_base, transform)
        return [point_in_map.point.x, point_in_map.point.y]

    def _uvd_cb(self, msg: String):
        try:
            det = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"/uvd 파싱 실패: {e}")
            return

        class_id = str(det.get('class_id', ''))

        event_type = DANGER_CLASS_MAP.get(class_id)
        if event_type is not None:
            self._handle_danger_event(event_type, det)
            return

        level = extract_level(class_id)
        if level is not None:
            self._handle_block_level(class_id, level)
            return

    def _handle_danger_event(self, event_type, det):
        confidence = float(det.get('confidence', 0.0))
        if confidence < self.min_confidence:
            return

        depth_xyz = det.get('depth_xyz')
        if depth_xyz is None:
            self.get_logger().warn(
                f"[{event_type}] depth_xyz 없음 - yolo_depth_publisher 최신 버전인지 확인")
            return

        ekf_global, _yaw = self._get_ekf_state()
        if ekf_global is None:
            self.get_logger().warn(
                f"[{event_type}] ekf_global 없음(EKF 미가동) - 이벤트 전송 보류")
            return

        map_xy = self._camera_xyz_to_map_xy(depth_xyz)
        if map_xy is None:
            self.get_logger().warn(
                f"[{event_type}] map_xy 변환 실패(TF 미가용) - 이벤트 전송 보류")
            return

        payload = {
            'event_type': event_type,
            'confidence': confidence,
            'map_xy': map_xy,
            'depth_xyz': depth_xyz,
            'ekf_global': ekf_global,
        }
        self._enqueue(payload)
        self.get_logger().info(
            f"[위험이벤트 큐] {event_type} conf={confidence:.2f} "
            f"map_xy=({map_xy[0]:.2f},{map_xy[1]:.2f})")

    def _handle_block_level(self, class_id, level):
        now = self.get_clock().now()

        with self._level_lock:
            if self._level_candidate != level:
                self._level_candidate = level
                self._level_candidate_since = now
                self.get_logger().info(
                    f"[조립단계] 새 후보 감지: class_id={class_id} level={level} (안정화 대기 시작)")
                return

            elapsed = now - self._level_candidate_since
            if elapsed < self.block_level_stability:
                return

            if self._level_confirmed == level:
                return

            self._level_confirmed = level

        payload = {
            'event_type': 'block_level',
            'block_id': self.block_id,
            'level': level,
        }
        self._enqueue(payload)
        self.get_logger().info(f"[조립단계 확정] level={level} -> 전송")

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

                while not self._stop_event.is_set():
                    try:
                        payload = self.send_queue.get(timeout=1.0)
                    except queue.Empty:
                        continue

                    try:
                        ws.send(json.dumps(payload))
                    except (ConnectionClosed, Exception) as send_err:
                        self.get_logger().warn(f"전송 실패, 재큐잉: {send_err}")
                        self.send_queue.put(payload)
                        raise

            except Exception as e:
                self.get_logger().warn(
                    f"WebSocket 오류: {e} - {self.reconnect_interval}초 후 재시도")
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
