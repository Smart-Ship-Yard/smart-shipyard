#!/usr/bin/env python3
"""
websocket_client.py
--------------------
젯슨 -> 백엔드 서버 WebSocket 전송 노드. (스펙: 젯슨-서버 통신 스펙 v1.2)
websockets 라이브러리 사용.

구현 범위: ① 위치 핑, ② 위험 이벤트, ③ 조립 단계, ④ 배 위치(중계)

[2026-07-14] ⑤ 영상 스트리밍, ⑥ stream_boost 관련 코드 전부 제거함.
영상 송출은 동료의 별도 시스템이 담당하며, 이 노드가 동시에 영상 채널을
열면 서버 쪽과 충돌하는 것으로 확인되어 이 프로젝트에서는 다루지 않기로 함.

EKF 토픽: ship_ugv_localization/launch/localization.launch.py 확인 결과
  ekf_global 노드가 remappings=[('odometry/filtered', '/odometry/global')]로
  nav_msgs/Odometry를 /odometry/global 에 발행 (world_frame=map).

필요 패키지: pip install websockets --user
"""

import json
import queue
import re
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import String
from nav_msgs.msg import Odometry

try:
    from websockets.sync.client import connect as ws_connect
    from websockets.exceptions import ConnectionClosed
except ImportError:
    ws_connect = None
    ConnectionClosed = Exception


# ② 위험 이벤트: YOLO 클래스 이름 -> 백엔드 event_type 매핑
DANGER_CLASS_MAP = {
    'fallen_person': 'fallen_person',
    'person_fallen': 'fallen_person',   # 클래스명 변경 전 구모델 호환
    'fire': 'fire',
    'no_helmet': 'no_helmet',
    'ship_defect': 'ship_defect',
}


def extract_level(class_name: str):
    """클래스 이름에서 숫자 추출: 'level2', 'levle3', 'ship_defect_2' 모두 처리."""
    match = re.search(r'(\d+)', str(class_name))
    return int(match.group(1)) if match else None


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

        if ws_connect is None:
            self.get_logger().error(
                "websockets 미설치. 'pip install websockets --user' 후 재실행."
            )

        # ---- ① 위치 핑용 상태 ----
        self.latest_ekf_global = None
        self._ekf_lock = threading.Lock()
        self._ping_count = 0

        # ---- ③ 조립 단계용 상태 ----
        self._level_candidate = None
        self._level_candidate_since = None
        self._level_confirmed = None
        self._level_lock = threading.Lock()

        self.send_queue = queue.Queue()

        self.create_subscription(String, uvd_topic, self._uvd_cb, 10)
        self.create_subscription(Odometry, ekf_topic, self._ekf_cb, 10)
        self.create_subscription(String, ship_pose_topic, self._ship_pose_cb, 10)

        self.create_timer(self.ping_interval, self._position_ping_cb)

        self._stop_event = threading.Event()
        self._ws_thread = threading.Thread(target=self._ws_worker, daemon=True)
        self._ws_thread.start()

        self.get_logger().info(
            f"websocket_client 시작: server={self.server_url}, "
            f"ekf_topic={ekf_topic}, ship_pose_topic={ship_pose_topic}, "
            f"ping={self.ping_interval}s, min_conf={self.min_confidence}, "
            f"block_level_stability={self.block_level_stability.nanoseconds/1e9:.1f}s"
        )

    # ------------------------------------------------------------------
    def _ekf_cb(self, msg: Odometry):
        with self._ekf_lock:
            self.latest_ekf_global = [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
            ]

    def _get_ekf_global(self):
        with self._ekf_lock:
            return list(self.latest_ekf_global) if self.latest_ekf_global else None

    # ------------------------------------------------------------------
    # ① 위치 핑 (0.5초 주기)
    def _position_ping_cb(self):
        ekf_global = self._get_ekf_global()
        if ekf_global is None:
            return

        self._enqueue({'event_type': 'position', 'ekf_global': ekf_global})

        self._ping_count += 1
        if self._ping_count % 10 == 0:
            self.get_logger().info(f"[위치핑] ekf_global={ekf_global}")

    # ------------------------------------------------------------------
    # /event_detection/uvd 콜백: ② 위험 이벤트 또는 ③ 조립 단계로 분기
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

    # ------------------------------------------------------------------
    # ② 위험 이벤트 처리
    def _handle_danger_event(self, event_type, det):
        confidence = float(det.get('confidence', 0.0))
        if confidence < self.min_confidence:
            return

        depth_xyz = det.get('depth_xyz')
        if depth_xyz is None:
            self.get_logger().warn(
                f"[{event_type}] depth_xyz 없음 - yolo_depth_publisher 최신 버전인지 확인")
            return

        ekf_global = self._get_ekf_global()
        if ekf_global is None:
            self.get_logger().warn(
                f"[{event_type}] ekf_global 없음(EKF 미가동) - 이벤트 전송 보류")
            return

        payload = {
            'event_type': event_type,
            'confidence': confidence,
            'depth_xyz': depth_xyz,
            'ekf_global': ekf_global,
        }
        self._enqueue(payload)
        self.get_logger().info(f"[위험이벤트 큐] {event_type} conf={confidence:.2f}")

    # ------------------------------------------------------------------
    # ③ 조립 단계 처리 (안정화 필터 포함)
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

    # ------------------------------------------------------------------
    # ④ 배 위치 (측량 결과 중계 - 측량 방법 자체는 아직 미정, TODO)
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

    # ------------------------------------------------------------------
    def _enqueue(self, payload: dict):
        self.send_queue.put(payload)

    # ------------------------------------------------------------------
    def _ws_worker(self):
        """전송 전담 스레드: 연결 유지 + 큐 소비. 실패 시 자동 재접속."""
        while not self._stop_event.is_set():
            ws = None
            try:
                self.get_logger().info(f"서버 연결 시도: {self.server_url}")
                ws = ws_connect(self.server_url, open_timeout=5)
                self.get_logger().info("서버 연결 성공")

                # 재접속 시 현재 확정된 조립 단계를 한 번 다시 통보
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
