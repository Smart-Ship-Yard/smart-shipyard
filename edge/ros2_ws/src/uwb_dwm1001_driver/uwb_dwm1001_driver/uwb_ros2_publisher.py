#!/usr/bin/env python3
"""
DWM1001 UWB Tag Driver (ROS2)
-----------------------------
UART shell 모드로 DWM1001 태그와 통신하여 'lec' (location engine, continuous)
스트림을 파싱하고 다음을 발행한다:

  /uwb/raw        (std_msgs/String)                 - 원시 lec 라인 (디버그용)
  /uwb/distances  (uwb_msgs/AnchorDistances 대용, JSON String)  - 앵커별 거리 + 품질계수(QF)
  /uwb/pose       (geometry_msgs/PoseWithCovarianceStamped, frame_id='uwb_frame')

핵심 설계 포인트
----------------
1. frame_id 버그 수정: 반드시 'uwb_frame' 으로 발행해야 uwb_map_calibration이 발행하는
   'map' -> 'uwb_frame' 정적 TF를 거쳐 EKF(map 프레임)로 올바르게 들어간다.
   과거 'map' 으로 하드코딩되어 있던 시절에는 캘리브레이션 변환이 통째로 무시되는
   치명적 버그가 있었다. (절대 다시 'map'으로 바꾸지 말 것)
2. 3중 아웃라이어 게이팅 (호스트 레벨):
   a) Quality Factor(QF) 임계값 미달 샘플 폐기
   b) 야드 경계(bounding box) 밖 좌표 폐기 (앵커 반사/고스트 멀티패스로 인한 튐)
   c) 순간 속도(직전 유효 샘플과의 시간당 이동거리) 임계값 초과 시 폐기
3. orientation covariance는 매우 큰 값으로 채워서 EKF가 UWB의 방향 정보를
   (애초에 없으므로) 신뢰하지 않도록 방어한다. yaw는 heading_complementary_filter가
   별도로 공급한다.
4. lec 토글 명령 안전화: 명령을 무작정 보내는 대신, 먼저 몇 초간 스트림을 프로브하여
   이미 lec 모드인지 확인한 뒤에만 토글 명령을 보낸다. (이미 켜진 상태에서 토글하면
   꺼져버리는 사고를 방지)
"""

import json
import math
import re
import time

import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped

# DWM1001 'lec' 출력 라인 예시:
# POS,x,-1.23,y,4.56,z,0.00,qf,87
# DIST,4,AN0,1783,x,0.00,y,0.00,z,0.00,dist,2.31,AN1,...
LEC_POS_RE = re.compile(
    r"POS,x,(?P<x>-?\d+\.?\d*),y,(?P<y>-?\d+\.?\d*),z,(?P<z>-?\d+\.?\d*),qf,(?P<qf>\d+)"
)


class UwbDwm1001Driver(Node):

    def __init__(self):
        super().__init__('uwb_dwm1001_driver')

        # ---- 파라미터 ----
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('uwb_frame_id', 'uwb_frame')  # 절대 'map'으로 바꾸지 말 것
        self.declare_parameter('qf_threshold', 60)            # 0~100, 이 미만이면 폐기
        self.declare_parameter('yard_min_x', -50.0)
        self.declare_parameter('yard_max_x', 50.0)
        self.declare_parameter('yard_min_y', -50.0)
        self.declare_parameter('yard_max_y', 50.0)
        self.declare_parameter('max_speed_mps', 3.0)           # 이 이상 순간이동시 폐기
        self.declare_parameter('probe_seconds', 2.0)           # lec 모드 여부 프로브 시간
        self.declare_parameter('base_position_covariance', 0.05)  # m^2, QF=100 기준
        self.declare_parameter('publish_rate_hz', 10.0)

        self.port = self.get_parameter('serial_port').value
        self.baud = self.get_parameter('baud_rate').value
        self.uwb_frame_id = self.get_parameter('uwb_frame_id').value
        self.qf_threshold = self.get_parameter('qf_threshold').value
        self.yard_bounds = (
            self.get_parameter('yard_min_x').value,
            self.get_parameter('yard_max_x').value,
            self.get_parameter('yard_min_y').value,
            self.get_parameter('yard_max_y').value,
        )
        self.max_speed_mps = self.get_parameter('max_speed_mps').value
        self.probe_seconds = self.get_parameter('probe_seconds').value
        self.base_cov = self.get_parameter('base_position_covariance').value

        if self.uwb_frame_id != 'uwb_frame':
            self.get_logger().warn(
                f"uwb_frame_id 파라미터가 'uwb_frame'이 아닙니다 ('{self.uwb_frame_id}'). "
                "uwb_map_calibration의 정적 TF와 프레임명이 일치하는지 다시 확인하세요."
            )

        # ---- 퍼블리셔 ----
        self.raw_pub = self.create_publisher(String, '/uwb/raw', 10)
        self.dist_pub = self.create_publisher(String, '/uwb/distances', 10)
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/uwb/pose', 10
        )

        # ---- 게이팅용 상태 ----
        self._last_valid_xy = None
        self._last_valid_stamp = None
        self._reject_count = 0
        self._accept_count = 0

        # ---- 시리얼 연결 ----
        self.ser = None
        self._connect_serial()
        self._ensure_lec_mode()

        period = 1.0 / self.get_parameter('publish_rate_hz').value
        self.timer = self.create_timer(period, self._poll_serial)

        self.get_logger().info(
            f"UWB DWM1001 driver started on {self.port}@{self.baud}, "
            f"publishing pose in frame_id='{self.uwb_frame_id}'"
        )

    # ------------------------------------------------------------------
    # 시리얼 연결 / lec 모드 안전 토글
    # ------------------------------------------------------------------
    def _connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
        except serial.SerialException as e:
            self.get_logger().error(f"시리얼 포트 연결 실패: {e}")
            raise

    def _send_shell_cmd(self, cmd: str):
        self.ser.write((cmd + '\r').encode('utf-8'))
        time.sleep(0.1)

    def _enter_shell_mode(self):
        # DWM1001 UART shell은 더블 엔터로 진입
        self.ser.write(b'\r\r')
        time.sleep(0.3)
        self.ser.reset_input_buffer()

    def _ensure_lec_mode(self):
        """
        무조건 'lec' 토글 명령을 보내지 않는다.
        먼저 probe_seconds 동안 들어오는 라인이 이미 POS,... 형식인지 확인하고,
        이미 스트리밍 중이면 아무것도 하지 않는다. 스트리밍이 없을 때만 lec를 켠다.
        """
        self._enter_shell_mode()

        self.get_logger().info(
            f"lec 스트림 상태 프로브 중... ({self.probe_seconds}s)"
        )
        t0 = time.time()
        already_streaming = False
        buf = b''
        while time.time() - t0 < self.probe_seconds:
            chunk = self.ser.read(256)
            if chunk:
                buf += chunk
                if b'POS,' in buf or b'DIST,' in buf:
                    already_streaming = True
                    break

        if already_streaming:
            self.get_logger().info("이미 lec 스트리밍 중 - 토글 명령 생략")
            return

        self.get_logger().info("lec 스트리밍 없음 - 'lec' 명령 전송")
        self._enter_shell_mode()
        self._send_shell_cmd('lec')
        time.sleep(0.3)

    # ------------------------------------------------------------------
    # 폴링 / 파싱
    # ------------------------------------------------------------------
    def _poll_serial(self):
        try:
            n = self.ser.in_waiting
        except (OSError, serial.SerialException) as e:
            self.get_logger().error(f"시리얼 읽기 오류, 재연결 시도: {e}")
            self._reconnect()
            return

        if n == 0:
            return

        raw = self.ser.read(n).decode('utf-8', errors='ignore')
        for line in raw.splitlines():
            line = line.strip()
            if not line:
                continue
            self._handle_line(line)

    def _reconnect(self):
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        time.sleep(1.0)
        try:
            self._connect_serial()
            self._ensure_lec_mode()
        except Exception as e:
            self.get_logger().error(f"재연결 실패: {e}")

    def _handle_line(self, line: str):
        raw_msg = String()
        raw_msg.data = line
        self.raw_pub.publish(raw_msg)

        if line.startswith('DIST'):
            self._handle_distances(line)
            return

        m = LEC_POS_RE.search(line)
        if not m:
            return
        self._handle_position(m)

    def _handle_distances(self, line: str):
        # "DIST,4,AN0,....,dist,2.31,AN1,....,dist,3.02,..." 형태를 앵커별로 분해
        parts = line.split(',')
        anchors = {}
        i = 0
        while i < len(parts):
            if parts[i].startswith('AN'):
                anchor_id = parts[i]
                try:
                    dist_idx = parts.index('dist', i)
                    dist = float(parts[dist_idx + 1])
                    anchors[anchor_id] = dist
                except (ValueError, IndexError):
                    pass
            i += 1
        if anchors:
            msg = String()
            msg.data = json.dumps({'stamp': time.time(), 'anchors': anchors})
            self.dist_pub.publish(msg)

    def _handle_position(self, m: re.Match):
        x = float(m.group('x'))
        y = float(m.group('y'))
        qf = int(m.group('qf'))
        now = time.time()

        # --- 게이트 1: Quality Factor ---
        if qf < self.qf_threshold:
            self._reject_count += 1
            self.get_logger().debug(f"QF 게이트 탈락: qf={qf} < {self.qf_threshold}")
            return

        # --- 게이트 2: 야드 경계 (물리적으로 있을 수 없는 좌표 = 고스트 반사) ---
        min_x, max_x, min_y, max_y = self.yard_bounds
        if not (min_x <= x <= max_x and min_y <= y <= max_y):
            self._reject_count += 1
            self.get_logger().debug(f"야드 경계 게이트 탈락: ({x:.2f},{y:.2f})")
            return

        # --- 게이트 3: 순간 속도 (다중경로로 인한 순간 점프) ---
        if self._last_valid_xy is not None:
            dt = now - self._last_valid_stamp
            if dt > 1e-3:
                dist = math.hypot(x - self._last_valid_xy[0], y - self._last_valid_xy[1])
                speed = dist / dt
                if speed > self.max_speed_mps:
                    self._reject_count += 1
                    self.get_logger().debug(
                        f"속도 게이트 탈락: {speed:.2f} m/s > {self.max_speed_mps}"
                    )
                    return

        # --- 통과: 상태 갱신 및 발행 ---
        self._last_valid_xy = (x, y)
        self._last_valid_stamp = now
        self._accept_count += 1

        self._publish_pose(x, y, qf)

    # ------------------------------------------------------------------
    # 발행
    # ------------------------------------------------------------------
    def _publish_pose(self, x: float, y: float, qf: int):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        # 반드시 'uwb_frame' 이어야 함. uwb_map_calibration이 발행하는
        # map -> uwb_frame 정적 TF를 통해서만 EKF의 map 프레임으로 정합됨.
        msg.header.frame_id = self.uwb_frame_id

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        # UWB는 방향 정보가 없으므로 항등 쿼터니언
        msg.pose.pose.orientation.w = 1.0

        # QF가 낮을수록(하지만 임계값은 통과) covariance를 키움 - 동적 신뢰도 반영
        qf_factor = max(1.0, (100.0 - qf) / 20.0)
        pos_cov = self.base_cov * qf_factor

        cov = [0.0] * 36
        cov[0] = pos_cov       # x
        cov[7] = pos_cov       # y
        cov[14] = 999999.0     # z (미사용, 매우 크게)
        # orientation covariance는 방어적으로 매우 크게 -> EKF가 orientation 무시
        cov[21] = 999999.0
        cov[28] = 999999.0
        cov[35] = 999999.0
        msg.pose.covariance = cov

        self.pose_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UwbDwm1001Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
