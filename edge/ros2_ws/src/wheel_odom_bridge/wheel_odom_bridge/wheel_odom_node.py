#!/usr/bin/env python3
"""
wheel_odom_node.py
-------------------
Arduino Mega(Cytron MDD10A 모터드라이버 x1(2채널) + JGB37-520 엔코더 x2,
차동구동) 시리얼 브리지.

★ 이 노드는 Arduino의 모터 드라이버 전기적 인터페이스(MDD10A의 PWM+DIR 모드인지,
  다른 드라이버/모드인지)와 완전히 무관하다. wheel_odom_node는 오직 시리얼로
  "V,<l>,<r>" / "E,<dl>,<dr>,<dt>" 텍스트만 주고받으므로, 모터 배선이 바뀌어도
  Arduino 펌웨어만 그에 맞게 고치면 되고 이 파이썬 코드는 그대로 둬도 된다.

역할:
  1) /cmd_vel(geometry_msgs/Twist) 구독 -> 차동구동 역기구학으로
     좌/우 목표 바퀴 속도(m/s) 계산 -> ticks/sec로 변환해 Arduino에 전달.
  2) Arduino가 주기적으로 보내는 엔코더 델타("E,<dl>,<dr>,<dt_ms>")를 읽어
     정기구학으로 로봇 이동량을 적분 -> /wheel/odom(nav_msgs/Odometry) 발행.

시리얼 프로토콜 (wheel_encoder_mcu.ino와 반드시 일치해야 함):
  Jetson -> Arduino:  "V,<left_ticks_per_sec>,<right_ticks_per_sec>\n"
  Arduino -> Jetson:  "E,<left_delta_ticks>,<right_delta_ticks>,<dt_ms>\n"

★ 이 노드는 TF를 발행하지 않는다 (odom -> base_link TF는 ekf_local이 발행).
  프로젝트 전체 원칙: TF의 최종 권위자는 항상 EKF 하나뿐이고, 개별 센서/브리지
  노드는 오직 "하나의 측정 소스"로서 토픽만 발행한다 (map<->uwb_frame,
  map<->slam_map과 동일한 설계 원칙).

★ track_width_m, wheel_radius_m 기본값은 0.0(placeholder)이며, 실제 값은
  localization.launch.py에서 실측치로 주입한다 (2026-08 최종 확정: 트랙폭
  0.22568m, 바퀴 반지름 0.0308m, ticks_per_rev 330 - UWB 실측 기반 역산,
  자세한 과정은 edge/docs/ju_ws_설치가이드_v3.md 4장 참고). 파라미터가
  0.0인 채로 실행되면 경고를 내고 오도메트리를 발행하지 않는다
  (잘못된 값을 EKF에 주입하지 않기 위함).
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray

try:
    import serial
except ImportError:
    serial = None


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class WheelOdomBridge(Node):

    def __init__(self):
        super().__init__('wheel_odom_bridge')

        # ---- 파라미터 ----
        self.declare_parameter('serial_port', '/dev/wheel_mcu')
        # /dev/wheel_mcu는 udev 규칙(edge/scripts/99-robot-serial.rules)으로
        # 고정된 심볼릭 링크. localization.launch.py가 이 값을 override해서
        # 넘겨준다. 규칙이 아직 안 걸려있다면 `ls /dev/ttyACM*`로 실제 포트를
        # 확인해서 launch 파라미터를 그 값으로 override할 것.
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/wheel/odom')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')

        # ★ 실측 전 placeholder - 실측값 나오면 반드시 갱신할 것
        self.declare_parameter('track_width_m', 0.0)
        self.declare_parameter('wheel_radius_m', 0.0)
        self.declare_parameter('ticks_per_rev', 1320)  # JGB37-520 확정 스펙

        # ★ 좌우 모터 개체차(힘 차이) 보정용 trim 계수.
        #   heading_hold로 상당 부분 잡히지만, 그래도 남는 잔차를 여기서 미세조정.
        #   왼쪽으로 휘면(로봇 기준) 왼쪽이 상대적으로 약하거나 오른쪽이 강한 것이므로
        #   right_trim을 줄이거나 left_trim을 올려서 균형을 맞춘다.
        self.declare_parameter('left_trim', 1.0)
        self.declare_parameter('right_trim', 1.0)

        # ★ heading_hold: /cmd_vel 발행자가 누구든(motion_controller,
        #   keyboard_teleop, 나중의 Nav2 등) 순수 직진/후진(w≈0) 명령을
        #   보내면, 여기서 자체 yaw(self.yaw)를 보고 좌우 편향을 실시간
        #   보정한다. 발행자 쪽에는 이 로직을 넣지 않는다 (한 곳에서만
        #   관리하기 위함 - 2026-08 리팩토링, 이전엔 노드마다 중복 구현했었음).
        self.declare_parameter('enable_heading_hold', True)
        self.declare_parameter('kp_heading_hold', 1.0)
        self.declare_parameter('ki_heading_hold', 0.5)
        self.declare_parameter('heading_hold_w_threshold', 0.02)  # 이 이하면 "순수 직진 의도"로 간주

        self.declare_parameter('cmd_vel_timeout_s', 0.5)   # ROS 레벨 세이프티(중복 방어)
        # 포트를 열면 DTR 토글로 Arduino가 자동 리셋된다. 부트로더 -> 사용자 스케치
        # 실행까지 걸리는 시간 동안은 통신을 시도해도 응답이 없으므로 건너뛴다.
        self.declare_parameter('reset_grace_period_s', 3.0)
        self.declare_parameter('cmd_resend_interval_s', 0.1)  # Arduino 워치독(500ms) 충족용 재전송 주기
        self.declare_parameter('serial_poll_interval_s', 0.02)

        self.port = self.get_parameter('serial_port').value
        self.baud = self.get_parameter('baud_rate').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame_id').value
        self.base_frame = self.get_parameter('base_frame_id').value

        self.track_width = self.get_parameter('track_width_m').value
        self.wheel_radius = self.get_parameter('wheel_radius_m').value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value
        self.left_trim = self.get_parameter('left_trim').value
        self.right_trim = self.get_parameter('right_trim').value

        self.enable_heading_hold = self.get_parameter('enable_heading_hold').value
        self.kp_heading_hold = self.get_parameter('kp_heading_hold').value
        self.ki_heading_hold = self.get_parameter('ki_heading_hold').value
        self.heading_hold_w_threshold = self.get_parameter('heading_hold_w_threshold').value

        self._drive_mode = 'stop'            # 'stop' | 'straight' | 'other'
        self._straight_start_yaw = 0.0
        self._heading_error_integral = 0.0
        self._last_heading_hold_time = None  # 직진 보정 dt 계산용

        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout_s').value
        self.cmd_resend_interval = self.get_parameter('cmd_resend_interval_s').value
        self.reset_grace = self.get_parameter('reset_grace_period_s').value

        if self.track_width <= 0.0 or self.wheel_radius <= 0.0:
            self.get_logger().warn(
                "track_width_m/wheel_radius_m이 아직 실측값으로 설정되지 않았습니다 "
                f"(현재 track_width={self.track_width}, wheel_radius={self.wheel_radius}). "
                "실측 전까지 속도 변환이 0으로 처리되어 로봇이 움직이지 않거나 "
                "오도메트리가 부정확합니다. launch 파라미터로 반드시 갱신하세요."
            )

        if serial is None:
            self.get_logger().error(
                "pyserial 미설치. 'pip install pyserial --user' 후 재실행하세요."
            )

        # ---- 내부 상태 ----
        self._ser = None
        self._ser_lock = threading.Lock()
        self._rx_buffer = ''
        self._connect_time = None       # 마지막으로 포트를 연 시각 (리셋 대기 판정용)
        self._grace_logged = False      # 대기 완료 로그를 한 번만 찍기 위한 플래그

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last_cmd_l_tps = 0
        self.last_cmd_r_tps = 0
        self.last_cmd_vel_time = None

        # ---- 시리얼 연결 ----
        self._connect_serial()

        # ---- 통신 ----
        self.create_subscription(Twist, cmd_vel_topic, self._cmd_vel_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 20)
        # ★ 진단용: 좌우 raw 엔코더 델타를 그대로 발행 (PID/trim 튜닝 데이터 수집용)
        self.raw_ticks_pub = self.create_publisher(Int32MultiArray, '/wheel/raw_ticks', 50)

        # 시리얼 폴링 (엔코더 델타 수신)
        poll_interval = self.get_parameter('serial_poll_interval_s').value
        self.create_timer(poll_interval, self._poll_serial)

        # Arduino 워치독(CMD_TIMEOUT_MS=500) 충족을 위한 주기적 명령 재전송
        self.create_timer(self.cmd_resend_interval, self._resend_cmd)

        self.get_logger().info(
            f"wheel_odom_bridge 시작: port={self.port}@{self.baud}, "
            f"track_width={self.track_width}m, wheel_radius={self.wheel_radius}m, "
            f"ticks_per_rev={self.ticks_per_rev}, TF는 발행하지 않음(ekf_local이 발행)"
        )

    # ------------------------------------------------------------------
    def _connect_serial(self):
        if serial is None:
            return
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.01)
            # ★ 여기서 time.sleep()을 하면 안 된다. 이 함수는 타이머 콜백에서도
            #   재연결용으로 불리기 때문에, 블로킹하면 노드 전체(cmd_vel 처리,
            #   오도메트리 발행)가 그 시간만큼 통째로 멈춘다.
            #   대신 연결 시각만 기록해두고, 통신 시도 쪽에서 건너뛰게 한다.
            self._connect_time = time.monotonic()
            self._grace_logged = False
            self.get_logger().info(
                f"시리얼 연결 성공: {self.port}, "
                f"Arduino 자동 리셋(DTR) 회복 대기 {self.reset_grace}초 (논블로킹)"
            )
        except Exception as e:
            self._ser = None
            self.get_logger().error(f"시리얼 연결 실패: {e} (재시도는 poll에서 계속됨)")

    def _in_reset_grace(self) -> bool:
        """포트를 연 직후 Arduino가 리셋되어 부팅 중인 구간인지 판정."""
        if self._connect_time is None:
            return False
        if (time.monotonic() - self._connect_time) < self.reset_grace:
            return True
        if not self._grace_logged:
            self.get_logger().info("리셋 회복 대기 완료, 통신 시작")
            self._grace_logged = True
        return False

    # ------------------------------------------------------------------
    # /cmd_vel 수신 -> 목표 바퀴속도(ticks/sec) 계산 -> 즉시 1회 전송
    def _cmd_vel_cb(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        # ★ heading_hold: /cmd_vel 발행자가 w≈0(순수 직진/후진 의도)을 보내면
        #   여기서 실시간으로 자체 yaw(self.yaw, 이 노드가 오도메트리 계산하며
        #   이미 갖고 있음)를 보고 좌우 편향을 보정한다. motion_controller나
        #   keyboard_teleop처럼 /cmd_vel을 보내는 쪽에는 이 로직을 넣지 않는다
        #   - 발행자가 누구든 여기 한 곳만 거치면 자동으로 적용되게 하기 위함.
        is_straight_intent = abs(w) < self.heading_hold_w_threshold and abs(v) > 1e-3

        if self.enable_heading_hold and is_straight_intent:
            if self._drive_mode != 'straight':
                # 새로 직진을 시작하는 순간 -> 지금 방향을 기준선으로 새로 잡음
                self._straight_start_yaw = self.yaw
                self._heading_error_integral = 0.0
                self._last_heading_hold_time = None
                self._drive_mode = 'straight'

            now = time.monotonic()
            if self._last_heading_hold_time is None:
                dt = 0.02  # 이번이 첫 호출이면 20ms로 근사
            else:
                dt = now - self._last_heading_hold_time
                dt = max(0.001, min(0.2, dt))  # 비정상적으로 크거나 작은 dt 방지
            self._last_heading_hold_time = now

            yaw_error = wrap_to_pi(self._straight_start_yaw - self.yaw)
            self._heading_error_integral += yaw_error * dt
            self._heading_error_integral = max(-1.0, min(1.0, self._heading_error_integral))  # 와인드업 방지

            w = (self.kp_heading_hold * yaw_error
                 + self.ki_heading_hold * self._heading_error_integral)
        else:
            self._drive_mode = 'rotate' if abs(w) >= self.heading_hold_w_threshold else 'stop'

        if self.track_width <= 0.0 or self.wheel_radius <= 0.0:
            # 실측값 없으면 안전하게 정지 명령만 유지 (경고는 __init__에서 이미 출력함)
            l_tps, r_tps = 0, 0
        else:
            v_l = v - (w * self.track_width / 2.0)
            v_r = v + (w * self.track_width / 2.0)

            # ★ 좌우 trim 적용 (모터 개체차 미세 보정)
            v_l *= self.left_trim
            v_r *= self.right_trim

            wheel_circumference = 2.0 * math.pi * self.wheel_radius
            l_tps = int(round((v_l / wheel_circumference) * self.ticks_per_rev))
            r_tps = int(round((v_r / wheel_circumference) * self.ticks_per_rev))

        self.last_cmd_l_tps = l_tps
        self.last_cmd_r_tps = r_tps
        self.last_cmd_vel_time = self.get_clock().now()

        self._send_velocity_cmd(l_tps, r_tps)

    # ------------------------------------------------------------------
    def _resend_cmd(self):
        """Arduino의 CMD_TIMEOUT_MS(500ms) 워치독을 충족시키기 위해 주기적으로
        마지막 명령을 재전송한다. 단, ROS 레벨에서도 cmd_vel_timeout_s 동안
        새 /cmd_vel이 없으면 0으로 간주해 전송한다 (이중 안전장치)."""
        l_tps, r_tps = self.last_cmd_l_tps, self.last_cmd_r_tps

        if self.last_cmd_vel_time is not None:
            elapsed = (self.get_clock().now() - self.last_cmd_vel_time).nanoseconds / 1e9
            if elapsed > self.cmd_vel_timeout:
                l_tps, r_tps = 0, 0

        self._send_velocity_cmd(l_tps, r_tps)

    def _send_velocity_cmd(self, l_tps: int, r_tps: int):
        if self._ser is None:
            self._connect_serial()
            if self._ser is None:
                return
        if self._in_reset_grace():
            return   # Arduino가 아직 부팅 중 - 보내봐야 유실됨
        line = f"V,{l_tps},{r_tps}\n"
        try:
            with self._ser_lock:
                self._ser.write(line.encode('ascii'))
        except Exception as e:
            self.get_logger().warn(f"시리얼 쓰기 실패: {e}")
            self._ser = None

    # ------------------------------------------------------------------
    # 시리얼에서 "E,<dl>,<dr>,<dt_ms>" 라인을 읽어 오도메트리 갱신
    def _poll_serial(self):
        if self._ser is None:
            self._connect_serial()
            if self._ser is None:
                return

        if self._in_reset_grace():
            return   # Arduino가 아직 부팅 중 - 읽어봐야 빈 값

        try:
            with self._ser_lock:
                # 이 CH341(클론) 드라이버는 in_waiting(TIOCINQ)이 항상 0을
                # 반환하는 알려진 문제가 있어 이에 의존하지 않는다.
                # 대신 짧은 타임아웃으로 바로 read를 시도한다.
                chunk = self._ser.read(256).decode('ascii', errors='ignore')
                if not chunk:
                    return
                # 정상 수신 상태를 눈으로 계속 확인할 수 있게, 로그 폭탄 없이
                # 대략 5초(250회 = 20ms * 250)에 한 번만 상태를 알려준다.
                self._rx_ok_count = getattr(self, '_rx_ok_count', 0) + 1
                if self._rx_ok_count % 250 == 0:
                    self.get_logger().info(
                        f"[시리얼 상태] 정상 수신 중 (누적 {self._rx_ok_count}회), "
                        f"최근 내용: {chunk!r}"
                    )
        except Exception as e:
            self.get_logger().warn(f"시리얼 읽기 실패: {e}")
            self._ser = None
            return

        self._rx_buffer += chunk
        while '\n' in self._rx_buffer:
            line, self._rx_buffer = self._rx_buffer.split('\n', 1)
            line = line.strip()
            if line:
                self._handle_encoder_line(line)

    def _handle_encoder_line(self, line: str):
        if not line.startswith('E,'):
            return
        parts = line.split(',')
        if len(parts) != 4:
            self.get_logger().debug(f"엔코더 라인 형식 이상, 무시: {line}")
            return

        try:
            delta_l = int(parts[1])
            delta_r = int(parts[2])
            dt_ms = int(parts[3])
        except ValueError:
            self.get_logger().debug(f"엔코더 라인 파싱 실패, 무시: {line}")
            return

        if dt_ms <= 0:
            return

        dt_s = dt_ms / 1000.0

        raw_msg = Int32MultiArray()
        raw_msg.data = [delta_l, delta_r, dt_ms]
        self.raw_ticks_pub.publish(raw_msg)

        self._update_odometry(delta_l, delta_r, dt_s)

    # ------------------------------------------------------------------
    def _update_odometry(self, delta_l_ticks: int, delta_r_ticks: int, dt_s: float):
        if self.track_width <= 0.0 or self.wheel_radius <= 0.0:
            # 실측값이 아직 없으면 오도메트리를 왜곡된 값으로 발행하지 않고
            # 그냥 건너뛴다 (0,0,0에 머무는 것으로 취급 - 정지 상태와 구분 안 되므로
            # 실측값 세팅 전에는 이 브리지의 오도메트리를 신뢰하면 안 됨).
            return

        wheel_circumference = 2.0 * math.pi * self.wheel_radius
        dist_l = (delta_l_ticks / self.ticks_per_rev) * wheel_circumference
        dist_r = (delta_r_ticks / self.ticks_per_rev) * wheel_circumference

        d_center = (dist_l + dist_r) / 2.0
        d_theta = (dist_r - dist_l) / self.track_width

        # 중점 근사로 적분 (짧은 dt에서 오차가 작음)
        mid_yaw = self.yaw + d_theta / 2.0
        self.x += d_center * math.cos(mid_yaw)
        self.y += d_center * math.sin(mid_yaw)
        self.yaw = math.atan2(math.sin(self.yaw + d_theta), math.cos(self.yaw + d_theta))

        vx = d_center / dt_s
        vyaw = d_theta / dt_s

        self._publish_odom(vx, vyaw)

    def _publish_odom(self, vx: float, vyaw: float):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = yaw_to_quaternion(self.yaw)

        msg.twist.twist.linear.x = vx
        msg.twist.twist.angular.z = vyaw

        # ekf_local.yaml의 odom0_config가 속도(vx, vyaw)만 fuse하도록 되어 있으므로
        # pose covariance는 크게 잡아 "이 위치값은 신뢰하지 말라"는 신호를 준다.
        # (이 브리지 자체가 순수 적분값이라 drift가 누적되기 때문)
        msg.pose.covariance[0] = 999999.0   # x
        msg.pose.covariance[7] = 999999.0   # y
        msg.pose.covariance[35] = 999999.0  # yaw

        msg.twist.covariance[0] = 0.02   # vx
        msg.twist.covariance[35] = 0.05  # vyaw

        self.odom_pub.publish(msg)

    # ------------------------------------------------------------------
    def destroy_node(self):
        try:
            self._send_velocity_cmd(0, 0)  # 종료 시 정지 명령 한 번 더
        except Exception:
            pass
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
