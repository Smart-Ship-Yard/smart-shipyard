#!/usr/bin/env python3
"""
motion_controller_node.py
--------------------------
"지정한 거리만큼 직진/후진", "지정한 각도만큼 제자리 회전"을 수행하는 상위 제어 노드.

[피드백 소스: /odometry/local]
  IMU 단독이 아니라 ekf_local이 엔코더(선속도) + IMU(각속도)를 융합해 만든
  /odometry/local을 피드백으로 쓴다. 이유:
    - IMU만으로는 "몇 미터 갔는지"를 알 수 없다 (가속도 2중 적분은 발산함.
      실제로 ekf_local.yaml에서 ax/ay를 끈 이유가 정확히 이것).
    - 엔코더만으로는 바퀴 미끄러짐(slip) 시 회전각이 부정확하다.
    - /odometry/global(UWB 절대위치)은 앵커가 3개 이상 있어야 좌표가 나오므로
      현재(앵커 2개) 사용 불가. 앵커가 다 도착하면 odom_topic 파라미터만
      /odometry/global로 바꿔서 절대 좌표 기반 제어로 확장할 수 있다.

[출력: /cmd_vel]
  wheel_odom_bridge가 이 토픽을 구독해 차동구동 역기구학 -> ticks/sec 변환 ->
  Arduino PID까지 처리한다. 이 노드는 그 위에 "목표까지 남은 오차를 보고
  속도를 조절하는" 계층만 얹는다.

[안전장치 - 3중]
  1) 이 노드: 목표 도달 / 타임아웃 / 취소 시 즉시 0 속도 발행 후 IDLE
  2) wheel_odom_bridge: cmd_vel_timeout_s(0.5초) 동안 새 명령 없으면 0으로 간주
  3) Arduino 펌웨어: CMD_TIMEOUT_MS(500ms) 워치독으로 자체 정지
  => 이 노드가 죽어도 로봇은 최대 0.5초 안에 반드시 멈춘다.

[사용법 - CLI에서 바로 테스트 가능]
  # 0.5m 전진
  ros2 topic pub --once /motion/move_distance std_msgs/msg/Float64 "{data: 0.5}"
  # 0.3m 후진 (음수)
  ros2 topic pub --once /motion/move_distance std_msgs/msg/Float64 "{data: -0.3}"
  # 좌회전 90도 (CCW 양수, REP-103)
  ros2 topic pub --once /motion/rotate_angle std_msgs/msg/Float64 "{data: 90.0}"
  # 즉시 중단
  ros2 topic pub --once /motion/cancel std_msgs/msg/Empty "{}"
  # 진행 상태 확인
  ros2 topic echo /motion/status
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Empty, String


def quaternion_to_yaw(q) -> float:
    """쿼터니언 -> yaw(rad). 2D 주행이므로 yaw만 뽑는다."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_to_pi(angle: float) -> float:
    """각도를 -pi ~ +pi 범위로 정규화. 359도와 -1도가 같은 값이 되도록."""
    return math.atan2(math.sin(angle), math.cos(angle))


class MotionState:
    IDLE = 'IDLE'
    MOVING = 'MOVING'
    ROTATING = 'ROTATING'


class MotionControllerNode(Node):

    def __init__(self):
        super().__init__('motion_controller')

        # ---- 토픽 파라미터 ----
        self.declare_parameter('odom_topic', '/odometry/local')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        # ---- 속도 제한 ----
        self.declare_parameter('max_linear_speed', 0.15)   # m/s
        self.declare_parameter('max_angular_speed', 0.6)   # rad/s
        # ★ 최소 속도: 모터에는 데드밴드(너무 낮은 PWM에서는 아예 안 도는 구간)가
        #   있어서, 목표에 가까워져 속도 명령이 너무 작아지면 로봇이 멈춘 채로
        #   영원히 오차를 못 줄이는 상태가 된다. 그래서 오차가 남아있는 동안에는
        #   이 값 아래로 내려가지 않도록 바닥을 깔아준다.
        self.declare_parameter('min_linear_speed', 0.04)   # m/s
        self.declare_parameter('min_angular_speed', 0.15)  # rad/s

        # ---- P 제어 게인 ----
        self.declare_parameter('kp_linear', 0.8)
        self.declare_parameter('kp_angular', 1.5)

        # ---- 도달 판정 ----
        self.declare_parameter('distance_tolerance_m', 0.02)     # 2cm
        self.declare_parameter('angle_tolerance_deg', 2.0)       # 2도
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('timeout_s', 30.0)  # 이 시간 넘으면 실패로 간주하고 정지

        # ---- 직진 중 방향 유지(heading hold) ----
        # 좌우 모터 특성이 미세하게 달라 직진해도 조금씩 휘는 것을 보정한다.
        self.declare_parameter('enable_heading_hold', True)
        self.declare_parameter('kp_heading_hold', 1.0)

        odom_topic = self.get_parameter('odom_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.max_v = self.get_parameter('max_linear_speed').value
        self.max_w = self.get_parameter('max_angular_speed').value
        self.min_v = self.get_parameter('min_linear_speed').value
        self.min_w = self.get_parameter('min_angular_speed').value
        self.kp_v = self.get_parameter('kp_linear').value
        self.kp_w = self.get_parameter('kp_angular').value
        self.dist_tol = self.get_parameter('distance_tolerance_m').value
        self.angle_tol = math.radians(self.get_parameter('angle_tolerance_deg').value)
        self.timeout_s = self.get_parameter('timeout_s').value
        self.enable_heading_hold = self.get_parameter('enable_heading_hold').value
        self.kp_hold = self.get_parameter('kp_heading_hold').value

        # ---- 내부 상태 ----
        self.state = MotionState.IDLE
        self.odom_ready = False          # 첫 오도메트리 수신 여부
        self.cur_x = 0.0
        self.cur_y = 0.0
        self.cur_yaw = 0.0

        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0
        self.target_distance = 0.0       # 부호 있음 (음수면 후진)
        self.target_angle = 0.0          # 부호 있음 (양수면 CCW)
        self.accumulated_yaw = 0.0       # 회전 누적각 (±180도 넘는 회전 지원)
        self.prev_yaw_for_accum = 0.0
        self.motion_start_time = None

        # ---- 통신 ----
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 20)
        self.create_subscription(Float64, '/motion/move_distance',
                                 self._move_distance_cb, 10)
        self.create_subscription(Float64, '/motion/rotate_angle',
                                 self._rotate_angle_cb, 10)
        self.create_subscription(Empty, '/motion/cancel', self._cancel_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.status_pub = self.create_publisher(String, '/motion/status', 10)

        rate = self.get_parameter('control_rate_hz').value
        self.create_timer(1.0 / rate, self._control_loop)

        self.get_logger().info(
            f"motion_controller 시작: 피드백={odom_topic}, 출력={cmd_vel_topic}, "
            f"max_v={self.max_v}m/s, max_w={self.max_w}rad/s, "
            f"허용오차={self.dist_tol}m / {math.degrees(self.angle_tol):.1f}도"
        )

    # ------------------------------------------------------------------
    def _odom_cb(self, msg: Odometry):
        self.cur_x = msg.pose.pose.position.x
        self.cur_y = msg.pose.pose.position.y
        self.cur_yaw = quaternion_to_yaw(msg.pose.pose.orientation)

        # 회전 누적각 갱신 (±180도를 넘는 회전도 정확히 추적하기 위함)
        if self.state == MotionState.ROTATING:
            delta = wrap_to_pi(self.cur_yaw - self.prev_yaw_for_accum)
            self.accumulated_yaw += delta
            self.prev_yaw_for_accum = self.cur_yaw

        self.odom_ready = True

    # ------------------------------------------------------------------
    def _move_distance_cb(self, msg: Float64):
        if not self._can_start_new_motion('직진'):
            return

        self.target_distance = msg.data
        self.start_x = self.cur_x
        self.start_y = self.cur_y
        self.start_yaw = self.cur_yaw          # 직진 중 유지할 목표 방향
        self.motion_start_time = self.get_clock().now()
        self.state = MotionState.MOVING

        self.get_logger().info(
            f"직진 시작: 목표 {self.target_distance:+.3f}m "
            f"(시작 위치 x={self.start_x:.3f}, y={self.start_y:.3f})"
        )
        self._publish_status(f"MOVING target={self.target_distance:+.3f}m")

    # ------------------------------------------------------------------
    def _rotate_angle_cb(self, msg: Float64):
        if not self._can_start_new_motion('회전'):
            return

        self.target_angle = math.radians(msg.data)
        self.start_yaw = self.cur_yaw
        self.prev_yaw_for_accum = self.cur_yaw
        self.accumulated_yaw = 0.0
        self.motion_start_time = self.get_clock().now()
        self.state = MotionState.ROTATING

        self.get_logger().info(
            f"회전 시작: 목표 {msg.data:+.1f}도 "
            f"(시작 yaw={math.degrees(self.start_yaw):.1f}도)"
        )
        self._publish_status(f"ROTATING target={msg.data:+.1f}deg")

    # ------------------------------------------------------------------
    def _can_start_new_motion(self, what: str) -> bool:
        """새 동작을 시작해도 되는지 확인. 오도메트리가 아직 없으면 거부한다
        (시작 위치를 모르는 채로 움직이면 목표 거리를 계산할 수 없음)."""
        if not self.odom_ready:
            self.get_logger().warn(
                f"{what} 명령 무시: 아직 오도메트리(/odometry/local)를 한 번도 못 받았습니다. "
                "ekf_local이 떠 있는지, 엔코더/IMU가 연결됐는지 확인하세요."
            )
            self._publish_status("REJECTED no_odometry")
            return False

        if self.state != MotionState.IDLE:
            self.get_logger().warn(
                f"{what} 명령 무시: 이미 {self.state} 상태입니다. "
                "먼저 /motion/cancel로 중단하세요."
            )
            self._publish_status(f"REJECTED busy_{self.state}")
            return False

        return True

    # ------------------------------------------------------------------
    def _cancel_cb(self, msg: Empty):
        if self.state == MotionState.IDLE:
            return
        self.get_logger().warn(f"{self.state} 중단 요청 수신 - 즉시 정지")
        self._stop_and_idle("CANCELLED")

    # ------------------------------------------------------------------
    def _control_loop(self):
        if self.state == MotionState.IDLE:
            return

        # 타임아웃 안전장치
        elapsed = (self.get_clock().now() - self.motion_start_time).nanoseconds / 1e9
        if elapsed > self.timeout_s:
            self.get_logger().error(
                f"{self.state} 타임아웃({self.timeout_s}초 초과) - 정지합니다. "
                "바퀴가 헛돌거나 목표가 너무 멀지 않은지 확인하세요."
            )
            self._stop_and_idle("TIMEOUT")
            return

        if self.state == MotionState.MOVING:
            self._control_move()
        elif self.state == MotionState.ROTATING:
            self._control_rotate()

    # ------------------------------------------------------------------
    def _control_move(self):
        traveled = math.hypot(self.cur_x - self.start_x, self.cur_y - self.start_y)
        # 이동 거리는 항상 양수로 나오므로, 목표 부호에 맞춰 방향을 부여한다
        signed_traveled = traveled * (1.0 if self.target_distance >= 0 else -1.0)
        error = self.target_distance - signed_traveled

        if abs(error) < self.dist_tol:
            self.get_logger().info(
                f"직진 완료: 목표 {self.target_distance:+.3f}m, "
                f"실제 {signed_traveled:+.3f}m (오차 {error:+.3f}m)"
            )
            self._stop_and_idle("DONE_MOVE")
            return

        v = self.kp_v * error
        v = self._clamp_with_deadband(v, self.min_v, self.max_v)

        # 직진 중 방향 유지: 시작 방향에서 벗어난 만큼 살짝 되돌린다
        w = 0.0
        if self.enable_heading_hold:
            yaw_error = wrap_to_pi(self.start_yaw - self.cur_yaw)
            w = self.kp_hold * yaw_error
            w = max(-self.max_w, min(self.max_w, w))  # 데드밴드 없이 순수 클램프

        self._publish_cmd(v, w)

    # ------------------------------------------------------------------
    def _control_rotate(self):
        error = self.target_angle - self.accumulated_yaw

        if abs(error) < self.angle_tol:
            self.get_logger().info(
                f"회전 완료: 목표 {math.degrees(self.target_angle):+.1f}도, "
                f"실제 {math.degrees(self.accumulated_yaw):+.1f}도 "
                f"(오차 {math.degrees(error):+.1f}도)"
            )
            self._stop_and_idle("DONE_ROTATE")
            return

        w = self.kp_w * error
        w = self._clamp_with_deadband(w, self.min_w, self.max_w)

        self._publish_cmd(0.0, w)

    # ------------------------------------------------------------------
    def _clamp_with_deadband(self, value: float, min_abs: float, max_abs: float) -> float:
        """부호는 유지하면서 크기를 [min_abs, max_abs] 범위로 맞춘다.
        min_abs가 필요한 이유: 모터 데드밴드 때문에 너무 작은 명령은 아예
        움직임을 만들지 못해, 목표 근처에서 로봇이 멈춘 채 오차만 남는
        교착 상태가 되기 때문."""
        sign = 1.0 if value >= 0 else -1.0
        magnitude = abs(value)
        magnitude = max(min_abs, min(max_abs, magnitude))
        return sign * magnitude

    # ------------------------------------------------------------------
    def _publish_cmd(self, v: float, w: float):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def _stop_and_idle(self, reason: str):
        """정지 명령을 여러 번 확실히 보내고 IDLE로 복귀.
        한 번만 보내면 패킷 유실 시 로봇이 계속 굴러갈 수 있으므로 3회 반복한다
        (wheel_odom_bridge의 재전송/워치독과 함께 3중 안전장치를 이룸)."""
        for _ in range(3):
            self._publish_cmd(0.0, 0.0)
        self.state = MotionState.IDLE
        self.motion_start_time = None
        self._publish_status(reason)

    def _publish_status(self, text: str):
        msg = String()
        msg.data = f"{self.state}: {text}"
        self.status_pub.publish(msg)

    # ------------------------------------------------------------------
    def destroy_node(self):
        # 노드가 내려갈 때도 반드시 정지 명령을 남긴다
        try:
            for _ in range(3):
                self._publish_cmd(0.0, 0.0)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotionControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
