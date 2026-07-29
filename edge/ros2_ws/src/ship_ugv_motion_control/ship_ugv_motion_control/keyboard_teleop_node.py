#!/usr/bin/env python3
"""
keyboard_teleop_node.py
-------------------------
터미널에서 WASD로 로봇을 직접 조종하는 노드. 손으로 미는 대신 모터로
정확하게 움직여서, SLAM 매핑 시 "손으로 밀어서 생기는 미끄러짐(slip)"
변수를 없애기 위해 만들었다.

[키 배치]
  w : 전진
  s : 후진
  a : 제자리 좌회전 (CCW, REP-103 양수 방향)
  d : 제자리 우회전 (CW)
  스페이스 또는 x : 즉시 정지
  + 또는 = : 속도 단계 올리기
  - 또는 _ : 속도 단계 내리기
  q 또는 Ctrl+C : 종료

[동작 방식 - 왜 "누르고 있으면 가고, 떼면 선다"처럼 동작하는가]
  터미널은 "키가 눌린 채로 있다"는 상태를 직접 알려주지 않는다. 대신
  OS의 키보드 자동반복(auto-repeat) 기능을 이용한다: 키를 누르고 있으면
  운영체제가 같은 문자를 짧은 간격으로 계속 보내주는데, 이 노드는
  "마지막으로 키를 받은 시각"을 계속 기록해두고, 그 시각으로부터
  IDLE_TIMEOUT(기본 0.3초) 동안 새 키가 안 오면 자동으로 속도를 0으로
  되돌린다. 즉 키를 떼서 자동반복이 멈추면, 0.3초 안에 로봇도 멈춘다.

  이 방식은 프로젝트 전체의 안전 철학(wheel_odom_bridge의 0.5초 타임아웃,
  Arduino의 500ms 워치독)과 같은 맥락 - "명령이 끊기면 반드시 멈춘다".

[사용 전 주의 - 반드시 읽을 것]
  motion_controller_node와 이 노드를 동시에 켜지 말 것. 둘 다 /cmd_vel에
  발행하므로, 동시에 켜면 두 명령이 뒤섞여 로봇이 불규칙하게 움직인다.
  SLAM 매핑용으로 이 노드를 쓸 때는 motion_control.launch.py를 끄고
  이 노드만 단독으로 실행할 것.

[실행법]
  ros2 run ship_ugv_motion_control keyboard_teleop_node
"""

import sys
import termios
import tty
import select
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


INSTRUCTIONS = """
==================================================
 WASD 키보드 텔레옵
==================================================
  w : 전진        s : 후진
  a : 좌회전(CCW)  d : 우회전(CW)
  스페이스/x : 즉시 정지
  +/- : 속도 단계 조절
  q 또는 Ctrl+C : 종료
--------------------------------------------------
  ※ motion_controller_node와 동시에 켜지 말 것
  ※ 키를 떼면 약 0.3초 안에 자동으로 정지함
==================================================
"""


class KeyboardTeleopNode(Node):

    def __init__(self):
        super().__init__('keyboard_teleop')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('linear_step_mps', 0.05)
        self.declare_parameter('angular_step_radps', 0.2)
        self.declare_parameter('max_linear_speed', 0.15)
        self.declare_parameter('max_angular_speed', 0.6)
        self.declare_parameter('idle_timeout_s', 0.3)
        self.declare_parameter('publish_rate_hz', 20.0)
        # ★ 가속/감속 램프 - 목표 속도로 한 번에 점프하지 않고, 주기마다
        #   이 값만큼씩만 다가가게 해서 부드럽게 만든다.
        self.declare_parameter('linear_accel_limit', 0.15)   # m/s^2
        self.declare_parameter('angular_accel_limit', 1.0)   # rad/s^2

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.linear_step = self.get_parameter('linear_step_mps').value
        self.angular_step = self.get_parameter('angular_step_radps').value
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.idle_timeout = self.get_parameter('idle_timeout_s').value
        publish_rate = self.get_parameter('publish_rate_hz').value

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        # ---- 조종 상태 (키보드 스레드와 타이머가 공유, lock으로 보호) ----
        self._lock = threading.Lock()
        self._target_v = 0.0        # 키보드가 "원하는" 목표 속도
        self._target_w = 0.0
        self._current_v = 0.0       # 실제로 지금 내보내는 속도 (서서히 목표를 따라감)
        self._current_w = 0.0
        self._last_key_time = time.monotonic()
        self._last_publish_time = time.monotonic()
        self._speed_scale = 1.0  # +/- 로 조절되는 배율 (0.2 ~ 2.0로 제한)
        self.linear_accel_limit = self.get_parameter('linear_accel_limit').value
        self.angular_accel_limit = self.get_parameter('angular_accel_limit').value

        self._stop_flag = threading.Event()
        self._key_thread = threading.Thread(target=self._key_loop, daemon=True)
        self._key_thread.start()

        self.create_timer(1.0 / publish_rate, self._publish_loop)
        self.create_timer(1.0 / publish_rate, self._idle_check_loop)

        print(INSTRUCTIONS)
        self.get_logger().info(
            f"keyboard_teleop 시작: {cmd_vel_topic} 발행, "
            f"idle_timeout={self.idle_timeout}s, publish_rate={publish_rate}Hz"
        )

    # ------------------------------------------------------------------
    def _key_loop(self):
        """터미널을 raw 모드로 바꿔서 한 글자씩 즉시 읽는다 (엔터 안 눌러도 됨)."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while not self._stop_flag.is_set():
                # 0.1초 타임아웃으로 폴링 - 종료 신호를 놓치지 않기 위함
                ready, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not ready:
                    continue
                c = sys.stdin.read(1)
                self._handle_key(c)
        finally:
            # ★ 반드시 원래 터미널 설정으로 복구 (안 하면 이후 터미널이 이상해짐)
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def _handle_key(self, c: str):
        with self._lock:
            scale = self._speed_scale

            if c == 'w':
                self._target_v = self.max_linear * scale
                self._target_w = 0.0
            elif c == 's':
                self._target_v = -self.max_linear * scale
                self._target_w = 0.0
            elif c == 'a':
                self._target_v = 0.0
                self._target_w = self.max_angular * scale
            elif c == 'd':
                self._target_v = 0.0
                self._target_w = -self.max_angular * scale
            elif c in (' ', 'x'):
                self._target_v = 0.0
                self._target_w = 0.0
            elif c in ('+', '='):
                self._speed_scale = min(2.0, self._speed_scale + 0.1)
                print(f"\r속도 배율: {self._speed_scale:.1f}   ", end='', flush=True)
                return
            elif c in ('-', '_'):
                self._speed_scale = max(0.2, self._speed_scale - 0.1)
                print(f"\r속도 배율: {self._speed_scale:.1f}   ", end='', flush=True)
                return
            elif c == 'q':
                self._stop_flag.set()
                return
            else:
                # 인식 안 되는 키는 무시 (오타로 인한 오동작 방지)
                return

            self._last_key_time = time.monotonic()

    # ------------------------------------------------------------------
    def _idle_check_loop(self):
        """마지막 키 입력 후 idle_timeout이 지나면 자동으로 정지시킨다."""
        with self._lock:
            elapsed = time.monotonic() - self._last_key_time
            if elapsed > self.idle_timeout:
                if self._target_v != 0.0 or self._target_w != 0.0:
                    self._target_v = 0.0
                    self._target_w = 0.0

        if self._stop_flag.is_set():
            self._shutdown()

    def _publish_loop(self):
        now = time.monotonic()
        dt = now - self._last_publish_time
        self._last_publish_time = now
        if dt <= 0.0:
            return

        with self._lock:
            target_v = self._target_v
            target_w = self._target_w

            # ★ 목표를 향해 이번 주기에 허용된 만큼만 다가간다 (가속도 제한)
            self._current_v = self._step_toward(
                self._current_v, target_v, self.linear_accel_limit * dt)
            self._current_w = self._step_toward(
                self._current_w, target_w, self.angular_accel_limit * dt)

            v = self._current_v
            w = self._current_w

        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.cmd_pub.publish(msg)

    @staticmethod
    def _step_toward(current: float, target: float, max_step: float) -> float:
        diff = target - current
        if abs(diff) <= max_step:
            return target
        return current + max_step * (1.0 if diff > 0 else -1.0)

    # ------------------------------------------------------------------
    def _shutdown(self):
        print("\n종료합니다.")
        rclpy.shutdown()

    def destroy_node(self):
        self._stop_flag.set()
        try:
            # 종료 시 정지 명령을 확실히 여러 번 보낸다
            msg = Twist()
            for _ in range(3):
                self.cmd_pub.publish(msg)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
