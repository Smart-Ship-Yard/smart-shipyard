#!/usr/bin/env python3
"""
square_test_node.py
--------------------
motion_controller에게 "1m 직진 -> 90도 좌회전"을 4번 반복시켜 정사각형을 그리게
하는 검증용 노드.

[왜 사각형인가 - 오도메트리 캘리브레이션 검증의 표준 방법]
  로봇이 정확히 제자리로 돌아오면 track_width_m / wheel_radius_m 실측값이
  맞다는 뜻이다. 어긋나는 방향으로 오차를 보고 무엇이 틀렸는지 알 수 있다:
    - 변의 길이가 실제보다 짧거나 길다  -> wheel_radius_m 이 틀림
    - 회전이 90도보다 덜/더 돈다          -> track_width_m 이 틀림
    - 사각형이 계속 한쪽으로 휜다          -> 좌우 모터 특성 차이 (PWM 타이머
      비대칭 등) 또는 바퀴 지름 좌우 편차

  예: 실제로는 90도를 돌아야 하는데 매번 85도만 돈다면, 로봇이 회전을 덜 한
      것이므로 track_width_m을 실제보다 크게 잡고 있다는 뜻이다.

[사용법]
  ros2 run ship_ugv_motion_control square_test_node
  # 시작 신호를 보내면 진행
  ros2 topic pub --once /motion/square_start std_msgs/msg/Empty "{}"

[주의]
  반드시 로봇 주변에 사람/장애물이 없는 넓은 공간에서 실행할 것.
  중단하려면: ros2 topic pub --once /motion/cancel std_msgs/msg/Empty "{}"
  (motion_controller가 즉시 정지하고, 이 노드도 다음 단계 진행을 멈춘다)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Empty, String


class SquareTestNode(Node):

    def __init__(self):
        super().__init__('square_test')

        self.declare_parameter('side_length_m', 1.0)
        self.declare_parameter('turn_angle_deg', 90.0)
        self.declare_parameter('num_sides', 4)
        self.declare_parameter('pause_between_steps_s', 1.0)

        self.side_length = self.get_parameter('side_length_m').value
        self.turn_angle = self.get_parameter('turn_angle_deg').value
        self.num_sides = self.get_parameter('num_sides').value
        self.pause_s = self.get_parameter('pause_between_steps_s').value

        # 실행할 동작 목록을 미리 만들어둔다: [직진, 회전, 직진, 회전, ...]
        self.steps = []
        for i in range(self.num_sides):
            self.steps.append(('move', self.side_length))
            self.steps.append(('rotate', self.turn_angle))

        self.step_index = 0
        self.running = False
        self.waiting_for_done = False
        self.pause_timer = None

        self.move_pub = self.create_publisher(Float64, '/motion/move_distance', 10)
        self.rotate_pub = self.create_publisher(Float64, '/motion/rotate_angle', 10)
        self.create_subscription(String, '/motion/status', self._status_cb, 10)
        self.create_subscription(Empty, '/motion/square_start', self._start_cb, 10)
        self.create_subscription(Empty, '/motion/cancel', self._cancel_cb, 10)

        self.get_logger().info(
            f"square_test 대기 중: 한 변 {self.side_length}m, 회전 {self.turn_angle}도, "
            f"{self.num_sides}변. 시작하려면 "
            "'ros2 topic pub --once /motion/square_start std_msgs/msg/Empty \"{}\"'"
        )

    # ------------------------------------------------------------------
    def _start_cb(self, msg: Empty):
        if self.running:
            self.get_logger().warn("이미 실행 중입니다.")
            return
        self.step_index = 0
        self.running = True
        self.get_logger().info("사각형 주행 시작")
        self._send_next_step()

    def _cancel_cb(self, msg: Empty):
        if not self.running:
            return
        self.running = False
        self.waiting_for_done = False
        self.get_logger().warn("사각형 주행 중단됨")

    # ------------------------------------------------------------------
    def _status_cb(self, msg: String):
        if not self.running or not self.waiting_for_done:
            return

        # motion_controller가 완료를 알리면 다음 단계로
        if 'DONE_MOVE' in msg.data or 'DONE_ROTATE' in msg.data:
            self.waiting_for_done = False
            self.step_index += 1
            # 다음 동작 전에 잠깐 쉰다 (관성으로 인한 잔여 움직임이 가라앉도록)
            self.pause_timer = self.create_timer(self.pause_s, self._on_pause_done)

        elif 'TIMEOUT' in msg.data or 'CANCELLED' in msg.data:
            self.get_logger().error(f"동작 실패({msg.data}) - 사각형 주행 중단")
            self.running = False
            self.waiting_for_done = False

    def _on_pause_done(self):
        if self.pause_timer is not None:
            self.pause_timer.cancel()
            self.pause_timer = None
        self._send_next_step()

    # ------------------------------------------------------------------
    def _send_next_step(self):
        if not self.running:
            return

        if self.step_index >= len(self.steps):
            self.get_logger().info(
                "사각형 주행 완료. 로봇이 출발 지점 근처로 돌아왔는지 눈으로 확인하세요. "
                "많이 벗어났다면 track_width_m / wheel_radius_m 값을 재점검할 것."
            )
            self.running = False
            return

        kind, value = self.steps[self.step_index]
        msg = Float64()
        msg.data = float(value)

        if kind == 'move':
            self.get_logger().info(
                f"[{self.step_index + 1}/{len(self.steps)}] 직진 {value}m")
            self.move_pub.publish(msg)
        else:
            self.get_logger().info(
                f"[{self.step_index + 1}/{len(self.steps)}] 회전 {value}도")
            self.rotate_pub.publish(msg)

        self.waiting_for_done = True


def main(args=None):
    rclpy.init(args=args)
    node = SquareTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
