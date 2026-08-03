#!/usr/bin/env python3
"""
encoder_diag_logger.py
------------------------
/wheel/raw_ticks(좌우 엔코더 델타)를 CSV 파일로 자동 기록하는 진단 도구.

목적: PID/trim 튜닝을 "값 바꾸고 눈으로 보고"의 반복이 아니라,
      한 번의 이동 테스트로 데이터를 전부 기록해서, 그 파일 내용을
      그대로 분석에 넘겨 정확한 보정값을 계산할 수 있게 한다.

사용법:
  터미널 1: ros2 launch ship_ugv_localization localization.launch.py
  터미널 2: ros2 launch ship_ugv_motion_control motion_control.launch.py
  터미널 3: python3 encoder_diag_logger.py
  터미널 4: ros2 topic pub --once /motion/move_distance std_msgs/msg/Float64 "{data: 2.0}"

  이동이 끝나면 터미널 3에서 Ctrl+C로 로거를 종료한다.
  ~/encoder_diag.csv 파일이 생성되며, 이 파일 내용을 그대로 복사해서
  분석 요청 시 붙여넣으면 된다.

CSV 컬럼:
  t_s          : 로거 시작 시점부터 경과 시간(초) - 가속/감속 구간 특정에 사용
  delta_l      : 이번 20ms 구간 왼쪽 엔코더 델타(ticks)
  delta_r      : 이번 20ms 구간 오른쪽 엔코더 델타(ticks)
  dt_ms        : 실제 경과 시간(ms, Arduino가 잰 값)
  cum_l        : 왼쪽 누적 델타(시작부터 지금까지)
  cum_r        : 오른쪽 누적 델타
  ratio_inst   : 이번 구간의 delta_r/delta_l (1.0이면 완벽히 대칭, <1이면 오른쪽이 덜 돎)
  ratio_cum    : 누적 cum_r/cum_l (전체적인 좌우 균형 - trim 계산에 직접 사용 가능)
"""

import csv
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


class EncoderDiagLogger(Node):

    def __init__(self):
        super().__init__('encoder_diag_logger')

        self.declare_parameter('output_path', '/home/ship_yard/encoder_diag.csv')
        self.output_path = self.get_parameter('output_path').value

        self.cum_l = 0
        self.cum_r = 0
        self.start_time = None
        self.rows = []

        self.create_subscription(
            Int32MultiArray, '/wheel/raw_ticks', self._cb, 50)

        self.get_logger().info(
            f"엔코더 진단 로거 시작. 이동 명령을 실행하세요. "
            f"끝나면 Ctrl+C로 종료 -> {self.output_path}에 저장됩니다."
        )

    def _cb(self, msg: Int32MultiArray):
        delta_l, delta_r, dt_ms = msg.data
        now = time.monotonic()
        if self.start_time is None:
            self.start_time = now

        self.cum_l += delta_l
        self.cum_r += delta_r

        ratio_inst = (delta_r / delta_l) if delta_l != 0 else float('nan')
        ratio_cum = (self.cum_r / self.cum_l) if self.cum_l != 0 else float('nan')

        self.rows.append({
            't_s': round(now - self.start_time, 3),
            'delta_l': delta_l,
            'delta_r': delta_r,
            'dt_ms': dt_ms,
            'cum_l': self.cum_l,
            'cum_r': self.cum_r,
            'ratio_inst': round(ratio_inst, 4) if ratio_inst == ratio_inst else '',
            'ratio_cum': round(ratio_cum, 4) if ratio_cum == ratio_cum else '',
        })

    def save(self):
        if not self.rows:
            self.get_logger().warn("기록된 데이터가 없습니다. 저장하지 않습니다.")
            return
        with open(self.output_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=list(self.rows[0].keys()))
            writer.writeheader()
            writer.writerows(self.rows)
        self.get_logger().info(
            f"저장 완료: {self.output_path} ({len(self.rows)}개 샘플). "
            f"최종 cum_l={self.cum_l}, cum_r={self.cum_r}, "
            f"최종 ratio_cum={(self.cum_r/self.cum_l if self.cum_l else 'N/A')}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = EncoderDiagLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
