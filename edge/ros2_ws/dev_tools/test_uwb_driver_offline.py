#!/usr/bin/env python3
"""
test_uwb_driver_offline.py
----------------------------
실제 UWB 하드웨어(J-Link 시리얼 포트) 없이 uwb_dwm1001_driver의 핵심 로직을
검증한다: DWM1001 lec 출력 라인 파싱 + 3중 아웃라이어 게이팅(QF/야드경계/속도).

방법: pyserial의 serial.Serial을 가짜 객체(FakeSerial)로 몽키패치해서,
노드가 "진짜처럼" 시리얼을 열고 읽게 만든다. FakeSerial은 미리 짜둔
DWM1001 출력 라인 시퀀스를 그대로 흘려보낸다.

시나리오 (7개 라인, 각각 다른 게이트를 테스트하도록 설계):
  1. 정상 POS (QF=90, 야드 내부)                    -> 통과 예상
  2. QF 낮음 (QF=30 < 임계값 60)                     -> 게이트1 탈락 예상
  3. 야드 경계 밖 좌표 (x=999)                       -> 게이트2 탈락 예상
  4. 순간 이동(직전 유효샘플에서 비현실적으로 먼 거리) -> 게이트3 탈락 예상
  5. 정상 이동 (라인1에서 자연스럽게 조금 이동)        -> 통과 예상
  6. DIST 라인 (POS 아님, 거리 정보)                  -> /uwb/distances로만 발행
  7. 다시 정상 POS                                    -> 통과 예상

기대 결과: /uwb/pose에 정확히 3개(라인 1, 5, 7)만 발행되어야 한다.

사용법:
  cd ~/smart-shipyard/edge/ros2_ws
  source install/setup.bash
  python3 dev_tools/test_uwb_driver_offline.py
"""

import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String


class FakeSerial:
    """pyserial.Serial과 같은 인터페이스를 흉내내는 가짜 시리얼 포트.

    실제 하드웨어 대신, 미리 정해둔 DWM1001 lec 출력 라인들을 read()
    호출 시마다 조금씩 흘려보낸다.
    """

    def __init__(self, *args, **kwargs):
        self._script = SCRIPT_LINES.copy()
        self._buf = b''
        self.is_open = True
        # _ensure_lec_mode의 프로브 단계에서 "이미 스트리밍 중"으로 인식되도록
        # 첫 read에서 곧바로 POS, 라인을 흘려보낸다.

    def write(self, data):
        # shell 명령(\r\r, 'lec\r' 등)은 무시 - 이 테스트에선 항상 스트리밍
        # 중인 것으로 간주 (probe에서 already_streaming=True로 처리되게)
        return len(data)

    def read(self, size=1):
        if not self._buf:
            if self._script:
                line = self._script.pop(0)
                self._buf = (line + '\r\n').encode('utf-8')
            else:
                return b''
        chunk, self._buf = self._buf[:size], self._buf[size:]
        return chunk

    @property
    def in_waiting(self):
        if not self._buf and self._script:
            line = self._script.pop(0)
            self._buf = (line + '\r\n').encode('utf-8')
        return len(self._buf)

    def reset_input_buffer(self):
        self._buf = b''

    def close(self):
        self.is_open = False


# 시나리오 라인 (야드 경계 기본값 -50~50 기준)
#
# 주의: _ensure_lec_mode()의 프로브 단계가 시작 시 첫 유효 데이터를
# (파싱/게이팅 없이) 먼저 "소비"해버린다 (POS, 문자열 존재만 확인하고 버림 -
# 실제 하드웨어에서도 동일하게 동작하는 정상 설계).
# 그래서 0번째 줄은 프로브에 먹히는 "미끼"이고, 실제 게이팅 검증은 1번부터.
SCRIPT_LINES = [
    "POS,x,0.00,y,0.00,z,0.00,qf,90",     # 0. 프로브가 소비 (미끼, 카운트 안 됨)
    "POS,x,0.00,y,0.00,z,0.00,qf,90",     # 1. 정상, 원점 -> 통과 (baseline 확립)
    "POS,x,0.10,y,0.00,z,0.00,qf,30",     # 2. QF 낮음 -> 게이트1 탈락
    "POS,x,999.00,y,0.00,z,0.00,qf,90",   # 3. 야드 밖 -> 게이트2 탈락
    "POS,x,40.00,y,0.00,z,0.00,qf,90",    # 4. 직전 유효(원점)에서 40m 순간이동 -> 게이트3 탈락
    "POS,x,0.15,y,0.02,z,0.00,qf,85",     # 5. 원점에서 자연스러운 이동 -> 통과
    "DIST,2,AN0,1783,x,0.00,y,0.00,z,0.00,dist,2.31,AN1,1784,x,1.00,y,0.00,z,0.00,dist,3.02",  # 6. 거리정보
    "POS,x,0.30,y,0.05,z,0.00,qf,88",     # 7. 계속 자연스러운 이동 -> 통과
]


class ResultCollector(Node):
    def __init__(self):
        super().__init__('test_result_collector')
        self.poses = []
        self.distances = []
        self.create_subscription(PoseWithCovarianceStamped, '/uwb/pose',
                                  self._pose_cb, 10)
        self.create_subscription(String, '/uwb/distances',
                                  self._dist_cb, 10)

    def _pose_cb(self, msg):
        self.poses.append((msg.pose.pose.position.x, msg.pose.pose.position.y))

    def _dist_cb(self, msg):
        self.distances.append(msg.data)


def main():
    # --- 몽키패치: 실제 하드웨어 열기 전에 가짜로 교체 ---
    import serial
    serial.Serial = FakeSerial

    # 패치 이후에 import 해야 driver 모듈 내부의 `import serial`이
    # 이미 패치된 상태를 보도록 보장됨
    from uwb_dwm1001_driver.uwb_ros2_publisher import UwbDwm1001Driver

    rclpy.init()

    driver = UwbDwm1001Driver()  # 가짜 시리얼로 "정상적으로" 초기화됨
    collector = ResultCollector()

    # publish_rate_hz(기본 10Hz) 주기로 여러 번 spin 하여 스크립트 라인을 소진
    print("드라이버 + 수집기 실행 중 (2초)...")
    deadline = time.time() + 2.0
    while time.time() < deadline:
        rclpy.spin_once(driver, timeout_sec=0.05)
        rclpy.spin_once(collector, timeout_sec=0.05)

    driver.destroy_node()
    collector_poses = collector.poses
    collector_dists = collector.distances
    collector.destroy_node()
    rclpy.shutdown()

    # --- 검증 ---
    print(f"\n수신된 /uwb/pose 개수: {len(collector_poses)} (기대: 3)")
    for i, (x, y) in enumerate(collector_poses):
        print(f"  [{i}] x={x:.2f}, y={y:.2f}")
    print(f"수신된 /uwb/distances 개수: {len(collector_dists)} (기대: 1)")

    ok = True
    if len(collector_poses) != 3:
        print("❌ FAIL: /uwb/pose 발행 개수가 기대(3)와 다름 — 게이팅 로직 확인 필요")
        ok = False
    else:
        expected = [(0.00, 0.00), (0.15, 0.02), (0.30, 0.05)]
        for (ex, ey), (ax, ay) in zip(expected, collector_poses):
            if abs(ex - ax) > 1e-6 or abs(ey - ay) > 1e-6:
                print(f"❌ FAIL: 좌표 불일치 — 기대({ex},{ey}) vs 실제({ax},{ay})")
                ok = False

    if len(collector_dists) != 1:
        print("❌ FAIL: /uwb/distances 발행 개수가 기대(1)와 다름")
        ok = False

    if ok:
        print("\n✅ PASS: 3중 게이팅(QF/야드경계/속도) 전부 설계대로 동작 확인")
    else:
        print("\n실패 — 위 로그와 driver 로그(게이트 탈락 사유)를 대조해 원인 파악 필요")
        sys.exit(1)


if __name__ == '__main__':
    main()
