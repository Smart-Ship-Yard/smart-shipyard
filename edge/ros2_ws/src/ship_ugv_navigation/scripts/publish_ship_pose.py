#!/usr/bin/env python3
"""줄자로 실측한 배 위치를 프론트엔드까지 보낸다.

    python3 scripts/publish_ship_pose.py <X> <Y>

왜 필요한가
-----------
프론트엔드 화면은 배 pose 를 **기준 좌표계**로 쓴다. 로봇도 이벤트 핑도
블록도 전부 배 기준으로 변환해서 그린다(ShipyardTwinDashboard.jsx 의
mapXYToShipLocalMeters). 그래서 배 pose 하나가 틀리면 화면 전체가 틀어진다.

YOLO 측량(ship_survey_node)이 이 값을 만들었는데 못 쓸 정도로 부정확했다
(실측 0.80 x 0.17 m 인 배를 1.68 x 0.35 m 로 쟀다). 그래서 사람이 재서 넣는다.

경로: 이 스크립트 -> /ship_survey/pose -> websocket_client -> 백엔드
      -> MongoDB 저장 + 프론트 브로드캐스트 -> /api/init-data 에도 반영

좌표 규약 (uwb_map_calibration 이 정한다)
-----------------------------------------
  원점    캘리브레이션을 **시작한 순간** 로봇 위치 (직진 전)
  +x      로봇이 직진한 방향
  +y      그 방향 기준 왼쪽
  기준점  base_link = 좌우 구동 바퀴 축의 중점

로봇을 배 왼편에 배와 나란히 두는 약속이라면, 배는 로봇 오른쪽 앞이므로
X 는 양수, Y 는 음수가 된다. yaw 는 그 약속상 0 이다.
"""
import argparse
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from measure_ship_center import MEASURED_PATH, load_measured

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

TOPIC = '/ship_survey/pose'


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    # 인자를 안 주면 measure_ship_center.py 가 저장해둔 값을 읽는다.
    # 사람이 숫자를 옮겨 적다 틀리는 것을 없애려는 것이다.
    ap.add_argument('x', type=float, nargs='?', default=None,
                    help='배 중심의 map X (m). 생략하면 저장된 측정값을 쓴다')
    ap.add_argument('y', type=float, nargs='?', default=None,
                    help='배 중심의 map Y (m). 생략하면 저장된 측정값을 쓴다')
    ap.add_argument('--yaw', type=float, default=0.0,
                    help='배 방향(도). 로봇을 배와 나란히 놓는 약속이면 0 (기본)')
    ap.add_argument('--block-id', default='B1')
    ap.add_argument('--seconds', type=float, default=5.0,
                    help='발행 지속 시간. 구독자 디스커버리에 몇 초가 필요하다')
    a = ap.parse_args()

    if a.x is None or a.y is None:
        got = load_measured()
        if got is None:
            print(f'  ❌ 저장된 배 중심 측정값이 없다: {MEASURED_PATH}')
            print('     먼저 재거나, X Y 를 직접 줄 것:')
            print('       python3 scripts/measure_ship_center.py')
            print('       python3 scripts/publish_ship_pose.py <X> <Y>')
            return 1
        a.x, a.y, meta = got
        print(f'  저장된 측정값을 읽었다: config/{os.path.basename(MEASURED_PATH)}'
              f"  (뎁스 {meta.get('depth_m', float('nan')):.3f} m, "
              f"검출 {meta.get('samples', '?')}개)")

    payload = {
        'event_type': 'ship_pose',
        'block_id': a.block_id,
        'map_xy': [a.x, a.y],
        'yaw': math.radians(a.yaw),
    }

    rclpy.init()
    node = Node('ship_pose_manual')
    # ★ websocket_client 가 TRANSIENT_LOCAL 로 구독한다. VOLATILE 로 발행하면
    #   QoS 가 안 맞아 **아무 일도 안 일어난다** (에러도 안 난다).
    qos = QoSProfile(depth=1,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=QoSReliabilityPolicy.RELIABLE)
    pub = node.create_publisher(String, TOPIC, qos)
    msg = String(data=json.dumps(payload))

    print(f'  {TOPIC} 로 발행: map_xy=({a.x:+.3f}, {a.y:+.3f}) yaw={a.yaw:.1f}도 '
          f'block_id={a.block_id}')
    end = time.time() + a.seconds
    while time.time() < end:
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.2)

    print()
    print(f'  ▶ 배 중심   X = {a.x:+.3f}   Y = {a.y:+.3f}   '
          f'(yaw {a.yaw:.1f}도)')
    print(f'  {a.seconds:.0f}초 발행 완료.')
    print('  확인:  터미널 1 에 [배위치] 로그  /  '
          'curl -s http://192.168.0.5:8000/api/init-data')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
