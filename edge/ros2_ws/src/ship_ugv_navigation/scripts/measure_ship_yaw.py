#!/usr/bin/env python3
"""로봇을 배와 나란히 세워두고 배의 방향(yaw)을 잰다.

    python3 scripts/measure_ship_yaw.py <맵이름>

왜 재야 하나
------------
"캘리브레이션 때 로봇을 배와 나란히 두면 배 yaw = 0" 이라는 약속에 기대
왔는데, 실측하니 -7.8도였다(2026-08-21). 두 가지가 겹친다.
  · 눈대중으로 맞춘 배치 오차 — 0.77 m 배에서 8도면 끝이 10 cm 어긋난다
  · 캘리브레이션 주행이 휘는 것 — map +x 는 주행 궤적에 맞춘 직선 방향이라
    출발 순간의 로봇 방향과 다르다 (실측 0.2도 수준, 배치 오차보다는 작다)
0.91 m 마스크에서 8도면 모서리가 6.3 cm 밀려 여유 7 cm 를 거의 다 먹는다.

왜 로봇 yaw 를 그냥 읽으면 안 되나
----------------------------------
방위각 추정 자체에 오차가 있다. 최초 yaw 를 0.3 m 구간의 UWB 진행방향으로
잡는데 UWB 노이즈가 ±0.15 m 라 십수 도씩 틀어진다(실측 15.5도, 손으로 들어
옮긴 뒤에는 85도까지 갔다). 그래서 **라이다-맵 정합으로 그 오차를 먼저
재서 빼야** 한다. 스캔을 몇 도 돌렸을 때 맵의 벽에 가장 잘 맞는지 보면 된다.

    배 yaw = (로봇이 배와 나란할 때 읽은 yaw) + (라이다가 말하는 보정각)
"""
import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_patrol_space import load_map

MAPS = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'maps'))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('map_name', help='맵 이름 (예: shipyard_map_hall_v6)')
    ap.add_argument('--seconds', type=float, default=6.0)
    a = ap.parse_args()

    yaml_path = os.path.join(MAPS, a.map_name + '.yaml')
    if not os.path.exists(yaml_path):
        print(f'  ❌ 맵이 없다: {yaml_path}')
        return 1
    m = load_map(yaml_path)

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    import tf2_ros

    rclpy.init()
    n = Node('measure_ship_yaw')
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, n)
    scans = []
    n.create_subscription(LaserScan, '/scan_filtered', lambda s: scans.append(s), 1)

    print(f'  {a.seconds:.0f}초 읽는다 — 로봇을 움직이지 말 것')
    end, tr = time.time() + a.seconds, None
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.2)
        if scans and tr is None:
            try:
                tr = buf.lookup_transform('map', scans[-1].header.frame_id,
                                          rclpy.time.Time())
            except Exception:
                pass
    if not scans or tr is None:
        print('  ❌ 스캔이나 TF 를 못 얻었다 — 터미널 1(위치추정)이 떠 있나?')
        rclpy.shutdown()
        return 1

    s = scans[-1]
    q = tr.transform.rotation
    th0 = math.atan2(2 * (q.w * q.z), 1 - 2 * q.z * q.z)
    tx, ty = tr.transform.translation.x, tr.transform.translation.y
    pts = [(d, s.angle_min + i * s.angle_increment)
           for i, d in enumerate(s.ranges)
           if s.range_min < d < s.range_max
           and not math.isinf(d) and not math.isnan(d)]

    def occ(x, y):
        r = int(round((m['h'] - 1) - (y - m['oy']) / m['res']))
        c = int(round((x - m['ox']) / m['res']))
        return 0 <= r < m['h'] and 0 <= c < m['w'] and not m['free'][r][c]

    def fit(dth):
        th = th0 + dth
        hit = sum(1 for d, ang in pts
                  if any(occ(tx + d * math.cos(th + ang) + ex,
                             ty + d * math.sin(th + ang) + ey)
                         for ex in (-0.1, 0, 0.1) for ey in (-0.1, 0, 0.1)))
        return 100.0 * hit / max(1, len(pts))

    # 굵게 훑고 그 근처를 0.5도 간격으로 다시 훑는다
    best = max(((d, fit(math.radians(d))) for d in range(-180, 181, 5)),
               key=lambda t: t[1])
    fine = [best[0] + x * 0.5 for x in range(-10, 11)]
    best = max(((d, fit(math.radians(d))) for d in fine), key=lambda t: t[1])

    robot_yaw = math.degrees(th0)
    ship_yaw = ((robot_yaw + best[0] + 90) % 180) - 90   # 사각형이라 ±90 로 정리

    print(f'  로봇 방위 (map)   {robot_yaw:+.1f}도   스캔 {len(pts)}점')
    print(f'  라이다 보정       {best[0]:+.1f}도   (그때 벽 정합 {best[1]:.1f}%)')
    if best[1] < 60:
        print('  ⚠️ 정합이 60% 미만이다 — 위치추정이 많이 틀어졌거나 맵이 안 맞는다.')
        print('     1 m 직진시켜 방위각을 다시 잡고 다시 재는 것을 권한다.')
    print()
    print(f'  ▶ 배 yaw = {ship_yaw:+.1f}도')
    print()
    print('  기존 명령 뒤에 이것만 붙이면 된다:')
    print(f'    python3 scripts/finalize_map.py {a.map_name} '
          f'--ship-yaw {ship_yaw:.1f}')
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
