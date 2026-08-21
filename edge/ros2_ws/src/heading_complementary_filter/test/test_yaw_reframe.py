#!/usr/bin/env python3
"""캘리브레이션이 바뀌었을 때 yaw 를 새 좌표계로 옮기는 식의 부호를 확인한다.

    yaw_new = wrap(yaw_old + (tf_yaw_new - tf_yaw_old))

부호를 반대로 쓰면 어긋남이 **두 배**가 되는데, 화면으로는 잘 안 보이고
주행이 미묘하게 틀어지는 식으로만 드러난다. 그래서 남겨둔다.

식을 다시 적어 맞추면 의미가 없으므로, 실제 좌표 변환으로 검증한다:
map_point = R(tf_yaw) * uwb_point 라는 정의에서 출발해, 같은 물리 방향의
map 각도가 tf_yaw 변화량만큼 움직이는지 본다.

    python3 test/test_yaw_reframe.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from heading_complementary_filter.complementary_filter_node import wrap_to_pi


def map_angle_of(uwb_dir_rad, tf_yaw):
    """uwb_frame 의 방향 하나를 map 좌표로 옮겨 그 각도를 잰다."""
    ux, uy = math.cos(uwb_dir_rad), math.sin(uwb_dir_rad)
    c, s = math.cos(tf_yaw), math.sin(tf_yaw)
    return math.atan2(s * ux + c * uy, c * ux - s * uy)


def check(uwb_dir_deg, tf_old_deg, tf_new_deg):
    uwb_dir = math.radians(uwb_dir_deg)
    tf_old, tf_new = math.radians(tf_old_deg), math.radians(tf_new_deg)

    yaw_old = map_angle_of(uwb_dir, tf_old)      # 옛 좌표계에서 본 로봇 방향
    yaw_true = map_angle_of(uwb_dir, tf_new)     # 새 좌표계에서 본 같은 방향

    delta = wrap_to_pi(tf_new - tf_old)
    yaw_fixed = wrap_to_pi(yaw_old + delta)      # ← 노드가 쓰는 식

    err = abs(wrap_to_pi(yaw_fixed - yaw_true))
    assert err < 1e-9, (
        f'uwb_dir={uwb_dir_deg} tf {tf_old_deg}->{tf_new_deg}: '
        f'{math.degrees(yaw_fixed):.3f} != {math.degrees(yaw_true):.3f}')
    return math.degrees(delta)


if __name__ == '__main__':
    cases = [
        (0, 0, -3.52),      # 실제로 겪은 값 (2026-08-21)
        (14.9, 0, -3.52),
        (120, -10, 40),
        (-170, 170, -170),  # wrap 경계
        (45, 0, 179),
        (45, 0, -179),
    ]
    for uwb_dir, a, b in cases:
        d = check(uwb_dir, a, b)
        print(f'  방향 {uwb_dir:>6.1f}도, TF {a:>6.1f} -> {b:>6.1f}  '
              f'(변화 {d:+7.1f}도)  OK')
    print('\n부호 확인: yaw_new = yaw_old + (tf_new - tf_old). 통과')
