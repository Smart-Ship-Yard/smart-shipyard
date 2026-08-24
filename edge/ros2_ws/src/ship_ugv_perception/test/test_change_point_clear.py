#!/usr/bin/env python3
"""change_point.clear_verdict 상태기계 검증.

    python3 test/test_change_point_clear.py

치워짐 판정은 **처음 그 불을 본 로봇 위치로 돌아왔는가**를 기준으로
한다. 그 자리에서는 불이 분명히 보였으므로, 같은 자리에서 안 보이면
배에 가려진 게 아니라 진짜 없는 것이다. 실제 사고 네 건이 근거다:

① 프론트 핑이 반짝하고 곧장 꺼졌다 — 막 검출해 아직 그 자리에 있는데
   YOLO 가 몇 프레임 놓친 것만으로 지워버렸다.
   -> left_vantage(그 자리를 한 번은 벗어나야 한다)로 막는다.
② clear 판정이 거리만 보고 카메라가 그쪽을 보는지는 안 봤다.
   -> in_camera_fov / 헤딩 허용오차로 막는다.
③ has_left_once 가 clear_radius(2.0m) 기준이라 순찰 기하상 영영 못
   벗어나 clear 가 아예 발동 못 했다(반지름 1.00, 불이 중심 0.30m 면
   로봇~불 거리가 0.70~1.30m).
   -> 기준을 revisit_radius_m(0.35m)로 좁혀 한 바퀴면 반드시 벗어난다.
④ "마지막으로 본 지 60초" 방식은 배 가림(약 30초)을 피하려다 최악
   2.2바퀴(110초)가 걸렸다.
   -> 위치 기준으로 바꿔 한 바퀴 안에 판정한다.
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'ship_ugv_perception'))
from change_point import clear_verdict, in_camera_fov, quat_to_yaw, angle_diff

RV, YT, G = 0.35, math.radians(45.0), 3.0   # revisit_radius_m, yaw_tol, grace


def check(name, verdict, expected):
    assert verdict == expected, f'{name}: got {verdict!r}, expected {expected!r}'
    print(f'  {name}: {verdict}  OK')


def cv(dist, yawd=0.0, last_seen=0.0, now=100.0, left=True):
    return clear_verdict(dist, yawd, last_seen, now, RV, YT, G, left)


if __name__ == '__main__':
    # 그 자리에서 멀리 있음 -> 판정 안 함. 다만 "벗어난 적 있음"으로 표시된다
    v, left = cv(dist=1.0, left=False)
    check('처음 본 자리에서 멀다', v, 'wait')
    assert left is True, '벗어났는데 left_vantage 가 안 켜졌다'

    # ★ 사고 재현 ①: 막 검출해서 아직 그 자리에 있다 -> 지우면 안 된다
    #   (예전 "핑이 반짝하고 꺼짐" 사고)
    v, left = cv(dist=0.1, last_seen=0.0, now=100.0, left=False)
    check('막 검출, 아직 그 자리 -> 판정 안 함', v, 'wait')
    assert left is False

    # ★ 사고 재현 ②: 예전 has_left_once 는 clear_radius(2.0m)를 기준으로 삼아
    #   순찰 기하상 영영 못 벗어나 clear 가 아예 발동 못 했다. 여기서는
    #   기준이 0.35m 라 한 바퀴만 돌면 반드시 벗어난다.
    _, left = cv(dist=1.0, left=False)          # 한 바퀴 도는 중
    v, left = cv(dist=0.1, last_seen=0.0, now=100.0, left=left)   # 그 자리로 복귀
    check('한 바퀴 뒤 복귀 -> 판정 가능', v, 'clear')

    # 그 자리로 돌아왔지만 방금 다시 봤다 -> 아직 있다
    v, _ = cv(dist=0.1, last_seen=99.0, now=100.0, left=True)
    check('복귀했는데 방금 봄', v, 'wait')

    # 그 자리로 돌아왔고 grace 만큼 안 보였다 -> 치워짐
    v, _ = cv(dist=0.1, last_seen=90.0, now=100.0, left=True)
    check('복귀했는데 안 보임 -> 확정', v, 'clear')

    # 위치는 맞지만 딴 데를 보고 있다(복구 회전 등) -> 판정 안 함
    v, _ = cv(dist=0.1, yawd=math.radians(90), last_seen=0.0, now=100.0, left=True)
    check('위치는 맞지만 헤딩이 다름', v, 'wait')

    # 경계: grace 정확히 채움
    v, _ = cv(dist=0.1, last_seen=100.0 - G, now=100.0, left=True)
    check('경계값(정확히 grace)', v, 'clear')

    # angle_diff 는 -pi~pi 로 감싼다
    assert abs(angle_diff(math.radians(350), math.radians(10)) - math.radians(20)) < 1e-9
    print('  angle_diff 350도 vs 10도 = 20도  OK')

    # ★ 사고 재현 (주현 진단): camera_yaw_deg=-90 상태에서 순찰 왕복
    #   구간을 반대 방향으로 지나가면 카메라가 이벤트 반대쪽을 본다.
    hfov = math.radians(74.0)
    cam_yaw = math.radians(-90.0)
    ev_x, ev_y = 0.0, -1.0

    # 로봇이 정면(yaw=0)일 때는 카메라(우측 고정)가 정확히 이벤트를 본다.
    assert in_camera_fov(math.radians(0), cam_yaw, hfov, 0.0, 0.0, ev_x, ev_y)
    print('  정면 주행 중 카메라가 이벤트를 봄: True  OK')

    # 왕복 구간에서 로봇이 180도 돌아 반대로 지나가면, 같은 위치에서도
    # 카메라는 이벤트 반대쪽을 본다 -> FOV 밖.
    assert not in_camera_fov(math.radians(180), cam_yaw, hfov, 0.0, 0.0, ev_x, ev_y)
    print('  반대 방향 주행 중 카메라가 등 돌림: False  OK')

    # quat_to_yaw: 흔한 각도 몇 개로 부호·범위 확인.
    assert abs(quat_to_yaw(0, 0, 0, 1) - 0.0) < 1e-9
    assert abs(quat_to_yaw(0, 0, 1, 0) - math.pi) < 1e-9
    s, c = math.sin(math.pi / 4), math.cos(math.pi / 4)
    assert abs(quat_to_yaw(0, 0, s, c) - math.pi / 2) < 1e-9
    print('  quat_to_yaw 기본 각도 확인  OK')

    print('\n통과')
