#!/usr/bin/env python3
"""change_point.clear_verdict 상태기계 검증.

    python3 test/test_change_point_clear.py

치워짐 판정은 **처음 그 불을 본 로봇 위치를 다시 통과했는가**를 기준으로
한다. 그 자리에서는 불이 분명히 보였으므로, 같은 자리를 지나가며 한 번도
못 봤다면 배에 가려진 게 아니라 진짜 없는 것이다. 실제 사고 다섯 건이 근거다:

① 프론트 핑이 반짝하고 곧장 꺼졌다 — 막 검출해 아직 그 자리에 있는데
   YOLO 가 몇 프레임 놓친 것만으로 지워버렸다.
   -> left_vantage(그 자리를 한 번은 벗어나야 한다)로 막는다.
② clear 판정이 거리만 보고 카메라가 그쪽을 보는지는 안 봤다.
   -> 헤딩 허용오차(revisit_yaw_tol_deg)로 막는다.
③ has_left_once 가 clear_radius(2.0m) 기준이라 순찰 기하상 영영 못
   벗어나 clear 가 아예 발동 못 했다(반지름 1.00, 불이 중심 0.30m 면
   로봇~불 거리가 0.70~1.30m).
   -> 기준을 revisit_radius_m(0.35m)로 좁혀 한 바퀴면 반드시 벗어난다.
④ "마지막으로 본 지 60초" 방식은 배 가림(약 30초)을 피하려다 최악
   2.2바퀴(110초)가 걸렸다.
   -> 위치 기준으로 바꿔 한 바퀴 안에 판정한다.
⑤ "도착 후 3초 못 보면 확정" 은 도착 지점이 하필 "막 보이기 시작하는
   경계" 라서 아직 안 보이는 쪽일 수 있었다. 불이 그대로인데 핑이 사라졌다
   (실측 01:11:54, 그 구간 confidence 0.88~0.94 로 임계값과 무관).
   -> 머무는 동안엔 판정하지 않고, **구역을 벗어나는 순간** 통과 전체를
      돌아보고 판정한다.
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


def cv(dist, yawd=0.0, last_seen=0.0, now=100.0, left=True, arrived=None):
    return clear_verdict(dist, yawd, last_seen, now, RV, YT, G, left, arrived)


IN, OUT = 0.1, 1.0        # 구역 안 / 밖 거리


if __name__ == '__main__':
    # 구역 밖 -> 판정 조건 안 맞으면 대기. 재방문 자격은 켜지고 시계는 접힌다
    v, left, arr = cv(dist=OUT, left=False)
    check('구역 밖 (첫 이탈)', v, 'wait')
    assert left is True and arr is None

    # ★ 사고 재현 ①: 막 검출해 아직 그 자리에 있다(정지-확인 대기) -> 지우면 안 됨
    v, left, arr = cv(dist=IN, left=False)
    check('막 검출, 아직 그 자리', v, 'wait')
    assert left is False and arr is None

    # 한 바퀴 뒤 도착 -> 시계만 켜고 판정 안 함
    v, left, arr = cv(dist=IN, now=100.0, left=True, arrived=None)
    check('도착 -> 시계만 켬', v, 'wait')
    assert arr == 100.0

    # ★ 사고 재현 ②(2026-08-27 실측): 도착 직후 3초가 지나도 **머무는 동안엔
    #   판정하지 않는다**. seen_from 은 "막 보이기 시작한 경계"라 도착 지점이
    #   아직 안 보이는 쪽일 수 있다 — 통과를 다 해봐야 안다.
    v, _, arr = cv(dist=IN, last_seen=50.0, now=100.0 + G + 5, left=True, arrived=100.0)
    check('머무는 중에는 판정 안 함', v, 'wait')
    assert arr == 100.0

    # 통과를 마치고 구역을 벗어나는 순간, 그동안 한 번도 못 봤으면 확정
    v, left, arr = cv(dist=OUT, last_seen=50.0, now=100.0 + G + 1, left=True, arrived=100.0)
    check('통과 완료 + 한 번도 못 봄 -> 확정', v, 'clear')
    assert left is True and arr is None

    # 통과 중에 한 번이라도 봤으면 아직 있다
    v, _, _ = cv(dist=OUT, last_seen=100.0 + 2, now=100.0 + G + 1, left=True, arrived=100.0)
    check('통과 중 봤음 -> 아직 있다', v, 'wait')

    # 모퉁이만 스치고 지나감(체류 < grace) -> 유효한 방문으로 안 침
    v, _, _ = cv(dist=OUT, last_seen=50.0, now=100.0 + G - 1, left=True, arrived=100.0)
    check('스치고 지나감 -> 판정 안 함', v, 'wait')

    # 도착했지만 헤딩이 어긋남 -> 유효한 방문으로 시작하지 않음
    v, _, arr = cv(dist=IN, yawd=math.radians(90), now=100.0, left=True, arrived=None)
    check('헤딩 어긋난 채 도착 -> 시계 안 켬', v, 'wait')
    assert arr is None

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
