#!/usr/bin/env python3
"""change_point.clear_verdict 상태기계 검증.

    python3 test/test_change_point_clear.py

실제 사고 세 건이 이 파일의 테스트 근거다:
① 프론트 핑이 반짝하고 곧장 꺼졌다 — 막 보고한 이벤트를 정지-확인 사이
   몇 초만으로 "재방문했는데 없다"고 오판해 지워버렸다.
   -> 지금은 min_age_s(이벤트 나이) 조건으로 막는다.
② clear 판정이 거리만 보고 카메라가 실제로 그쪽을 보는지는 안 봤다.
   -> in_camera_fov 게이트 (호출부에서 적용).
③ has_left_once("반경을 한 번은 벗어나야 판정 시작")가 순찰 기하에
   의존해 영영 발동 못 했다 — 순찰 반지름 1.00 에 불이 중심 0.30m 면
   로봇~불 거리가 0.70~1.30m 라 clear_radius 2.0 을 못 벗어난다.
   그 바람에 이벤트가 안 지워지고 쌓여, 나중엔 그 근처 어디에 불을
   놓아도 "기존 이벤트"로 먹혀 정지조차 안 했다.
   -> has_left_once 를 걷어내고 ①의 나이 조건으로 대체했다.
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'ship_ugv_perception'))
from change_point import clear_verdict, in_camera_fov, quat_to_yaw

R, W = 2.0, 15.0   # clear_radius_m, clear_watch_s


def check(name, verdict, expected):
    assert verdict == expected, f'{name}: got {verdict!r}, expected {expected!r}'
    print(f'  {name}: {verdict}  OK')


if __name__ == '__main__':
    AGE = 30.0          # min_event_age_s
    T0 = 100.0          # 이벤트가 생긴 시각(first_seen_s)
    OLD = T0 + AGE + 1  # 나이 조건을 넘긴 시각

    # 판정 범위 밖 -> 항상 reset, 지켜본 시간과 무관
    v, r = clear_verdict(dist_m=3.0, range_entered_at_s=None,
                         last_seen_s=T0, now_s=OLD,
                         clear_radius_m=R, clear_watch_s=W,
                         first_seen_s=T0, min_age_s=AGE)
    check('판정 범위 밖', v, 'reset')
    assert r is None

    # ★ 사고 재현 ①: 막 보고된 이벤트는 나이가 안 차 판정을 시작 못 한다.
    #   (정지-확인 사이 몇 초로 방금 보고한 것을 지워버리던 것)
    v, r = clear_verdict(dist_m=0.3, range_entered_at_s=None,
                         last_seen_s=T0, now_s=T0 + 5.0,
                         clear_radius_m=R, clear_watch_s=W,
                         first_seen_s=T0, min_age_s=AGE)
    check('막 보고됨(5초) -> 시계 안 켬', v, 'wait')
    assert r is None, f'나이가 안 찼는데 시계가 켜졌다: {r}'

    # ★ 사고 재현 ②: 로봇이 clear_radius 를 영영 못 벗어나는 순찰 기하에서도
    #   나이만 차면 판정이 시작돼야 한다 (예전 has_left_once 는 여기서 막혔다).
    v, r = clear_verdict(dist_m=0.7, range_entered_at_s=None,
                         last_seen_s=T0, now_s=OLD,
                         clear_radius_m=R, clear_watch_s=W,
                         first_seen_s=T0, min_age_s=AGE)
    check('반경 못 벗어나도 나이 차면 시계 켬', v, 'wait')
    assert r == OLD, f'시계가 안 켜졌다: {r}'

    # 시계 켠 지 1초 (판정시간 W 미만) -> wait, 시계 유지
    v, r = clear_verdict(dist_m=0.3, range_entered_at_s=OLD,
                         last_seen_s=T0, now_s=OLD + 1.0,
                         clear_radius_m=R, clear_watch_s=W,
                         first_seen_s=T0, min_age_s=AGE)
    check('판정시간 미달', v, 'wait')
    assert r == OLD

    # 판정시간은 지났지만 그동안 다시 잡혔다 -> 아직 있다
    v, r = clear_verdict(dist_m=0.3, range_entered_at_s=OLD,
                         last_seen_s=OLD + 1.0, now_s=OLD + W + 1.0,
                         clear_radius_m=R, clear_watch_s=W,
                         first_seen_s=T0, min_age_s=AGE)
    check('지켜보는 중 재검출', v, 'wait')

    # 판정시간 지났고 last_seen 이 시계보다 이전 -> 치워짐
    v, r = clear_verdict(dist_m=0.3, range_entered_at_s=OLD,
                         last_seen_s=T0, now_s=OLD + W + 1.0,
                         clear_radius_m=R, clear_watch_s=W,
                         first_seen_s=T0, min_age_s=AGE)
    check('재검출 없음 -> 확정', v, 'clear')

    # 확정 직전에 범위를 벗어나면 시계를 접는다
    v, r = clear_verdict(dist_m=3.0, range_entered_at_s=OLD,
                         last_seen_s=T0, now_s=OLD + W + 1.0,
                         clear_radius_m=R, clear_watch_s=W,
                         first_seen_s=T0, min_age_s=AGE)
    check('확정 전 범위 이탈 -> reset', v, 'reset')
    assert r is None

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
