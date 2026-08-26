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

R, W = 2.0, 60.0   # clear_radius_m, clear_watch_s


def check(name, verdict, expected):
    assert verdict == expected, f'{name}: got {verdict!r}, expected {expected!r}'
    print(f'  {name}: {verdict}  OK')


if __name__ == '__main__':
    AGE = 10.0          # min_event_age_s
    T0 = 100.0          # 이벤트가 생긴 시각(first_seen_s)
    OLD = T0 + AGE + 1  # 나이 조건을 넘긴 시각

    # 판정 범위 밖 -> 아무리 오래 안 보였어도 판정하지 않는다
    v = clear_verdict(dist_m=3.0, last_seen_s=T0, now_s=T0 + 1000,
                      clear_radius_m=R, clear_watch_s=W,
                      first_seen_s=T0, min_age_s=AGE)
    check('판정 범위 밖', v, 'wait')

    # ★ 사고 재현 ①: 막 보고된 이벤트는 나이가 안 차 판정하지 않는다.
    #   (정지-확인 사이 몇 초로 방금 보고한 것을 지워버리던 것 = 핑 반짝임)
    v = clear_verdict(dist_m=0.3, last_seen_s=T0, now_s=T0 + 5.0,
                      clear_radius_m=R, clear_watch_s=W,
                      first_seen_s=T0, min_age_s=AGE)
    check('막 보고됨(5초) -> 판정 안 함', v, 'wait')

    # ★ 사고 재현 ②: 배에 가려 30초쯤 안 보이는 구간은 넘어가야 한다.
    #   (실측 가림 구간 약 30초 < clear_watch_s 60초)
    v = clear_verdict(dist_m=1.0, last_seen_s=OLD, now_s=OLD + 30.0,
                      clear_radius_m=R, clear_watch_s=W,
                      first_seen_s=T0, min_age_s=AGE)
    check('배에 30초 가림 -> 아직 있다고 본다', v, 'wait')

    # ★ 사고 재현 ③: 검출이 이어지는 한 영영 판정되면 안 된다.
    #   예전 range_entered_at 방식은 반대로 **영영 판정이 안 됐다**.
    #   여기서는 last_seen 이 갱신되는 한 자연히 밀린다.
    for elapsed in (0.0, 100.0, 1000.0):
        v = clear_verdict(dist_m=1.0, last_seen_s=T0 + elapsed,
                          now_s=T0 + elapsed + 1.0,
                          clear_radius_m=R, clear_watch_s=W,
                          first_seen_s=T0, min_age_s=AGE)
        assert v == 'wait', f'검출이 이어지는데 판정됨 (경과 {elapsed}s)'
    print('  검출이 이어지는 한 판정 안 함 (0/100/1000초): wait  OK')

    # 진짜로 치웠다 -> clear_watch_s 지나면 확정
    v = clear_verdict(dist_m=1.0, last_seen_s=OLD, now_s=OLD + W + 1.0,
                      clear_radius_m=R, clear_watch_s=W,
                      first_seen_s=T0, min_age_s=AGE)
    check('치움 -> 60초 뒤 확정', v, 'clear')

    # 경계: 딱 clear_watch_s 만큼 지났으면 확정
    v = clear_verdict(dist_m=1.0, last_seen_s=OLD, now_s=OLD + W,
                      clear_radius_m=R, clear_watch_s=W,
                      first_seen_s=T0, min_age_s=AGE)
    check('경계값(정확히 60초)', v, 'clear')

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
