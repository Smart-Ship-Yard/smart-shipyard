#!/usr/bin/env python3
"""change_point.clear_verdict 상태기계 검증.

    python3 test/test_change_point_clear.py

실제 사고 두 건:
① fire 를 정지-확인-재개했는데 반바퀴도 못 가서 같은 불로 다시 정지했다.
   원인: event_gate_node/websocket_client 가 change_point.py 의 위치 기반
   중복 제거를 거치지 않고 원본 검출을 직접 봤다.
② 그 첫 번째 수정 직후, 프론트 핑이 반짝하고 곧장 꺼졌다.
   원인: 불을 막 보고한 순간 로봇은 이미 반경 안에 서 있는데, 자리를 뜬
   적이 한 번도 없어도 "치워짐" 판정 시계가 그냥 시작해버렸다.
이 파일은 ②의 has_left_once 상태기계를 검증한다(①은 노드 배선 문제라
순수 함수 테스트 대상이 아니다).
"""
import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'ship_ugv_perception'))
from change_point import clear_verdict

R, W = 0.6, 3.0   # clear_radius_m, clear_watch_s


def check(name, verdict, expected):
    assert verdict == expected, f'{name}: got {verdict!r}, expected {expected!r}'
    print(f'  {name}: {verdict}  OK')


if __name__ == '__main__':
    # 반경 밖 -> 항상 reset, 지켜본 시간과 무관. has_left_once 는 True 로 켜진다
    v, r, left = clear_verdict(dist_m=1.0, range_entered_at_s=None,
                               last_seen_s=0.0, now_s=100.0,
                               clear_radius_m=R, clear_watch_s=W,
                               has_left_once=False)
    check('반경 밖', v, 'reset')
    assert r is None and left is True

    # ★ 사고 재현: 방금 보고돼 반경 안에 있는데 아직 한 번도 안 떠났다.
    #   3초가 훌쩍 지나도(now_s 를 크게 줘도) 판정을 시작하면 안 된다.
    v, r, left = clear_verdict(dist_m=0.3, range_entered_at_s=None,
                               last_seen_s=0.0, now_s=100.0,
                               clear_radius_m=R, clear_watch_s=W,
                               has_left_once=False)
    check('막 보고됨, 아직 안 떠남 -> 시계 안 켬', v, 'wait')
    assert r is None, f'has_left_once=False 인데 시계가 켜졌다: {r}'
    assert left is False

    # 이제 반경을 벗어난다 -> has_left_once 가 True 로 바뀐다
    v, r, left = clear_verdict(dist_m=1.0, range_entered_at_s=None,
                               last_seen_s=0.0, now_s=105.0,
                               clear_radius_m=R, clear_watch_s=W,
                               has_left_once=False)
    check('처음으로 반경 이탈', v, 'reset')
    assert left is True

    # 떠난 적이 있는 채로 방금 재진입 -> wait, 시계가 지금 시각으로 켜진다
    v, r, left = clear_verdict(dist_m=0.3, range_entered_at_s=None,
                               last_seen_s=0.0, now_s=110.0,
                               clear_radius_m=R, clear_watch_s=W,
                               has_left_once=True)
    check('떠났다 재진입', v, 'wait')
    assert r == 110.0 and left is True

    # 들어온 지 1초 (판정시간 3초 미만) -> wait, 시계 유지
    v, r, left = clear_verdict(dist_m=0.3, range_entered_at_s=110.0,
                               last_seen_s=50.0, now_s=111.0,
                               clear_radius_m=R, clear_watch_s=W,
                               has_left_once=True)
    check('판정시간 미달', v, 'wait')
    assert r == 110.0

    # 3초 지났지만 이번 방문 중(진입 이후)에 다시 잡혔다 -> 아직 있다
    v, r, left = clear_verdict(dist_m=0.3, range_entered_at_s=110.0,
                               last_seen_s=111.5, now_s=114.0,
                               clear_radius_m=R, clear_watch_s=W,
                               has_left_once=True)
    check('이번 방문 중 재검출', v, 'wait')

    # 3초 지났고 last_seen 이 그 전 방문(진입 이전) 값 그대로 -> 치워짐
    v, r, left = clear_verdict(dist_m=0.3, range_entered_at_s=110.0,
                               last_seen_s=90.0, now_s=114.0,
                               clear_radius_m=R, clear_watch_s=W,
                               has_left_once=True)
    check('재검출 없음 -> 확정 (재방문한 경우에만)', v, 'clear')

    # 확정 클리어 직후 반경을 벗어났다가 다시 들어오면 새로 시계가 켜져야 한다
    v, r, left = clear_verdict(dist_m=2.0, range_entered_at_s=110.0,
                               last_seen_s=90.0, now_s=114.0,
                               clear_radius_m=R, clear_watch_s=W,
                               has_left_once=True)
    check('클리어 전 반경 이탈 -> reset', v, 'reset')
    assert r is None

    print('\n통과')
