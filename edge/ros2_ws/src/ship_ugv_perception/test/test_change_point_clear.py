#!/usr/bin/env python3
"""change_point.clear_verdict 상태기계 검증.

    python3 test/test_change_point_clear.py

실제 사고: fire 를 정지-확인-재개했는데 반바퀴도 못 가서 같은 불로 다시
정지했다. 원인은 event_gate_node/websocket_client 가 change_point.py 의
위치 기반 중복 제거를 거치지 않고 원본 검출을 직접 봤기 때문이다.
이 파일은 그 중복 제거에 새로 얹은 "치워짐 확인" 상태기계만 검증한다.
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
    # 반경 밖 -> 항상 reset, 지켜본 시간과 무관
    v, r = clear_verdict(dist_m=1.0, range_entered_at_s=None,
                         last_seen_s=0.0, now_s=100.0,
                         clear_radius_m=R, clear_watch_s=W)
    check('반경 밖', v, 'reset')
    assert r is None

    # 방금 들어옴 -> wait, 시계가 지금 시각으로 켜진다
    v, r = clear_verdict(dist_m=0.3, range_entered_at_s=None,
                         last_seen_s=0.0, now_s=100.0,
                         clear_radius_m=R, clear_watch_s=W)
    check('방금 진입', v, 'wait')
    assert r == 100.0, r

    # 들어온 지 1초 (판정시간 3초 미만) -> wait, 시계 유지
    v, r = clear_verdict(dist_m=0.3, range_entered_at_s=100.0,
                         last_seen_s=50.0, now_s=101.0,
                         clear_radius_m=R, clear_watch_s=W)
    check('판정시간 미달', v, 'wait')
    assert r == 100.0

    # 3초 지났지만 이번 방문 중(진입 이후)에 다시 잡혔다 -> 아직 있다
    v, r = clear_verdict(dist_m=0.3, range_entered_at_s=100.0,
                         last_seen_s=101.5, now_s=104.0,
                         clear_radius_m=R, clear_watch_s=W)
    check('이번 방문 중 재검출', v, 'wait')

    # 3초 지났고 last_seen 이 그 전 방문(진입 이전) 값 그대로 -> 치워짐
    v, r = clear_verdict(dist_m=0.3, range_entered_at_s=100.0,
                         last_seen_s=90.0, now_s=104.0,
                         clear_radius_m=R, clear_watch_s=W)
    check('재검출 없음 -> 확정', v, 'clear')

    # 실제 사고 재현: 확정 클리어 직후 반경을 벗어났다가 다시 들어오면
    # 새로 시계가 켜져야 한다 (이전 판정을 우려먹지 않는다)
    v, r = clear_verdict(dist_m=2.0, range_entered_at_s=100.0,
                         last_seen_s=90.0, now_s=104.0,
                         clear_radius_m=R, clear_watch_s=W)
    check('클리어 전 반경 이탈 -> reset', v, 'reset')
    assert r is None

    print('\n통과')
