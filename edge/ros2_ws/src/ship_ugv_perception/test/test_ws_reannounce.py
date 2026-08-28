#!/usr/bin/env python3
"""재연결 시 위험 이벤트 재통보 + 위치 핑 큐 폭주 방지 (ROS 없이 돌아간다).

    python3 test/test_ws_reannounce.py

★ 왜 필요한가 (2026-08-29)

서버가 재시작하면 프론트는 빈 화면으로 시작한다. 그런데 로봇의
change_point 는 그 불을 이미 기억하고 있어 **재발행을 하지 않는다.**
그래서 불이 눈앞에 있는데 화면에는 아무것도 없는 상태가 된다.

처음에는 "event_ttl(600초) 이 지나면 자동으로 풀린다" 고 봤는데 **틀렸다.**
last_seen 은 재검출마다 갱신되므로, 로봇이 한 바퀴마다 그 불을 보는 한
만료되지 않는다. 즉 **무기한** 이어진다. 그래서 재연결 때 다시 알린다.

그리고 위치 핑은 0.5초마다 만들어지므로 큐에 넣으면 1시간 끊김에 7,200건이
쌓였다가 한꺼번에 쏟아진다. 최신 하나만 의미가 있으므로 슬롯으로 덮어쓴다.
"""
import os
import sys
import queue
import threading

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'ship_ugv_perception'))


class Fake:
    """websocket_client 의 상태 관리 부분만 흉내 낸다."""

    def __init__(self):
        self.send_queue = queue.Queue()
        self._pending_position = None
        self._active_events = {}
        self._active_lock = threading.Lock()

    # 실제 코드와 같은 순서로 동작
    def on_danger(self, event_id, cls='fire'):
        payload = {'event_type': cls, 'event_id': event_id, 'map_xy': [0, 0]}
        if event_id:
            with self._active_lock:
                self._active_events[event_id] = dict(payload)
        self.send_queue.put(payload)

    def on_cleared(self, event_id):
        payload = {'event_type': 'event_cleared', 'event_id': event_id}
        if event_id:
            with self._active_lock:
                self._active_events.pop(event_id, None)
        self.send_queue.put(payload)

    def on_position(self):
        self._pending_position = {'event_type': 'position', 'ekf_global': [0, 0]}

    def reannounce(self):
        with self._active_lock:
            revive = [dict(v) for v in self._active_events.values()]
        for ev in revive:
            ev['replay'] = True
            self.send_queue.put(ev)
        return revive


if __name__ == '__main__':
    f = Fake()

    # 위치 핑은 아무리 많이 만들어도 큐에 안 쌓인다
    for _ in range(7200):          # 1시간치
        f.on_position()
    assert f.send_queue.qsize() == 0, \
        f'위치 핑이 큐에 쌓였다: {f.send_queue.qsize()}건'
    assert f._pending_position is not None, '최신 위치를 안 들고 있다'
    print('  위치 핑 7,200건 -> 큐 0건, 최신 1건만 보관  OK')

    # 이벤트는 하나도 안 버린다
    f.on_danger('fire@0.1,-1.2')
    f.on_danger('no_helmet@0.5,0.3')
    assert f.send_queue.qsize() == 2, '이벤트가 유실됐다'
    print('  이벤트는 큐에 그대로 쌓임  OK')

    # 재연결 -> 살아있는 것만 replay 플래그를 달고 재전송
    revive = f.reannounce()
    assert len(revive) == 2, f'재통보 대상이 2건이 아니다: {len(revive)}'
    assert all(e['replay'] is True for e in revive), 'replay 플래그가 없다'
    ids = {e['event_id'] for e in revive}
    assert ids == {'fire@0.1,-1.2', 'no_helmet@0.5,0.3'}, ids
    print('  재연결 -> 살아있는 2건을 replay 로 재전송  OK')

    # 치워진 것은 재통보 대상에서 빠진다 (되살아나면 안 된다)
    f.on_cleared('fire@0.1,-1.2')
    revive = f.reannounce()
    ids = {e['event_id'] for e in revive}
    assert ids == {'no_helmet@0.5,0.3'}, f'치워진 것이 되살아났다: {ids}'
    print('  치워진 것은 재통보 안 함  OK')

    # 같은 이벤트를 여러 번 받아도 거울은 하나만 (중복 핑 방지)
    for _ in range(5):
        f.on_danger('fire@9,9')
    with f._active_lock:
        assert len(f._active_events) == 2, f._active_events.keys()
    print('  같은 event_id 를 여러 번 받아도 거울은 1건  OK')

    # event_id 가 없는 구버전 메시지는 거울에 안 넣는다 (지울 방법이 없으므로)
    before = len(f._active_events)
    f.on_danger(None)
    assert len(f._active_events) == before, 'event_id 없는 것을 거울에 넣었다'
    print('  event_id 없으면 거울에 안 넣음  OK')

    # ★ 프론트 [초기화] 버튼 (2026-08-29)
    #   유령 핑이 생겼을 때 프로세스를 껐다 켜는 대신 기억만 비운다.
    #   change_point 목록과 여기 거울을 **둘 다** 비워야 한다 — 한쪽만
    #   비우면 다음 재연결에 지워진 이벤트가 되살아난다.
    f.on_danger('fire@1,1')
    f.on_danger('fire@2,2')
    with f._active_lock:
        assert len(f._active_events) > 0

    def reset(data):
        if not isinstance(data, dict) or data.get('event_type') != 'reset_events':
            return
        with f._active_lock:
            f._active_events.clear()

    reset({'event_type': 'stream_boost'})          # 다른 명령은 무시
    with f._active_lock:
        assert len(f._active_events) > 0, '엉뚱한 명령에 거울을 비웠다'
    reset({'event_type': 'reset_events'})
    with f._active_lock:
        assert len(f._active_events) == 0, '초기화가 안 됐다'
    assert f.reannounce() == [], '초기화 뒤에도 재통보가 나간다'
    print('  [초기화] -> 거울 비움, 재통보 없음  OK')
    print('  다른 명령에는 반응 안 함  OK')

    print('\n통과')
