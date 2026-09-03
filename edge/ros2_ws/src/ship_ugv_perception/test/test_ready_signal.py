#!/usr/bin/env python3
"""로봇 "준비 완료" 판정 (ROS 없이 돌아간다).

    python3 test/test_ready_signal.py

★ 왜 필요한가 (2026-08-29)

대시보드의 "연결됨" 은 서버가 **소켓이 열린 것**을 보고 켠다. 그런데
websocket_client 는 로컬라이제이션에서 뜨고, AMCL 과 Nav2 는 그보다
10~60초 뒤에 뜬다. 그 사이 화면은 준비됐다는데 로봇은 순찰도 못 하고
이벤트도 못 잡는다 — 사용자가 그때 확인 버튼을 눌러도 소용이 없다.

진짜 준비된 시점은 둘 다 맞을 때다:
  · change_point armed  (AMCL 확인 -> 좌표를 믿을 수 있다)
  · patrol 이 WAIT_NAV2 를 벗어남 (Nav2 액션 서버가 응답했다)

두 신호는 어느 쪽이 먼저 올지 모른다. 순서에 안 휘둘려야 한다.
"""
import json


class Fake:
    """websocket_client 의 준비 판정 부분만 흉내 낸다."""

    def __init__(self):
        self.sent = []
        self._armed = False
        self._nav_ready = False
        self._ready_sent = False
        self._ws_live = True

    def on_armed(self, value):
        self._armed = bool(value)
        self._check_ready()

    def on_patrol_status(self, state):
        try:
            state = json.loads(json.dumps({'state': state})).get('state')
        except (ValueError, TypeError):
            return
        if state and state != 'WAIT_NAV2':
            self._nav_ready = True
            self._check_ready()

    def _check_ready(self):
        if self._ready_sent or not (self._armed and self._nav_ready):
            return
        self._ready_sent = True
        self._send_ready()

    def _send_ready(self):
        self.sent.append({'event_type': 'jetson_ready',
                          'armed': self._armed,
                          'nav_ready': self._nav_ready})

    def on_connect(self):
        if self._ready_sent:
            self._send_ready()


if __name__ == '__main__':
    # 한쪽만으로는 안 뜬다 — 이게 핵심이다. AMCL 만 붙고 Nav2 가 없으면
    # 로봇은 순찰을 못 한다. 그때 "준비 완료" 를 띄우면 거짓말이다.
    f = Fake()
    f.on_armed(True)
    assert f.sent == [], 'AMCL 만으로 준비 완료가 나갔다'
    print('  armed 만 -> 안 알림  OK')

    g = Fake()
    g.on_patrol_status('RUNNING')
    assert g.sent == [], 'Nav2 만으로 준비 완료가 나갔다'
    print('  Nav2 만 -> 안 알림  OK')

    # WAIT_NAV2 는 아직 Nav2 를 기다리는 중이다
    h = Fake()
    h.on_armed(True)
    h.on_patrol_status('WAIT_NAV2')
    assert h.sent == [], 'WAIT_NAV2 인데 준비 완료가 나갔다'
    print('  WAIT_NAV2 -> 안 알림  OK')

    # 둘 다 -> 딱 한 번
    f.on_patrol_status('RUNNING')
    assert len(f.sent) == 1, f'준비 완료가 {len(f.sent)}번 나갔다'
    print('  armed + Nav2 -> 1회 알림  OK')

    # 반대 순서여도 결과가 같다
    g.on_armed(True)
    assert len(g.sent) == 1, f'반대 순서에서 {len(g.sent)}번'
    print('  Nav2 먼저 -> armed 나중이어도 알림  OK')

    # 상태가 계속 흘러도 두 번 안 보낸다 (/patrol/status 는 1초마다 온다)
    for st in ['RUNNING'] * 60 + ['STOPPED', 'BLOCKED', 'RUNNING']:
        f.on_patrol_status(st)
    f.on_armed(True)
    assert len(f.sent) == 1, f'상태가 흐르며 {len(f.sent)}번 나갔다'
    print('  상태 63회 추가 수신 -> 여전히 1회  OK')

    # 멈춰 있어도 준비된 것이다 — Nav2 는 살아있고 이벤트로 정지 중일 뿐
    k = Fake()
    k.on_armed(True)
    k.on_patrol_status('STOPPED')
    assert len(k.sent) == 1, 'STOPPED 를 준비 안 된 것으로 봤다'
    print('  STOPPED/BLOCKED 도 준비된 것으로 봄  OK')

    # 재연결하면 프론트가 새로고침됐을 수 있으니 다시 알린다
    before = len(k.sent)
    k.on_connect()
    assert len(k.sent) == before + 1, '재연결 때 준비 상태를 다시 안 알렸다'
    print('  재연결 -> 준비 상태 재통보  OK')

    # 준비 전에 재연결하면 아무것도 안 보낸다
    m = Fake()
    m.on_connect()
    assert m.sent == [], '준비 전인데 재연결에서 보냈다'
    print('  준비 전 재연결 -> 안 보냄  OK')

    print('\n통과')
