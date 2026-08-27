#!/usr/bin/env python3
"""이벤트 기억 저장·복원 왕복 검증 (ROS 없이 돌아간다).

    python3 test/test_change_point_state.py

이게 없으면 _save_state 가 쓰는 키와 _restore_state 가 읽는 키가 어긋나도
**노드가 실제로 재시작될 때까지** 모른다. 오늘 그런 종류의 실수를 두 번
했다(arrived_at / fov_seen 을 덮어쓴 뒤 읽음).
"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'ship_ugv_perception'))
from change_point import ChangePointDetector      # noqa: E402


class FakeTime:
    def __init__(self, ns):
        self.nanoseconds = ns

    def __sub__(self, other):
        return FakeTime(self.nanoseconds - other.nanoseconds)

    def __ge__(self, other):
        return self.nanoseconds >= other.nanoseconds

    def __lt__(self, other):
        return self.nanoseconds < other.nanoseconds


class Fake:
    """_save_state / _restore_state 가 실제로 쓰는 것만 흉내 낸다."""
    _save_state = ChangePointDetector._save_state
    _restore_state = ChangePointDetector._restore_state

    def __init__(self, state_file, now_ns, ttl_ns):
        self.state_file = state_file
        self.reported_events = []
        self._now = FakeTime(now_ns)
        self.event_ttl = FakeTime(ttl_ns)
        self.logs = []

    def get_clock(self):
        return self

    def now(self):
        return self._now

    def get_logger(self):
        return self

    def info(self, m):
        self.logs.append(m)

    def warn(self, m):
        self.logs.append(m)


S = 1_000_000_000

if __name__ == '__main__':
    d = tempfile.mkdtemp()
    path = os.path.join(d, 'sub', 'events.json')

    # change_point.py 의 Time 을 테스트용으로 바꿔 끼운다 (rclpy 없이 돌리기 위해)
    import change_point
    change_point.Time = lambda nanoseconds=0: FakeTime(nanoseconds)

    now = 1000 * S
    ttl = 600 * S

    a = Fake(path, now, ttl)
    a.reported_events = [
        {'class_id': 'fire', 'event_id': 'fire@0.10,-1.20',
         'x': 0.1, 'y': -1.2, 'last_seen': FakeTime(now - 5 * S),
         'seen_from': (0.5, -0.3), 'seen_yaw': 1.23,
         'left_vantage': True, 'arrived_at': 995.0, 'fov_seen': 2.5},
        # TTL 을 넘긴 항목 — 복원되면 안 된다
        {'class_id': 'fire', 'event_id': 'fire@9.00,9.00',
         'x': 9.0, 'y': 9.0, 'last_seen': FakeTime(now - 700 * S),
         'seen_from': (1.0, 1.0), 'seen_yaw': 0.0,
         'left_vantage': False, 'arrived_at': None, 'fov_seen': 0.0},
    ]
    a._save_state()
    assert os.path.exists(path), '저장 파일이 안 만들어졌다 (하위 디렉터리 생성 실패?)'
    print('  저장 OK (하위 디렉터리도 만듦)')

    b = Fake(path, now, ttl)
    b._restore_state()
    assert len(b.reported_events) == 1, \
        f'복원 결과가 1건이 아니다: {[e["event_id"] for e in b.reported_events]}'
    r = b.reported_events[0]
    assert r['event_id'] == 'fire@0.10,-1.20'
    print('  TTL 넘긴 항목은 안 복원함  OK')

    src = a.reported_events[0]
    for k in ('class_id', 'event_id', 'x', 'y', 'seen_yaw',
              'left_vantage', 'arrived_at', 'fov_seen'):
        assert r[k] == src[k], f'{k} 가 왕복에서 달라졌다: {r[k]!r} != {src[k]!r}'
    assert r['seen_from'] == src['seen_from'], 'seen_from 이 튜플로 안 돌아왔다'
    assert r['last_seen'].nanoseconds == src['last_seen'].nanoseconds
    print('  모든 필드가 왕복에서 보존됨  OK')

    # 깨진 파일 -> 빈 상태로 시작하고 죽지 않는다
    with open(path, 'w') as f:
        f.write('{ 이건 JSON 이 아니다')
    c = Fake(path, now, ttl)
    c._restore_state()
    assert c.reported_events == [], '깨진 파일인데 뭔가 복원됐다'
    assert any('못 읽었' in m for m in c.logs), '경고를 안 찍었다'
    print('  깨진 파일이면 경고 후 빈 상태로 시작  OK')

    # 파일이 없으면 조용히 빈 상태
    os.remove(path)
    e = Fake(path, now, ttl)
    e._restore_state()
    assert e.reported_events == [] and not e.logs
    print('  파일 없으면 조용히 빈 상태  OK')

    print('\n통과')
