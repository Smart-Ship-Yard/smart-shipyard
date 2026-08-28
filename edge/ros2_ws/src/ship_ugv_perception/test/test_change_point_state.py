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
    # clock_type 을 흉내 내야 한다 — 복원 코드가 now.clock_type 을 읽어
    # 되살린 Time 의 시계 종류를 맞추기 때문이다(2026-08-28 버그의 핵심).
    def __init__(self, ns, clock_type='ROS_TIME'):
        self.nanoseconds = ns
        self.clock_type = clock_type

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
    change_point.Time = (lambda nanoseconds=0, clock_type='ROS_TIME':
                         FakeTime(nanoseconds, clock_type))

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
    # 정체성 정보는 그대로 살아야 한다
    for k in ('class_id', 'event_id', 'x', 'y', 'seen_yaw'):
        assert r[k] == src[k], f'{k} 가 왕복에서 달라졌다: {r[k]!r} != {src[k]!r}'
    assert r['seen_from'] == src['seen_from'], 'seen_from 이 튜플로 안 돌아왔다'
    print('  정체성 필드(위치·id·seen_from)는 보존됨  OK')

    # ★ 판정 진행상황은 **일부러 초기화된다** (2026-08-29 사고).
    #   종료 전 값을 되살리면 켜자마자 치워짐이 나간다:
    #     now - last_seen = 꺼져 있던 시간 >= min_unseen_s
    #     max_departure   = 지난 세션 값   >= min_departure_m
    #     fov_seen        = 3초 이상이면 그대로 통과
    #   -> 불이 눈앞에 있는데 시동 몇 초 만에 "치워졌다" 를 보냈다.
    assert r['left_vantage'] is False, '진행상황(left_vantage)이 되살아났다'
    assert r['arrived_at'] is None, '진행상황(arrived_at)이 되살아났다'
    assert r['fov_seen'] == 0.0, '진행상황(fov_seen)이 되살아났다'
    assert r['max_departure'] == 0.0, '진행상황(max_departure)이 되살아났다'
    assert r['last_seen'].nanoseconds == b._now.nanoseconds, \
        'last_seen 이 종료 전 시각 그대로다 — 켜자마자 min_unseen_s 를 통과한다'
    print('  판정 진행상황은 초기화됨 (켜자마자 치워짐 방지)  OK')

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

    # ★ 진짜 rclpy Time 으로도 확인한다 (2026-08-28).
    #   위 FakeTime 은 clock_type 개념이 없어서, 실제로 터진 버그를 못 잡았다:
    #       Time(nanoseconds=...) 의 기본은 SYSTEM_TIME
    #       node.get_clock().now() 는 ROS_TIME
    #       둘을 빼면 TypeError -> except 가 "깨진 항목" 으로 삼킴
    #   그래서 기억 파일이 한 번도 복원되지 않았는데 테스트는 계속 통과했다.
    #   **가짜로 대체한 바로 그 부분이 깨졌으면 테스트는 의미가 없다.**
    try:
        from rclpy.time import Time as RclTime
        from rclpy.duration import Duration as RclDuration
        from rclpy.clock import Clock, ClockType
    except ImportError:
        print('  (rclpy 없음 — 실제 Time 검증 건너뜀)')
    else:
        node_now = Clock(clock_type=ClockType.ROS_TIME).now()
        # 복원이 하는 것과 똑같이 만든다
        revived = RclTime(nanoseconds=node_now.nanoseconds - 60 * 10**9,
                          clock_type=node_now.clock_type)
        age = node_now - revived          # clock_type 이 다르면 여기서 TypeError
        assert age.nanoseconds > 0
        assert not (age >= RclDuration(seconds=600)), 'TTL 판정이 뒤집혔다'
        print('  실제 rclpy Time 왕복 + TTL 비교  OK')

        # clock_type 을 안 맞추면 정말로 터지는지 (회귀의 정확한 형태)
        try:
            _ = node_now - RclTime(nanoseconds=node_now.nanoseconds)
            raise AssertionError('clock_type 이 달라도 뺄셈이 됐다 - 테스트가 무의미')
        except TypeError:
            print('  clock_type 안 맞추면 TypeError 나는 것 확인  OK')

    print('\n통과')
