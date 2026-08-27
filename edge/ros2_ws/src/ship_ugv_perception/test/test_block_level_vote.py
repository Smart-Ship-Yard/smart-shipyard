#!/usr/bin/env python3
"""조립 단계 다수결 확정 검증 (ROS 없이 돌아간다).

    python3 test/test_block_level_vote.py

예전 방식("마지막 값이 3초 연속 같아야 확정")은 값이 하나만 달라도 시계가
리셋돼 확정이 영영 안 되거나, 틀린 값이 우연히 연속되면 그게 확정됐다.
실물 검출은 배가 잘리는 각도에서 level3/level4 가 섞여 나온다(실측).
"""
import os
import sys
from collections import Counter, deque

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'ship_ugv_perception'))


class T:
    """rclpy Time 흉내 — 초 단위 float 하나로 충분하다."""
    def __init__(self, s): self.s = s
    def __sub__(self, o): return T(self.s - o.s)
    def __lt__(self, o): return self.s < o.s


def run(levels, stability=3.0, ratio=0.6, min_samples=6, hz=4.0):
    """websocket_client._handle_block_level 의 판정부를 그대로 흉내 낸다."""
    win = deque(); confirmed = None; out = []
    for i, lv in enumerate(levels):
        now = T(i / hz)
        win.append((now, lv))
        cutoff = T(now.s - stability)
        while win and win[0][0] < cutoff:
            win.popleft()
        if len(win) < min_samples:
            continue
        c = Counter(x for _, x in win)
        winner, votes = c.most_common(1)[0]
        if votes / len(win) < ratio:
            continue
        if confirmed == winner:
            continue
        confirmed = winner
        out.append(winner)
    return confirmed, out


if __name__ == '__main__':
    # 깨끗하게 level3 만 → level3 확정
    c, seq = run([3] * 12)
    assert c == 3 and seq == [3], f'{c} {seq}'
    print('  일관된 level3 -> level3 확정  OK')

    # level3 다수 + level4 섞임 → 여전히 level3 (예전 방식이면 계속 리셋됐다)
    c, seq = run([3, 3, 4, 3, 3, 3, 4, 3, 3, 3, 3, 3])
    assert c == 3, f'섞였다고 확정을 못 했다: {c} {seq}'
    print('  level4 섞여도 다수결로 level3 확정  OK')

    # 반반이면 아무것도 확정하지 않는다 (틀린 값 내보내느니 조용한 게 낫다)
    c, seq = run([3, 4] * 8)
    assert c is None, f'의견이 갈리는데 확정했다: {c} {seq}'
    print('  50:50 -> 확정 보류  OK')

    # 표본이 모자라면 확정하지 않는다
    c, seq = run([3, 3, 3])
    assert c is None, f'표본 3개로 확정했다: {c}'
    print('  표본 부족 -> 확정 보류  OK')

    # 진짜로 단계가 바뀌면 따라간다 (창이 밀려 새 값이 다수가 됨)
    c, seq = run([3] * 12 + [4] * 12)
    assert seq == [3, 4], f'단계 변화를 못 따라갔다: {seq}'
    print('  level3 -> level4 실제 변화는 따라감  OK')

    # 같은 값이 계속 와도 재발행하지 않는다
    c, seq = run([3] * 40)
    assert seq == [3], f'같은 값을 여러 번 발행했다: {seq}'
    print('  같은 값 반복 -> 발행 1회뿐  OK')

    print('\n통과')
