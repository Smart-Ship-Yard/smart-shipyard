#!/usr/bin/env python3
"""bake_map_origin.rotate_pixels 가 월드 좌표를 보존하는지 확인한다.

회전을 그림에 구우면 origin 의 yaw 가 0 이 된다. 그래도 **같은 벽이 같은
세계 좌표에 있어야** 한다. 이게 깨지면 맵 전체가 밀린다 — 눈으로는 잘 안
보이고 Nav2 가 벽을 뚫거나 허공을 막는 식으로만 드러난다. 그래서 남겨둔다.

    python3 scripts/test_bake_rotation.py
"""
import math
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bake_map_origin import rotate_pixels, write_pgm, UNKNOWN_PX

RES = 0.05
OCCUPIED = 0


def world_of(col, row_from_bottom, ox, oy, yaw, res=RES):
    """맵 yaml 규약: 세계좌표 = R(yaw) * (열*res, 아래에서센행*res) + origin."""
    x, y = col * res, row_from_bottom * res
    c, s = math.cos(yaw), math.sin(yaw)
    return (ox + c * x - s * y, oy + s * x + c * y)


def check(yaw_deg, w=13, h=7, mark=(9, 2), ox=-1.3, oy=0.4):
    yaw = math.radians(yaw_deg)
    px = bytearray([UNKNOWN_PX]) * (w * h)
    px[(h - 1 - mark[1]) * w + mark[0]] = OCCUPIED

    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, 'm.pgm')
        write_pgm(p, w, h, px, 'test')
        nw, nh, npx, da, db = rotate_pixels(p, yaw)

    found = [(u, nh - 1 - v) for v in range(nh) for u in range(nw)
             if npx[v * nw + u] == OCCUPIED]
    assert len(found) == 1, f'{yaw_deg}도: 칠한 칸이 {len(found)}개 (1개여야 함)'

    before = world_of(*mark, ox, oy, yaw)
    after = world_of(found[0][0], found[0][1],
                     ox + da * RES, oy + db * RES, 0.0)
    err = math.dist(before, after)
    assert err <= RES * 0.75, (
        f'{yaw_deg}도: 세계좌표가 {err * 100:.1f} cm 어긋났다 '
        f'(허용 {RES * 75:.1f} cm) — {before} vs {after}')
    return err


if __name__ == '__main__':
    worst = 0.0
    for deg in (-8.36, -3.5, 0.0, 12.0, 45.0, -90.0, 179.0):
        e = check(deg)
        worst = max(worst, e)
        print(f'  {deg:>7.2f}도  어긋남 {e * 100:5.2f} cm  OK')
    print(f'\n최대 {worst * 100:.2f} cm — 격자 반 칸({RES * 50:.1f} cm) 수준. 통과')
