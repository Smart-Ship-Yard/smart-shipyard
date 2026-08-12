#!/usr/bin/env python3
"""
check_patrol_space.py
======================
"이 맵에서 원형 순찰이 가능한가"를 로봇 실측 footprint로 검사한다.
재매핑할 때마다 시연 투입 전에 이걸 먼저 돌릴 것.

왜 필요한가
-----------
원형 경로를 도는 로봇은 자기 몸통 크기보다 훨씬 넓은 띠를 쓸고 지나간다.
base_link가 뒷바퀴 축이라 차체가 앞으로 0.332m 튀어나와 있기 때문이다.
반지름 R 원을 돌면 실제 점유 범위는

    안쪽 = R - 0.089
    바깥 = sqrt((R + 0.089)^2 + 0.332^2)      <-- R보다 훨씬 큼

예) R=0.40m로 돌면 중심에서 0.31~0.59m 구간을 전부 점유한다(폭 0.28m).
"반지름 0.4m니까 0.4m만 있으면 되겠지"라고 생각하면 반드시 박는다.

사용법
------
  # (A) 물체 크기만으로 "얼마나 치워야 하나" 계산
  python3 check_patrol_space.py --obstacle 0.127 0.127        # 폼롤러(세움)
  python3 check_patrol_space.py --obstacle 0.35 0.40          # 레고 배

  # (B) 실제 맵 파일로 순찰 가능 여부 검사
  python3 check_patrol_space.py --map maps/demo_room.yaml
  python3 check_patrol_space.py --map maps/my_map.yaml --center 2.81 1.14
"""

import argparse
import math
import os
import sys
from collections import deque

# ======================================================================
# 로봇 실측값 (ship_ugv_core.urdf.xacro 기준, base_link = 뒷바퀴 축·지면)
#   전방  +0.332 = chassis_offset_x(0.1315) + chassis_length/2(0.2005)
#   후방  -0.069 = wheel_axle_to_rear_face
#   좌우  ±0.089 = chassis_width(0.178)/2
# nav2_params.yaml 의 footprint 와 반드시 같은 값을 쓸 것.
# ======================================================================
FRONT, REAR, HALF_W = 0.332, -0.069, 0.089
DEFAULT_MARGIN = 0.10          # 안팎 최소 안전여유 (m)

FREE, OCCUPIED, UNKNOWN = 254, 0, 205


def footprint_points(n=8):
    """차체 사각형을 격자로 샘플링한 점들 (base_link 기준)."""
    return [(REAR + (FRONT - REAR) * i / n, -HALF_W + 2 * HALF_W * j / n)
            for i in range(n + 1) for j in range(n + 1)]


def sweep_bounds(r):
    """반지름 r 원을 돌 때 차체가 점유하는 (안쪽, 바깥쪽) 반경."""
    pts = footprint_points()
    ds = [math.hypot(r - py, px) for px, py in pts]
    return min(ds), max(ds)


# ======================================================================
# (A) 물체 크기 -> 필요한 공터 크기
# ======================================================================
def required_space(obs_x, obs_y, margin=DEFAULT_MARGIN):
    reach = math.hypot(obs_x / 2.0, obs_y / 2.0)     # 물체 중심에서 가장 먼 모서리
    r_min = reach + HALF_W + margin                  # 안쪽이 물체에 닿지 않을 최소 반지름
    _, outer = sweep_bounds(r_min)
    need = outer + margin                            # 물체 중심에서 벽까지 필요한 거리

    print(f'중앙 물체: {obs_x:.3f} x {obs_y:.3f} m  (중심에서 모서리까지 {reach:.3f} m)')
    print(f'안전여유: 안팎 각 {margin:.2f} m')
    print()
    print(f'  권장 순찰 반지름 : {r_min:.2f} m')
    print(f'  차체 점유 범위   : {sweep_bounds(r_min)[0]:.3f} ~ {outer:.3f} m (물체 중심 기준)')
    print()
    print(f'  >>> 물체 중심에서 사방 {need:.2f} m 이상을 비워야 함')
    print(f'  >>> 즉 {need * 2:.2f} x {need * 2:.2f} m 의 공터가 필요 '
          f'(바닥에 테이프로 표시해두면 편함)')
    return r_min, need


# ======================================================================
# 맵 로딩
# ======================================================================
def load_map(yaml_path):
    meta = {}
    with open(yaml_path) as f:
        for line in f:
            line = line.split('#')[0].strip()
            if ':' not in line:
                continue
            k, v = line.split(':', 1)
            meta[k.strip()] = v.strip()

    res = float(meta['resolution'])
    origin = [float(x) for x in meta['origin'].strip('[]').split(',')]
    pgm_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), meta['image'])

    d = open(pgm_path, 'rb').read()
    parts, i = [], 0
    while len(parts) < 4:                     # P5 / width / height / maxval
        while d[i:i + 1].isspace():
            i += 1
        if d[i:i + 1] == b'#':
            while d[i:i + 1] != b'\n':
                i += 1
            continue
        s = i
        while not d[i:i + 1].isspace():
            i += 1
        parts.append(d[s:i])
    i += 1
    w, h = int(parts[1]), int(parts[2])
    px = d[i:i + w * h]

    free = [[px[r * w + c] == FREE for c in range(w)] for r in range(h)]
    return {'w': w, 'h': h, 'res': res, 'ox': origin[0], 'oy': origin[1],
            'free': free, 'yaml': meta}


def is_free(m, x, y):
    r = int(round((m['h'] - 1) - (y - m['oy']) / m['res']))
    c = int(round((x - m['ox']) / m['res']))
    return 0 <= r < m['h'] and 0 <= c < m['w'] and m['free'][r][c]


def find_islands(m):
    """자유공간에 완전히 포위된 장애물 덩어리 = 그 주위를 돌 수 있는 대상."""
    w, h, free = m['w'], m['h'], m['free']
    seen = [[False] * w for _ in range(h)]
    dq = deque()
    border = [(r, c) for r in range(h) for c in (0, w - 1)] + \
             [(r, c) for c in range(w) for r in (0, h - 1)]
    for r, c in border:
        if not free[r][c] and not seen[r][c]:
            seen[r][c] = True
            dq.append((r, c))
    while dq:
        r, c = dq.popleft()
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nr, nc = r + dr, c + dc
            if 0 <= nr < h and 0 <= nc < w and not seen[nr][nc] and not free[nr][nc]:
                seen[nr][nc] = True
                dq.append((nr, nc))

    islands, vis = [], [[False] * w for _ in range(h)]
    for r in range(h):
        for c in range(w):
            if not free[r][c] and not seen[r][c] and not vis[r][c]:
                comp, q = [], deque([(r, c)])
                vis[r][c] = True
                while q:
                    a, b = q.popleft()
                    comp.append((a, b))
                    for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                        na, nb = a + dr, b + dc
                        if (0 <= na < h and 0 <= nb < w and not vis[na][nb]
                                and not free[na][nb] and not seen[na][nb]):
                            vis[na][nb] = True
                            q.append((na, nb))
                if len(comp) >= 2:
                    islands.append(comp)
    return islands


def clearance_at(m, x, y, cap=0.8):
    """(x,y)에서 가장 가까운 비자유 셀까지의 거리 (근사)."""
    if not is_free(m, x, y):
        return 0.0
    d = 0.0
    step = m['res'] / 2
    while d < cap:
        d += step
        if any(not is_free(m, x + d * math.cos(math.radians(t)),
                           y + d * math.sin(math.radians(t)))
               for t in range(0, 360, 10)):
            return d - step
    return cap


def check_circle(m, cx, cy, r, step_deg=2):
    """반지름 r 원을 실제 footprint로 훑어 충돌 각도 수와 최소 여유를 반환."""
    pts = footprint_points()
    bad, worst = 0, 99.0
    for a in range(0, 360, step_deg):
        th = math.radians(a)
        bx, by = cx + r * math.cos(th), cy + r * math.sin(th)
        hd = th + math.pi / 2                      # 원의 접선 방향
        ch, sh = math.cos(hd), math.sin(hd)
        hit = False
        for px, py in pts:
            X, Y = bx + px * ch - py * sh, by + px * sh + py * ch
            if not is_free(m, X, Y):
                hit = True
                break
            worst = min(worst, clearance_at(m, X, Y, cap=0.5))
        if hit:
            bad += 1
    return bad, (0.0 if worst > 90 else worst)


# ======================================================================
#  웨이포인트 개수 계산
# ======================================================================
#  nav2_params.yaml 의 값과 맞물려 있다. 그쪽을 바꾸면 여기도 바꿔야 한다.
XY_GOAL_TOLERANCE = 0.15    # controller_server > general_goal_checker
ROTATE_MIN_ANGLE_DEG = 40.0  # FollowPath > rotate_to_heading_min_angle (0.7 rad)
SPACING_FACTOR = 1.5         # 지점 간격이 도달반경의 몇 배 이상이어야 하는가
PREFERRED_N = 12


def pick_num_waypoints(radius):
    """순찰 반지름에 맞는 웨이포인트 개수를 고른다.

    제약 두 가지
      ① 방향 변화 360/N 이 rotate_to_heading_min_angle(40도) 미만이어야 한다.
         넘으면 지점마다 제자리 회전이 필요해진다. 우리 로봇은 회전에
         옆으로 0.259 m 를 더 요구하므로 좁은 곳에서는 **물리적으로 불가능**하다.
      ② 지점 간격 2R·sin(pi/N) 이 도달 판정 반경의 1.5배 이상이어야 한다.
         간격이 판정 반경만 하면 한 지점에 서 있는 채로 다음 지점도
         "도달"로 처리되어 순찰이 제자리에서 헛돈다.

    둘 다 만족하는 N 이 없으면 **①을 우선**한다.
      ① 위반 = 회전이 필요해짐   -> 좁은 곳에서 아예 못 감 (치명적)
      ② 위반 = 원 궤적이 헐거워짐 -> 품질 저하일 뿐 (감수 가능)

    반환: (N, 경고문자열 또는 None)
    """
    n_min = math.ceil(360.0 / ROTATE_MIN_ANGLE_DEG) + 1      # 40도 "미만"이라 +1
    def spacing(n):
        return 2.0 * radius * math.sin(math.pi / n)

    both = [n for n in range(n_min, 25)
            if spacing(n) >= XY_GOAL_TOLERANCE * SPACING_FACTOR]
    if both:
        n = PREFERRED_N if PREFERRED_N in both else max(both)
        return n, None

    # ① 만 만족하는 것 중 간격이 가장 넓은 것 = n_min
    n = n_min
    ratio = spacing(n) / XY_GOAL_TOLERANCE
    warn = (f'반지름 {radius:.2f} m 가 작아 두 제약을 동시에 만족할 수 없다. '
            f'회전을 피하는 쪽(N={n}, 방향변화 {360.0/n:.0f}도)을 택했고 '
            f'지점 간격은 {spacing(n):.3f} m 로 도달반경의 {ratio:.2f}배뿐이다. '
            f'순찰은 돌지만 궤적이 헐거워진다. 대상 주변을 더 치우고 재매핑해 '
            f'반지름을 0.45 m 이상으로 키우는 것이 근본 해결이다.')
    return n, warn


def emit_patrol_yaml(path, map_name, cx, cy, radius, margin_val, tight):
    """patrol_<맵이름>.yaml 을 만든다. 손으로 옮겨 적는 단계를 없애기 위함."""
    n, warn = pick_num_waypoints(radius)
    spacing = 2.0 * radius * math.sin(math.pi / n)
    lines = [
        '# ' + '=' * 74,
        f'#  patrol_{map_name}.yaml — {map_name} 맵의 순찰 원',
        '# ' + '=' * 74,
        '#  ⚠️ 이 파일은 check_patrol_space.py 가 자동으로 만든다. 손으로 고치지 말 것.',
        '#     맵을 다시 만들면 finalize_map.py 가 이 파일도 다시 쓴다.',
        '#',
        '#  navigation.launch.py 가 map 인자를 보고 patrol_<맵이름>.yaml 을 찾아',
        '#  자동으로 로드한다. 따로 지정할 일이 없다.',
        '#',
        f'#      ros2 launch ship_ugv_navigation navigation.launch.py \\',
        f'#          map:={map_name} patrol:=true ...',
        '#',
        '#  ── 이 값들이 나온 근거 ────────────────────────────────────────────',
        '#  중심·반지름: 실제 footprint 를 원 위에서 한 바퀴 쓸어보며 통과 가능한',
        '#               반지름을 찾은 결과. 추측이 아니라 맵 픽셀 검사 결과다.',
        f'#      최소 여유 {margin_val:.3f} m',
        '#',
        '#  웨이포인트 개수: 아래 두 제약으로 계산',
        f'#      ① 방향 변화 360/N < {ROTATE_MIN_ANGLE_DEG:.0f}도  '
        f'(넘으면 지점마다 제자리 회전이 필요)',
        f'#      ② 지점 간격 >= 도달반경 {XY_GOAL_TOLERANCE} m 의 {SPACING_FACTOR}배',
        f'#      -> N={n}: 방향 변화 {360.0/n:.0f}도, 지점 간격 {spacing:.3f} m '
        f'(도달반경의 {spacing/XY_GOAL_TOLERANCE:.2f}배)',
        '#',
    ]
    if warn:
        lines += ['#  ⚠️ 경고'] + ['#     ' + s for s in _wrap(warn, 70)] + ['#']
    if tight:
        lines += [
            '#  🟡 이 맵은 여유가 0.1 m 미만이다.',
            '#     사람이 막아서면 우회할 공간이 없어 대기(BLOCKED)로 들어간다.',
            '#     그것이 올바른 동작이다 — 비집고 가는 편이 위험하다.',
            '#     space:=narrow 를 반드시 함께 준다 (inflation 0.25 로는 경로가 안 나온다).',
            '#',
        ]
    lines += [
        '# ' + '=' * 74,
        '',
        'patrol_mission_node:',
        '  ros__parameters:',
        f'    center_x: {cx:.3f}',
        f'    center_y: {cy:.3f}',
        f'    radius: {radius:.2f}',
        f'    num_waypoints: {n}',
        '',
        '    # 시계방향 = 로봇 오른쪽 90도에 달린 카메라가 중앙 대상을 향한다',
        '    direction: cw',
        '',
    ]
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'w') as f:
        f.write('\n'.join(lines))
    print(f'   📝 순찰 설정 생성: {path}')
    print(f'      center ({cx:.3f}, {cy:.3f})  radius {radius:.2f}  '
          f'num_waypoints {n}')
    if warn:
        print(f'      ⚠️ {warn}')
    return n


def _wrap(text, width):
    out, cur = [], ''
    for word in text.split():
        if len(cur) + len(word) + 1 > width:
            out.append(cur); cur = word
        else:
            cur = (cur + ' ' + word).strip()
    if cur:
        out.append(cur)
    return out


# ======================================================================
def analyze_map(yaml_path, center=None, margin=DEFAULT_MARGIN, emit_patrol=None):
    m = load_map(yaml_path)
    res, w, h = m['res'], m['w'], m['h']
    n_free = sum(sum(row) for row in m['free'])

    print(f'맵: {yaml_path}')
    print(f'  크기 {w}x{h}셀 = {w * res:.2f} x {h * res:.2f} m')
    print(f'  origin [{m["ox"]:.3f}, {m["oy"]:.3f}]  resolution {res}')
    print(f'  탐사된 자유공간 {n_free}셀 = {n_free * res * res:.2f} m^2 '
          f'({100 * n_free / (w * h):.0f}%)')

    ft = float(m['yaml'].get('free_thresh', 0.25))
    if ft > 0.2:
        print(f'  ⚠️ free_thresh={ft} -> 미탐사 픽셀(205)이 자유공간으로 분류됨!')
        print(f'     0.196 으로 고칠 것. 안 그러면 Nav2가 매핑 안 된 곳으로 경로를 뽑는다.')
    print()

    if center is None:
        islands = find_islands(m)
        if not islands:
            print('❌ 자유공간에 완전히 포위된 장애물이 없음.')
            print('   = 무언가의 주위를 한 바퀴 도는 경로 자체가 존재하지 않는다.')
            print('   매핑할 때 대상 주위를 완전히 한 바퀴 돌지 않았을 가능성이 높음.')
            return False
        islands.sort(key=len, reverse=True)
        comp = islands[0]
        rs = [a for a, b in comp]
        cs = [b for a, b in comp]
        cx = m['ox'] + (sum(cs) / len(cs)) * res
        cy = m['oy'] + ((h - 1) - (sum(rs) / len(rs))) * res
        size_x, size_y = (max(cs) - min(cs) + 1) * res, (max(rs) - min(rs) + 1) * res
        print(f'포위된 장애물 {len(islands)}개 발견. 가장 큰 것을 순찰 대상으로 봄:')
        print(f'  중심 map ({cx:.2f}, {cy:.2f}), 크기 약 {size_x:.2f} x {size_y:.2f} m')
    else:
        cx, cy = center
        print(f'지정된 순찰 중심: map ({cx:.2f}, {cy:.2f})')
    print()

    print(' 반지름 | 완주 | 최소여유 | 판정')
    print('--------+------+----------+---------------------------')
    good = []       # 완주 + 여유 충분
    passable = []   # 완주는 되나 여유 부족
    r = 0.20
    while r <= 1.60:
        bad, worst = check_circle(m, cx, cy, r)
        if bad == 0:
            passable.append((r, worst))
            verdict = '★ 사용 가능' if worst >= margin else f'통과하나 여유 부족(<{margin}m)'
            if worst >= margin:
                good.append((r, worst))
            print(f'  {r:.2f}m |  OK  |  {worst:.3f}m | {verdict}')
        else:
            print(f'  {r:.2f}m |  --  |  {worst:.3f}m | {bad}개 각도에서 충돌')
        r += 0.05

    def report(cands, radius_label, tight):
        best = max(cands, key=lambda t: t[1])
        lo, hi = cands[0][0], cands[-1][0]
        print(f'   {radius_label} {lo:.2f} ~ {hi:.2f} m')
        print(f'   권장: {best[0]:.2f} m (여유 {best[1]:.3f} m)')
        print()
        if emit_patrol:
            map_name = os.path.splitext(os.path.basename(yaml_path))[0]
            emit_patrol_yaml(emit_patrol, map_name, cx, cy, best[0], best[1], tight)
        else:
            n, warn = pick_num_waypoints(best[0])
            print('   patrol_mission_node 파라미터에 넣을 값:')
            print(f'     center_x: {cx:.3f}')
            print(f'     center_y: {cy:.3f}')
            print(f'     radius:   {best[0]:.2f}')
            print(f'     num_waypoints: {n}')
            if warn:
                print(f'   ⚠️ {warn}')
            print()
            print('   (--emit-patrol <경로> 를 주면 이 값으로 파일을 만들어 준다.')
            print('    finalize_map.py 를 쓰면 자동으로 붙는다)')

    print()
    if good:
        print('✅ 순찰 가능 (여유 충분)')
        report(good, '반지름', tight=False)
        return 0

    if passable:
        print(f'🟡 순찰 가능하지만 여유가 {margin} m 미만이다 — 사용은 가능')
        report(passable, '완주 가능 반지름', tight=True)
        print()
        print('   좁아도 되는 이유: Nav2 로컬 코스트맵이 라이다로 실시간 회피하므로')
        print('   UWB 위치 오차와 무관하게 벽에 부딪히지 않는다.')
        print('   단 inflation_radius를 0.10 m 수준으로 낮출 것.')
        print()
        print('   더 넉넉하게 하려면 대상 주변을 사방으로 5~10 cm씩 더 치우고 재매핑.')
        return 2

    print('❌ 완주 가능한 반지름이 없다 — 이 맵으로는 순찰 불가')
    print('   대상 주변을 더 치우고 재매핑할 것. 필요한 공터 크기는:')
    print()
    required_space(0.127, 0.127, margin)
    return 1


def main():
    ap = argparse.ArgumentParser(description='원형 순찰 가능 여부 검사')
    ap.add_argument('--map', help='맵 yaml 경로')
    ap.add_argument('--center', nargs=2, type=float, metavar=('X', 'Y'),
                    help='순찰 중심 map 좌표 (생략 시 자동 탐지)')
    ap.add_argument('--obstacle', nargs=2, type=float, metavar=('X', 'Y'),
                    help='중앙 물체 크기(m). 맵 없이 필요 공터만 계산')
    ap.add_argument('--margin', type=float, default=DEFAULT_MARGIN,
                    help=f'안전여유 m (기본 {DEFAULT_MARGIN})')
    ap.add_argument('--emit-patrol', metavar='PATH',
                    help='순찰 가능하면 이 경로에 patrol_<맵이름>.yaml 을 만든다. '
                         '순찰 불가 맵이면 만들지 않는다')
    a = ap.parse_args()

    if a.obstacle:
        required_space(a.obstacle[0], a.obstacle[1], a.margin)
        return 0
    if a.map:
        # 종료 코드: 0 = 여유 충분, 2 = 완주되나 여유 부족(사용 가능), 1 = 순찰 불가
        return analyze_map(a.map, a.center, a.margin, a.emit_patrol)
    ap.print_help()
    return 3


if __name__ == '__main__':
    sys.exit(main())
