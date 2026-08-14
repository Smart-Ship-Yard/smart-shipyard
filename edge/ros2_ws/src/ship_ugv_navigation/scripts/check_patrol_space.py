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

# --center 로 준 좌표에서 이 거리 안에 있는 장애물을 "그것을 가리킨 것"으로 본다.
# 사람이 RViz 에서 눈대중으로 찍어도 되게 하려는 값. 중심 좌표 자체는 찍은
# 좌표가 아니라 그 장애물의 픽셀 무게중심을 쓰므로 정확도는 떨어지지 않는다.
SNAP_RADIUS = 1.0              # m

# keepout 마스크를 대상 외곽에서 얼마나 더 부풀릴지.
# 라이다에 안 잡히는 대상이라 실측 오차를 흡수할 여유를 둔다.
# 이 값이 크면 순찰 원이 좁아지므로 무작정 키우면 안 된다.
MASK_PAD = 0.05                # m

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


def emit_keepout_mask(path, m, bbox, map_name):
    """Nav2 KeepoutFilter 용 마스크(.pgm + .yaml)를 만든다.

    왜 필요한가:
      라이다 스캔 평면(지면 약 0.20 m)보다 낮은 대상은 코스트맵에 안 잡힌다.
      맵의 static_layer 는 글로벌 코스트맵에만 있어 컨트롤러 단에서는 여전히
      대상을 모른다. KeepoutFilter 는 센서와 무관하게 코스트맵에 영역을 직접
      박고 글로벌·로컬 양쪽에 붙일 수 있어 이 구멍을 메운다.

    마스크는 맵과 **같은 해상도·원점·크기**여야 정확히 겹친다.
    검은색(0) = 진입 금지, 흰색(FREE) = 제약 없음. mode: trinary 로 읽으면
    검은 픽셀이 lethal 로 들어간다.
    """
    w, h, res = m['w'], m['h'], m['res']
    r0, r1, c0, c1 = bbox
    pad = int(round(MASK_PAD / res))
    r0 = max(0, r0 - pad)
    r1 = min(h - 1, r1 + pad)
    c0 = max(0, c0 - pad)
    c1 = min(w - 1, c1 + pad)

    px = bytearray([FREE]) * (w * h)
    for r in range(r0, r1 + 1):
        base = r * w
        for c in range(c0, c1 + 1):
            px[base + c] = OCCUPIED

    pgm_path = os.path.splitext(path)[0] + '.pgm'
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(pgm_path, 'wb') as f:
        f.write(b'P5\n')
        f.write(f'# keepout mask for {map_name} '
                f'(check_patrol_space.py 자동 생성)\n'.encode())
        f.write(f'{w} {h}\n255\n'.encode())
        f.write(bytes(px))

    with open(path, 'w') as f:
        f.write('\n'.join([
            f'# keepout 마스크 — {map_name}',
            '#',
            '# ⚠️ 자동 생성 파일. 손으로 고치지 말 것.',
            '#    맵을 다시 만들면 finalize_map.py 가 이 파일도 다시 쓴다.',
            '#',
            '# 검은 사각형 = 로봇이 들어가면 안 되는 영역(순찰 대상이 놓인 자리).',
            '# 라이다에 안 잡히는 낮은 대상을 코스트맵에 알려주기 위한 것이다.',
            '# 해상도·원점은 맵과 반드시 같아야 한다.',
            '#',
            f'image: {os.path.basename(pgm_path)}',
            'mode: trinary',
            f'resolution: {res}',
            f'origin: [{m["ox"]:.3f}, {m["oy"]:.3f}, 0.0]',
            'negate: 0',
            'occupied_thresh: 0.65',
            'free_thresh: 0.196',
            '',
        ]))

    print(f'   🧱 keepout 마스크 생성: {path}')
    print(f'      금지 영역 {(c1 - c0 + 1) * res:.2f} x {(r1 - r0 + 1) * res:.2f} m '
          f'(대상 외곽 + 여유 {MASK_PAD} m)')


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
def rect_bbox(m, cx, cy, sx, sy, yaw):
    """map 좌표의 (중심, 크기, 회전각) 을 픽셀 bbox 로 바꾼다.

    회전된 사각형을 그대로 그리는 대신 그것을 감싸는 축정렬 사각형을 쓴다.
    조금 더 커지지만 안전한 쪽이고, 마스크는 대상보다 넉넉한 편이 낫다.
    """
    hx, hy = sx / 2.0, sy / 2.0
    c, s = math.cos(yaw), math.sin(yaw)
    xs, ys = [], []
    for dx, dy in ((hx, hy), (hx, -hy), (-hx, hy), (-hx, -hy)):
        xs.append(cx + dx * c - dy * s)
        ys.append(cy + dx * s + dy * c)
    res, h = m['res'], m['h']
    # 바깥으로 확장(floor/ceil)한다. round 를 쓰면 경계 셀이 잘려 대상의 끝부분이
    # 마스크 밖으로 삐져나올 수 있다. keepout 은 모자란 것보다 넘치는 편이 안전하다.
    cols = [(x - m['ox']) / res for x in xs]
    rows = [(h - 1) - (y - m['oy']) / res for y in ys]
    return (max(0, int(math.floor(min(rows)))), min(h - 1, int(math.ceil(max(rows)))),
            max(0, int(math.floor(min(cols)))), min(m['w'] - 1, int(math.ceil(max(cols)))))


def analyze_map(yaml_path, center=None, margin=DEFAULT_MARGIN, emit_patrol=None,
                emit_mask=None, mask_size=None, mask_yaw=0.0):
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

    # 순찰 대상의 픽셀 범위(행/열 min·max). keepout 마스크 사각형을 그릴 때 쓴다.
    # 중심 좌표만으로는 대상 크기를 알 수 없어 따로 들고 다닌다.
    #
    # 크기를 얻는 경로는 둘이다.
    #   ① 맵의 섬에서        — 대상이 라이다에 잡히는 경우
    #   ② --mask-size 로 직접 — 라이다에 안 잡히는 낮은 대상(모형 배).
    #                          이때는 맵에 아무것도 없으므로 ①이 불가능하다.
    bbox = None

    islands = find_islands(m)
    cands = []
    for comp in islands:
        rs = [a for a, b in comp]
        cs = [b for a, b in comp]
        cands.append({
            'cx': m['ox'] + (sum(cs) / len(cs)) * res,
            'cy': m['oy'] + ((h - 1) - (sum(rs) / len(rs))) * res,
            'sx': (max(cs) - min(cs) + 1) * res,
            'sy': (max(rs) - min(rs) + 1) * res,
            'bbox': (min(rs), max(rs), min(cs), max(cs)),
        })
    cands.sort(key=lambda c: c['sx'] * c['sy'], reverse=True)

    if center is None:
        if not cands:
            print('❌ 자유공간에 완전히 포위된 장애물이 없음.')
            print('   = 무언가의 주위를 한 바퀴 도는 경로 자체가 존재하지 않는다.')
            print('   매핑할 때 대상 주위를 완전히 한 바퀴 돌지 않았을 가능성이 높음.')
            print()
            print('   ※ 대상이 라이다 스캔 평면(지면 약 0.20 m)보다 낮으면 맵에 안 찍힌다.')
            print('     모형 배처럼 낮은 대상은 매핑하는 동안만 높은 상자를 놓을 것')
            print('     (재매핑_체크리스트.md 0단계). 좌표를 아는 경우에는')
            print('     --center <X> <Y> 로 직접 지정할 수도 있다.')
            return False

        pick = cands[0]
        cx, cy, bbox = pick['cx'], pick['cy'], pick['bbox']

        if len(cands) == 1:
            print('포위된 장애물 1개 발견. 이것을 순찰 대상으로 본다:')
            print(f'  중심 map ({cx:.2f}, {cy:.2f}), '
                  f'크기 약 {pick["sx"]:.2f} x {pick["sy"]:.2f} m')
        else:
            # 후보가 여럿이면 사람이 확인해야 한다. 크기로 고르는 것은
            # 기본값일 뿐, 가장 큰 것이 순찰 대상이라는 보장은 없다.
            print('=' * 68)
            print(f'⚠️  포위된 장애물이 {len(cands)}개다 — 자동 선택이 맞는지 확인할 것')
            print('=' * 68)
            for i, c in enumerate(cands):
                mark = '   <-- 선택됨 (가장 큼)' if i == 0 else ''
                print(f'  [{i + 1}] 중심 ({c["cx"]:7.2f}, {c["cy"]:7.2f})  '
                      f'크기 {c["sx"]:.2f} x {c["sy"]:.2f} m{mark}')
            print()
            print('  의도한 대상이 아니면 위 목록의 중심 좌표를 그대로 복사해 다시 돌린다:')
            print(f'    python3 scripts/finalize_map.py <맵이름> '
                  f'--center {cands[-1]["cx"]:.2f} {cands[-1]["cy"]:.2f}')
            print('  정확히 맞출 필요 없다 — 가장 가까운 것을 지목한 것으로 보고')
            print('  그 장애물의 무게중심을 다시 계산해서 쓴다.')
            print('=' * 68)
    else:
        cx, cy = center
        print(f'지정된 좌표: map ({cx:.2f}, {cy:.2f})')

        if mask_size:
            # 대상 크기를 직접 받은 경우. 라이다에 안 잡히는 대상(모형 배)은
            # 맵에 섬이 없으므로 이 경로로만 마스크를 만들 수 있다.
            bbox = rect_bbox(m, cx, cy, mask_size[0], mask_size[1], mask_yaw)
            print(f'  대상 크기 {mask_size[0]:.2f} x {mask_size[1]:.2f} m '
                  f'(yaw {math.degrees(mask_yaw):.0f}도) 를 함께 받았다 — 마스크에 사용')
            print('  좌표도 지정값을 그대로 쓴다 (맵의 장애물로 스냅하지 않음)')

        # 크기를 못 받았으면 "어느 물체냐"를 고르는 용도로만 좌표를 쓴다.
        # 중심 좌표 자체는 그 물체의 픽셀 무게중심으로 바꾼다 — 사람이 눈대중으로
        # 찍은 점보다 훨씬 정확하다. 사람은 지목만 하고 계산은 기계가 한다.
        elif cands:
            near = min(cands, key=lambda c: math.hypot(c['cx'] - cx, c['cy'] - cy))
            d = math.hypot(near['cx'] - cx, near['cy'] - cy)
            if d <= SNAP_RADIUS:
                cx, cy, bbox = near['cx'], near['cy'], near['bbox']
                print(f'  → {d:.2f} m 거리의 장애물'
                      f'({near["sx"]:.2f} x {near["sy"]:.2f} m)을 지목한 것으로 본다')
                print(f'  순찰 중심은 그 장애물의 무게중심 ({cx:.3f}, {cy:.3f}) 을 쓴다')
            else:
                print(f'  ⚠️ {SNAP_RADIUS} m 안에 포위된 장애물이 없다 '
                      f'(가장 가까운 것 {d:.2f} m).')
                print(f'     지정한 좌표를 그대로 순찰 중심으로 쓰고, '
                      f'마스크는 만들지 않는다')
        else:
            print('  ⚠️ 맵에 포위된 장애물이 없다.')
            print('     지정한 좌표를 그대로 순찰 중심으로 쓰고, 마스크는 만들지 않는다')
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
            if emit_mask:
                if bbox is not None:
                    emit_keepout_mask(emit_mask, m, bbox, map_name)
                    # 마스크는 대상 외곽에 MASK_PAD 만큼 더 부풀린다. 좁은 맵에서는
                    # 그 여유가 순찰 여유를 거의 다 먹어 경로가 금지영역에 닿는다.
                    # 조용히 두면 "왜 갑자기 경로가 안 나오지" 로 헤매게 된다.
                    if MASK_PAD >= best[1] - 0.03:
                        print()
                        print('   ' + '=' * 62)
                        print(f'   ⚠️ 마스크 여유({MASK_PAD} m)가 순찰 여유'
                              f'({best[1]:.3f} m)를 거의 다 먹는다')
                        print(f'      남는 폭 {best[1] - MASK_PAD:.3f} m — '
                              f'순찰 경로가 금지영역에 닿을 수 있다.')
                        print('      대상이 라이다에 잡히는 물체라면(코스트맵이 이미 안다)')
                        print('      마스크가 필요 없으므로 --no-mask 로 끄는 편이 낫다.')
                        print('      배처럼 라이다에 안 잡히는 대상이면 주변을 더 치우고')
                        print('      재매핑해 반지름을 키울 것.')
                        print('   ' + '=' * 62)
                else:
                    # 조용히 넘어가면 나중에 "왜 keepout 이 안 먹지" 로 헤맨다.
                    print('   ⚠️ keepout 마스크를 만들지 못했다 — 대상의 크기를 '
                          '알 수 없다.')
                    print('      맵에 포위된 장애물이 없거나, --center 좌표가 '
                          f'어느 장애물과도 {SNAP_RADIUS} m 안에서 만나지 않는다.')
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
    ap.add_argument('--emit-mask', metavar='PATH',
                    help='Nav2 KeepoutFilter 용 마스크(.pgm + .yaml)를 만든다. '
                         '--emit-patrol 과 함께 쓴다. 라이다에 안 잡히는 낮은 '
                         '대상을 코스트맵에 알려주기 위한 것')
    ap.add_argument('--mask-size', nargs=2, type=float, metavar=('W', 'H'),
                    help='마스크에 그릴 대상 크기(m). 대상이 라이다에 안 잡혀 '
                         '맵에서 크기를 알 수 없을 때 쓴다. --center 와 함께 준다')
    ap.add_argument('--mask-yaw', type=float, default=0.0, metavar='RAD',
                    help='대상 방향각(라디안). --mask-size 와 함께 쓴다. '
                         '회전 사각형을 감싸는 축정렬 사각형으로 그린다')
    a = ap.parse_args()

    if a.obstacle:
        required_space(a.obstacle[0], a.obstacle[1], a.margin)
        return 0
    if a.map:
        # 종료 코드: 0 = 여유 충분, 2 = 완주되나 여유 부족(사용 가능), 1 = 순찰 불가
        if a.mask_size and not a.center:
            print('❌ --mask-size 는 --center 와 함께 줘야 한다 '
                  '(어디에 그릴지 알 수 없음)')
            return 3
        return analyze_map(a.map, a.center, a.margin, a.emit_patrol, a.emit_mask,
                           a.mask_size, a.mask_yaw)
    ap.print_help()
    return 3


if __name__ == '__main__':
    sys.exit(main())
