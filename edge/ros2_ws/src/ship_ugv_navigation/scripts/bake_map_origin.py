#!/usr/bin/env python3
"""
bake_map_origin.py
===================
`map_saver_cli`로 저장한 맵을 `map`(UWB 절대) 좌표계로 보정한다.
맵을 새로 뜰 때마다 한 번씩 실행하면 된다.

무엇을 고치나
-------------
slam_toolbox는 이 프로젝트 규칙상 map_frame을 `slam_map`으로 쓴다.
`map_saver_cli`는 TF를 적용하지 않고 `/map` 토픽 값을 그대로 파일에 쓰므로
저장된 맵은 `slam_map` 좌표계다. 반면 Nav2와 EKF는 `map` 좌표계로 동작한다.
그 차이(= `slam_map_alignment`가 계산한 map→slam_map 변환)를 yaml의
`origin` 세 숫자에 합성해 넣는다.

`origin`의 세 번째 값이 yaw이므로 **이미지를 다시 그릴 필요가 없다.**
회전까지 origin 한 줄로 표현된다.

    new_x   = tx + cos(θ)·ox − sin(θ)·oy
    new_y   = ty + sin(θ)·ox + cos(θ)·oy
    new_yaw = θ + oyaw

이 공식은 장소·UWB 배치와 무관하게 항상 같다. 매번 바뀌는 것은 θ, tx, ty
세 숫자뿐이고 그 값은 `align_*.json`에 자동 저장되어 있다.

같이 하는 일
------------
- `free_thresh`를 0.196으로 교정 (map_saver_cli는 매번 0.25로 쓴다.
  0.25면 미탐사 픽셀 205가 자유공간으로 분류되어 Nav2가 매핑 안 된
  영역으로 경로를 뽑는다)
- 원본을 `.yaml.orig` 로 백업
- 이미 보정된 파일은 다시 보정하지 않음 (중복 적용 방지)

사용법
------
  # align 결과 json에서 자동으로 읽기 (권장)
  python3 bake_map_origin.py maps/shipyard_map_<장소>_v<버전번호>.yaml \
      --align maps/calibration_records/align_001.json

  # tf2_echo map slam_map 값을 직접 넣기
  python3 bake_map_origin.py maps/shipyard_map_<장소>_v<버전번호>.yaml \
      --tf 0.9446 0.1026 2.4119

  # 계산만 해보고 파일은 안 고치기
  python3 bake_map_origin.py maps/x.yaml --align a.json --dry-run
"""

import argparse
import json
import math
import os
import shutil
import sys

from check_patrol_space import read_pgm

UNKNOWN_PX = 205        # 표준 맵 PGM 의 '미탐사' 값


def rotate_pixels(pgm_path, yaw):
    """맵 그림 자체를 yaw 만큼 돌린다. (새 폭, 새 높이, 픽셀, da, db) 를 준다.

    ★ 왜 필요한가 (2026-08-20 실측으로 드러난 문제) ★
    -----------------------------------------------------
    map yaml 의 origin 세 번째 값(yaw)을 **Nav2 코스트맵은 읽지 않는다.**
    nav2_costmap_2d 의 Costmap2D 에는 origin_x, origin_y 만 있고 회전 필드가
    아예 없어서, StaticLayer 가 맵을 받을 때 yaw 를 그냥 버린다.

    실측 (shipyard_map_hall_v3):
        /map                     origin yaw = -8.36도
        /global_costmap/costmap  origin yaw =  0도      <- 버려졌다
        /keepout_mask            origin yaw =  0도      <- 마찬가지

    RViz 의 Map 디스플레이만 yaw 를 제대로 그리기 때문에, 화면에서는 회색 맵과
    분홍 코스트맵이 8.36도 어긋난 채 겹쳐 보인다. 원점에서 7 m 떨어진 벽이면
    7 * 0.146 = 1.0 m 밀린다. Nav2 는 그 밀린 코스트맵으로 경로를 짠다.

    그래서 회전을 origin 이 아니라 **그림에 굽는다.** 굽고 나면 origin yaw 가
    0 이 되어 코스트맵·마스크·RViz 가 모두 같은 것을 본다.

    최근접 표본추출이라 5 cm 격자에서 반 칸 이하의 오차가 생긴다. 정합 자체의
    오차(inlier 99%에서 수 cm)보다 작으므로 무시할 수 있다.
    """
    w, h, px = read_pgm(pgm_path)
    c, s = math.cos(yaw), math.sin(yaw)

    # 픽셀 좌표를 아래에서 센다: a = 열, b = (아래에서 센 행). 맵 원점이 (0,0).
    rot = [(c * a - s * b, s * a + c * b)
           for a in (0, w - 1) for b in (0, h - 1)]
    a_min, a_max = min(p[0] for p in rot), max(p[0] for p in rot)
    b_min, b_max = min(p[1] for p in rot), max(p[1] for p in rot)
    nw = int(round(a_max - a_min)) + 1
    nh = int(round(b_max - b_min)) + 1

    out = bytearray([UNKNOWN_PX]) * (nw * nh)
    for v in range(nh):
        b_rot = (nh - 1 - v) + b_min
        for u in range(nw):
            a_rot = u + a_min
            sa = int(round(c * a_rot + s * b_rot))     # 역회전해서 원본을 찾는다
            sb = int(round(-s * a_rot + c * b_rot))
            if 0 <= sa < w and 0 <= sb < h:
                out[v * nw + u] = px[(h - 1 - sb) * w + sa]
    return nw, nh, out, a_min, b_min


def w_h_txt(pgm_path):
    w, h, _ = read_pgm(pgm_path)
    return f'{w}x{h}'


def write_pgm(path, w, h, px, comment):
    with open(path, 'wb') as f:
        f.write(b'P5\n')
        f.write(f'# {comment}\n'.encode())
        f.write(f'{w} {h}\n255\n'.encode())
        f.write(bytes(px))

FREE_THRESH_CORRECT = 0.196
BAKED_MARKER = '# baked: map<-slam_map applied'


def read_yaml(path):
    """맵 yaml의 단순 key: value를 읽는다 (전체 파싱 아님, 줄은 보존)."""
    lines = open(path).read().splitlines()
    meta = {}
    for i, line in enumerate(lines):
        body = line.split('#')[0].strip()
        if ':' not in body:
            continue
        k, v = body.split(':', 1)
        meta[k.strip()] = (v.strip(), i)
    return lines, meta


def parse_origin(text):
    vals = [float(x) for x in text.strip('[]').split(',')]
    if len(vals) != 3:
        raise ValueError(f'origin 값이 3개가 아님: {text}')
    return vals


def main():
    ap = argparse.ArgumentParser(
        description='저장된 맵의 origin을 map 좌표계로 보정')
    ap.add_argument('yaml_path', help='map_saver_cli가 만든 .yaml 경로')
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument('--align', metavar='JSON',
                     help='slam_map_alignment의 align_*.json 경로')
    src.add_argument('--tf', nargs=3, type=float, metavar=('TX', 'TY', 'YAW'),
                     help='tf2_echo map slam_map 의 x y yaw(radian)')
    ap.add_argument('--dry-run', action='store_true', help='계산만 하고 저장 안 함')
    ap.add_argument('--force', action='store_true', help='이미 보정된 파일도 다시 보정')
    a = ap.parse_args()

    if not os.path.exists(a.yaml_path):
        print(f'❌ 파일 없음: {a.yaml_path}')
        return 1

    # ---- 변환값 확보 ----
    if a.align:
        if not os.path.exists(a.align):
            print(f'❌ align json 없음: {a.align}')
            return 1
        j = json.load(open(a.align))
        try:
            tx, ty, th = j['tx'], j['ty'], j['theta_rad']
        except KeyError:
            print(f'❌ {a.align} 에 tx/ty/theta_rad 가 없음. align_*.json이 맞나?')
            return 1
        n = j.get('num_correspondence_points')
        ratio = j.get('inlier_ratio')
        print(f'정합 결과: {a.align}')
        if n is not None and ratio is not None:
            warn = '  ⚠️ 신뢰도 낮음 (0.4 미만) — 재매핑 권장' if ratio < 0.4 else ''
            print(f'  대응점 {n}개, inlier {ratio:.0%}{warn}')
    else:
        tx, ty, th = a.tf
        print('정합 결과: 직접 입력')

    print(f'  map<-slam_map:  tx={tx:.4f}  ty={ty:.4f}  yaw={th:.4f} rad '
          f'({math.degrees(th):.2f}°)')
    print()

    lines, meta = read_yaml(a.yaml_path)

    if BAKED_MARKER in '\n'.join(lines) and not a.force:
        print('⚠️ 이미 보정된 맵이다 (중복 적용 방지).')
        print('   다시 보정하려면 --force, 또는 .yaml.orig 를 되돌린 뒤 실행할 것.')
        return 1

    if 'origin' not in meta:
        print('❌ yaml에 origin 항목이 없음')
        return 1

    ox, oy, oyaw = parse_origin(meta['origin'][0])

    # ---- 핵심 계산 ----
    c, s = math.cos(th), math.sin(th)
    nx = tx + c * ox - s * oy
    ny = ty + s * ox + c * oy
    nyaw = th + oyaw

    print(f'origin (slam_map): [{ox:.4f}, {oy:.4f}, {oyaw:.4f}]')
    print(f'origin (map)     : [{nx:.4f}, {ny:.4f}, {nyaw:.4f}]')

    # ---- 회전을 그림에 굽는다 (rotate_pixels 주석 참고) ----
    # origin 의 yaw 는 Nav2 코스트맵이 읽지 않는다. 그대로 두면 코스트맵만
    # 회전 없이 놓여서 실제 벽과 어긋난다. 그림을 돌리고 yaw 는 0 으로 만든다.
    rotated = None
    if abs(nyaw) > 1e-6:
        pgm_path = os.path.join(os.path.dirname(os.path.abspath(a.yaml_path)),
                                meta['image'][0])
        res = float(meta['resolution'][0])
        nw, nh, npx, da, db = rotate_pixels(pgm_path, nyaw)
        nx += da * res
        ny += db * res
        rotated = (pgm_path, nw, nh, npx)
        print(f'그림 회전    {math.degrees(nyaw):+.2f}° 를 픽셀에 구웠다 '
              f'({w_h_txt(pgm_path)} -> {nw}x{nh})')
        print('             (Nav2 코스트맵은 origin 의 yaw 를 읽지 않는다)')
        nyaw = 0.0

    print(f'origin (map)     : [{nx:.4f}, {ny:.4f}, {nyaw:.4f}]   ← 적용')

    lines[meta['origin'][1]] = f'origin: [{nx:.4f}, {ny:.4f}, {nyaw:.4f}]'

    # ---- free_thresh 교정 ----
    if 'free_thresh' in meta:
        cur = float(meta['free_thresh'][0])
        if abs(cur - FREE_THRESH_CORRECT) > 1e-9:
            print(f'free_thresh: {cur} -> {FREE_THRESH_CORRECT}  '
                  f'(미탐사 픽셀이 자유공간으로 분류되는 것 방지)')
            lines[meta['free_thresh'][1]] = f'free_thresh: {FREE_THRESH_CORRECT}'
        else:
            print(f'free_thresh: {cur} (이미 정상)')
    else:
        print('⚠️ free_thresh 항목이 없음 — 수동 확인 필요')

    lines.append(BAKED_MARKER +
                 f' tx={tx:.4f} ty={ty:.4f} yaw={th:.4f}'
                 + (' (yaw 는 그림에 구움 — 코스트맵이 origin yaw 를 못 읽는다)'
                    if rotated else ''))

    if a.dry_run:
        print('\n--dry-run 이므로 파일을 고치지 않음.')
        return 0

    backup = a.yaml_path + '.orig'
    if not os.path.exists(backup):
        shutil.copy2(a.yaml_path, backup)
        print(f'\n원본 백업: {backup}')

    if rotated:
        pgm_path, nw, nh, npx = rotated
        pgm_backup = pgm_path + '.orig'
        if not os.path.exists(pgm_backup):
            shutil.copy2(pgm_path, pgm_backup)
            print(f'원본 백업: {pgm_backup}')
        write_pgm(pgm_path, nw, nh, npx,
                  f'{os.path.basename(pgm_path)} — bake_map_origin.py 가 '
                  f'{math.degrees(th + oyaw):.2f}° 회전을 구운 그림')
        print(f'✅ 그림 회전 적용: {pgm_path}')
    open(a.yaml_path, 'w').write('\n'.join(lines) + '\n')
    print(f'✅ 보정 완료: {a.yaml_path}')
    print()
    print('다음: 순찰 가능 여부 검사')
    print(f'  python3 scripts/check_patrol_space.py --map {a.yaml_path}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
