#!/usr/bin/env python3
"""
finalize_map.py
================
매핑이 끝난 뒤 실행하는 **마무리 명령 하나**.
맵을 저장한 직후 이것만 돌리면 Nav2에 바로 쓸 수 있는 상태가 된다.

    python3 scripts/finalize_map.py shipyard_map_<장소>_v<버전번호>

하는 일 (4가지를 순서대로)
--------------------------
1. `/tmp`에 있는 정합·캘리브레이션 기록(json)을 레포로 복사
   - 슬램 노드들이 결과를 `/tmp/...`에 저장하는데 `/tmp`는 재부팅 시 삭제된다
   - 파일명에 맵 이름을 붙여 저장 → 다음 매핑 때 덮어쓰이지 않음
     (`/tmp`가 지워지면 노드의 번호 카운터가 001부터 다시 시작하므로
      원래 이름 그대로 두면 이전 기록을 덮어쓴다)
2. 맵 yaml의 `origin`을 map 좌표계로 보정 (bake_map_origin.py)
3. `free_thresh`를 0.196으로 교정 (같은 스크립트가 함께 처리)
4. 순찰 가능 여부 검사 (check_patrol_space.py)

전제
----
- `ros2 run nav2_map_server map_saver_cli -f maps/<맵이름>` 이 먼저 끝나 있을 것
- `align` 서비스를 호출해 정합이 완료돼 있을 것 (그래야 json이 생긴다)
- ROS는 필요 없다. 파이썬 표준 라이브러리만 사용하므로 어느 기계에서든 돌아간다.

옵션
----
  --align-dir / --calib-dir   json 위치 (기본: /tmp/...)
  --maps-dir                  맵 폴더 (기본: 이 스크립트 옆의 ../maps)
  --skip-check                순찰 검사 생략
"""

import argparse
import glob
import json
import math
import os
import shutil
import sys as _sys

_sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bake_map_origin import BAKED_MARKER
from measure_ship_center import (MEASURED_PATH, load_measured,
                                 measured_age_text)


def copy_record(src, dst):
    """기록 파일을 레포로 보존한다. 이미 그 파일이면 그냥 둔다.

    /tmp 가 비워졌을 때 --align-dir maps/calibration_records 로 레포 기록을
    다시 쓰는 복구 경로가 있는데(도구가 직접 그렇게 안내한다), 그때 원본과
    대상이 같은 파일이 되어 SameFileError 로 죽었다(2026-08-20).
    """
    if os.path.abspath(src) == os.path.abspath(dst):
        print(f'       (이미 레포에 있는 파일이라 그대로 둔다)')
        return
    shutil.copy2(src, dst)
import subprocess
import sys

# ==========================================================================
#  순찰 대상(모형 배)의 실측 크기 — 줄자로 잰 값이다
# ==========================================================================
#  ★ 측량(ship_survey_node)이 내는 size_xy 를 쓰지 않는 이유 (2026-08-19 실측)
#     같은 배를 네 번 재서 0.115 / 0.589 / 0.699 / 2.560 m 가 나왔다.
#     검출 하나가 배 표면 윤곽이 아니라 **배 중심 한 점**을 주기 때문에,
#     size_xy 는 크기가 아니라 "점들이 얼마나 퍼졌나"(=위치 오차)를 잰다.
#     정지 상태면 안 퍼져서 작게, 주행 중이면 퍼져서 크게 나온다.
#     구조적인 문제라 튜닝으로는 못 고친다.
#
#  그래서 **크기는 실측 상수를 쓰고, 측량에서는 중심만 가져온다.**
#  배가 바뀌면 이 값만 고치면 된다. --ship-size 로 덮어쓸 수도 있다.
# ★ 위에서 내려다본 **가장 넓은 부분**을 재야 한다. 선체가 아니라 갑판이다.
#   모형이 항공모함이라 비행갑판이 선체보다 좌우로 튀어나와 있다. 선체만
#   재면 마스크가 좁아져서, 로봇이 갑판 밑으로 파고들어 부딪힐 수 있다.
#   라이다(21 cm)가 배(19 cm)보다 높아 맵에 안 찍히므로, 이 상수가 배를
#   지키는 유일한 수단이다.
#   ★ 여기에 여유를 섞지 말 것. 순수한 실측 최대치수만 넣는다.
#     여유는 SHIP_MASK_PAD 한 곳에서만 준다. 2026-08-20 까지 쓰던
#     0.80 x 0.17 은 "넉넉하게" 대충 잰 값이라 실제보다 3 cm 컸고,
#     거기에 여유 0.10 이 또 붙어 실질 틈이 11.5 cm 였다. 여유가 두 겹이면
#     얼마나 부풀렸는지 아무도 모르게 되고, 좁은 방에서 순찰 원이 마스크에
#     물리는 원인이 된다.
SHIP_SIZE_XY = (0.77, 0.14)    # m — 5단계 조립 완성형 모형 배
                               #     2026-08-21 정밀 재측정 (앞뒤·좌우 최대)
SHIP_YAW_DEG = 0.0             # 도 — 위 약속을 지키면 항상 0
SHIP_MASK_PAD = 0.07   # m — 배 외곽에서 사방으로 이만큼 더 막는다
#   0.10 -> 0.07 (2026-08-21). 0.10 이면 순찰 원이 마스크에 물린다.
#     마스크 대각(0.97x0.34 기준) 0.514 m
#     반지름 0.60 순찰 시 차체 안쪽 끝  0.511 m   -> 3 mm 부족
#   배 앞뒤 끝에서 로봇이 금지영역에 들어가고, Nav2 가 충돌로 보고 복구
#   동작만 반복한다. 예전에 겪은 그 증상이다.
#   0.07 이면 3.5 cm 여유가 생긴다.
#
#   왜 0.05 가 아닌가: 배가 라이다(21 cm)보다 낮아(19 cm) 맵에 안 찍히므로
#   **마스크가 유일한 보호막**이다. 실시간 회피가 안 된다. UWB 위치 오차가
#   실측 ±3 cm 라(위치핑이 5 cm 폭으로 흔들린다) 0.05 면 실질 여유가 2 cm 다.
#
#   ※ Nav2 의 inflation 은 여기에 더해지는 안전거리가 아니다. 내접반경
#     0.109 = 차체 반폭 0.089 + footprint_padding 0.02 로, 차체 크기를
#     대신 계산하는 값이다. 그래서 로봇 몸통과 배 사이의 실제 틈은
#     이 상수 그대로다.
                               #     -> 마스크 1.00 x 0.37 m

# ★ 배의 방향도 상수다 — **운영 약속**으로 고정한다 (2026-08-19).
#
#   약속: 캘리브레이션을 시작할 때 **로봇을 배와 나란히, 같은 방향으로** 놓는다.
#
#   UWB 캘리브레이션은 로봇의 시작 자세를 map 원점·x축으로 삼는다. 그래서 위
#   약속을 지키면 **배의 긴 축이 곧 map 의 x축**이 되고, 배의 yaw 는 0 이다.
#   측량이 주는 yaw 도, 로봇을 나란히 세워 재는 30초짜리 절차도 필요 없어진다.
#
#   왜 상수로 못 박았었나: yaw 는 크기와 달리 **map 프레임에 상대적**이라
#   캘리브레이션 자리가 바뀌면 값도 바뀐다. 위 약속이 그 변수를 없앤다.
#
#   손으로 놓다 5도쯤 틀어져도 괜찮다. 배 길이 0.80 m 의 끝이
#   0.40 * sin(5도) = 3.5 cm 움직이는데 여유가 0.10 m 다.
#   약속을 못 지킨 판이면 --ship-yaw 로 실측값을 주면 된다
#   (로봇을 배와 나란히 세우고 /odometry/global 의 heading 을 읽는다).

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(PKG_ROOT, 'scripts')


def newest(pattern):
    """패턴에 맞는 파일 중 가장 최근에 수정된 것. 없으면 None.

    번호(align_001, align_002...)가 아니라 수정 시각으로 고르는 이유:
    /tmp가 지워지면 카운터가 001로 되돌아가므로 번호가 최신을 보장하지 않는다.
    """
    files = glob.glob(pattern)
    return max(files, key=os.path.getmtime) if files else None


# 정합을 호출한 뒤 맵을 저장하기까지 걸리는 시간의 허용 범위(초).
# 맵을 먼저 저장하고 정합을 다시 호출하는 순서도 있을 수 있어 여유를 둔다.
SAVE_TOLERANCE_S = 120


def pick_align(align_dir, pgm_mtime):
    """맵에 대응하는 정합 기록을 고른다.

    핵심: "가장 최신"이 아니라 **"맵 저장 시점 이전의 것 중 가장 최신"**을 고른다.

    왜 그냥 최신을 쓰면 안 되나 —
        14:00  align_001 생성 (A방)
        14:05  map_v3.pgm 저장
        14:10  align_002 생성 (B방)
        14:15  map_v4.pgm 저장
        14:20  finalize_map.py shipyard_map_<장소>_v<버전번호>
    이때 "최신"은 align_002(B방)이지만 v3에 필요한 것은 align_001(A방)이다.
    시간 차이가 5분뿐이라 차이 기반 검사로는 절대 잡히지 않는다.
    맵보다 나중에 만들어진 정합 기록은 그 맵의 것일 수 없다는 순서 규칙으로 판별한다.

    반환: (고른 파일 또는 None, 전체 후보 목록[시각순])
    """
    files = sorted(glob.glob(os.path.join(align_dir, 'align_*.json')),
                   key=os.path.getmtime)
    if not files:
        return None, []
    before = [f for f in files if os.path.getmtime(f) <= pgm_mtime + SAVE_TOLERANCE_S]
    return (before[-1] if before else None), files


def fmt_time(path):
    import datetime
    return datetime.datetime.fromtimestamp(
        os.path.getmtime(path)).strftime('%m-%d %H:%M:%S')


def pick_survey_center(survey_dir, records, name, pgm_mtime):
    """배 중심좌표 측량 기록(ship_pose_*.json)이 있으면 읽어서 중심·크기·yaw 를 준다.

    젯슨의 ship_survey_node 가 매핑 주행 중 뎁스카메라로 배 표면점을 모아
    중심을 계산하고 남기는 파일이다. 형식은 docs/interface.md ④번과 같다.

    선택 규칙은 pick_align 과 같다 — **맵 저장 시각 이전의 것 중 가장 최신**.
    번호가 아니라 시각으로 고르는 이유는 /tmp 가 지워지면 카운터가 001 로
    되돌아가 번호가 최신을 보장하지 않기 때문이다. 그리고 맵보다 나중에 만들어진
    측량 기록은 그 맵의 것일 수 없다 (배를 옮기고 다시 측량했을 수 있다).

    라이다에 안 잡히는 낮은 대상(모형 배)을 위한 것이라 없는 것이 정상이며,
    없으면 맵에서 자동 탐지한다. 그래서 실패해도 중단하지 않는다.
    """
    files = sorted(glob.glob(os.path.join(survey_dir, 'ship_pose_*.json')),
                   key=os.path.getmtime)
    if not files:
        print(f'  측량    기록 없음 ({survey_dir}) — 맵에서 자동 탐지한다')
        return None

    before = [f for f in files if os.path.getmtime(f) <= pgm_mtime + SAVE_TOLERANCE_S]
    if not before:
        print(f'  ⚠️ 측량 기록 {len(files)}개가 모두 맵 저장보다 나중이다 '
              f'— 이 맵의 것이 아닐 가능성이 높다')
        for f in files:
            print(f'       {fmt_time(f)}  {os.path.basename(f)}')
        print('     맵에서 자동 탐지로 진행한다')
        return None

    src = before[-1]
    if len(files) > 1:
        print(f'  측량 기록 {len(files)}개 중 맵 저장 이전의 최신 것을 고른다:')
        for f in files:
            rel = '이전' if f in before else '이후'
            mark = '  <-- 선택' if f == src else ''
            print(f'       {fmt_time(f)}  [{rel}]  {os.path.basename(f)}{mark}')
    try:
        with open(src) as f:
            d = json.load(f)
        xy = d['map_xy']
        cx, cy = float(xy[0]), float(xy[1])
    except Exception as e:
        print(f'  ⚠️ 측량 기록을 읽지 못했다 ({src}): {e}')
        print('     맵에서 자동 탐지로 진행한다')
        return None

    dst = os.path.join(records, f'survey_{name}.json')
    copy_record(src, dst)

    yaw = d.get('yaw')
    yaw = float(yaw) if isinstance(yaw, (int, float)) else 0.0

    # size_xy 는 keepout 마스크를 그리는 데 쓴다. 대상이 라이다에 안 잡히면
    # 맵에서 크기를 알아낼 수 없으므로, 이 값이 없으면 마스크를 못 만든다.
    size = d.get('size_xy')
    if isinstance(size, (list, tuple)) and len(size) == 2:
        size = (float(size[0]), float(size[1]))
    else:
        size = None

    # 측량에서 실제로 쓰는 것은 **중심 좌표뿐**이다.
    # 크기(size_xy)와 방향(yaw)도 파일에는 들어 있지만 마스크에 쓰지 않는다
    # — 크기는 점이 퍼진 정도를 잰 값이라 못 믿고, 방향은 실측 상수로 고정했다.
    # 예전에는 이 둘까지 화면에 찍어서 "저 값이 마스크에 쓰이나" 하고
    # 헷갈리게 했다(2026-08-20). 안 쓰는 값은 안 찍는다.
    print(f'  측량    {src}\n       -> {dst}')
    print(f'          배 중심 ({cx:.3f}, {cy:.3f})  ← 마스크·순찰이 쓰는 값은 이것뿐')
    return {'center': (cx, cy), 'yaw': yaw, 'size': size}


def step(n, title):
    print(f'\n[{n}/4] {title}')
    print('-' * 60)



def read_patrol_yaml(path):
    """이미 만들어둔 순찰 설정에서 (중심x, 중심y, 반지름) 을 읽는다."""
    try:
        got = {}
        for line in open(path):
            line = line.split('#')[0].strip()
            for key in ('center_x', 'center_y', 'radius'):
                if line.startswith(key + ':'):
                    got[key] = float(line.split(':', 1)[1])
        return (got['center_x'], got['center_y'], got['radius'])
    except (OSError, ValueError, KeyError):
        return None


def print_recovery_options(name):
    """정합 기록이 미심쩍을 때 사람이 고를 두 갈래.

    판단 기준은 하나뿐이다: **맵을 저장한 뒤에 로봇이 더 움직였는가.**
    움직였으면 저장된 맵과 지금 정합값이 서로 다른 것을 가리킨다.
    두 중단 지점에서 같은 안내를 쓰므로 여기 하나로 모았다.
    """
    print('  아무 파일도 고치지 않았다.')
    print('  판단 기준은 하나다 — 맵을 저장한 뒤에 로봇이 더 돌아다녔는가?')
    print()
    print('  1) 저장한 뒤에도 더 주행했다  ->  정합값을 직접 읽어서 넣는다')
    print('     저장 이후 맵이 자라서, 저장본과 지금 정합값이 서로 안 맞는다.')
    print('     지금 TF 를 읽는다:')
    print('         ros2 run tf2_ros tf2_echo map slam_map')
    print('     ★ uwb_frame 이 아니라 slam_map 이다. 둘은 다른 변환이다.')
    print('        map<-uwb_frame  = UWB 캘리브레이션 결과 (여기에 넣으면 안 된다)')
    print('        map<-slam_map   = 정합 결과 (이게 필요한 값이다)')
    print('     나온 Translation 의 x, y 와 Rotation RPY 의 yaw(radian)를 그대로:')
    print(f'         python3 scripts/bake_map_origin.py maps/{name}.yaml --tf <x> <y> <yaw>')
    print()
    print('  2) 저장 직후 그 자리에 세워둔 채였다  ->  그냥 진행한다')
    print('     로봇이 안 움직였으면 맵도 안 바뀌었다. 정합값은 이 맵에 그대로 맞다.')
    print(f'         python3 scripts/finalize_map.py {name} --force')

def main():
    ap = argparse.ArgumentParser(
        description='매핑 후 마무리: 기록 보존 + origin 보정 + 순찰 검사')
    ap.add_argument('map_name',
                    help='맵 이름 (확장자 없이). 예: shipyard_map_JG_room_v2, +) 버전번호는 같은 장소에서 여러 번 맵핑했을 때 맵핑한 순서를 기입하면 됨')
    ap.add_argument('--maps-dir', default=os.path.join(PKG_ROOT, 'maps'))
    ap.add_argument('--align-dir', default='/tmp/slam_map_alignment_results')
    ap.add_argument('--align-file',
                    help='쓸 정합 기록을 직접 지정 (자동 선택을 건너뜀)')
    ap.add_argument('--calib-dir', default='/tmp/uwb_calibration_results')
    ap.add_argument('--survey-dir', default='/tmp/ship_survey_results',
                    help='배 중심좌표 측량 기록(ship_pose_*.json) 위치. '
                         '있으면 순찰 중심으로 쓴다. 없으면 맵에서 자동 탐지')
    ap.add_argument('--center', nargs=2, type=float, metavar=('X', 'Y'),
                    help='순찰 중심을 직접 지정 (map 좌표). 맵에 포위된 장애물이 '
                         '여럿이라 자동 선택이 틀렸을 때 쓴다. 정확히 찍을 필요 없다 '
                         '— 가장 가까운 장애물을 지목한 것으로 보고 그 무게중심을 쓴다')
    ap.add_argument('--mask-pad', type=float, metavar='M',
                    default=SHIP_MASK_PAD,
                    help=f'마스크를 대상 외곽에서 이만큼 더 부풀린다(m). 기본 {SHIP_MASK_PAD}. '
                         '측량 중심에 오차가 있을 때 그만큼 키워 덮는 용도다. '
                         '예: 배 실측 0.77 x 0.14 m 에 사방 0.10 m 를 주면 '
                         '마스크가 1.00 x 0.37 m 가 된다. '
                         '키울수록 순찰 여유를 깎으니 경고가 뜨면 줄일 것.')
    ap.add_argument('--ship-yaw', dest='ship_yaw_deg', type=float, metavar='DEG',
                    default=SHIP_YAW_DEG,
                    help='순찰 대상의 방향(도). --center 와 --ship-size 를 손으로 줄 때 '
                         '함께 준다. 안 주면 0도로 그려서, 길쭉한 대상은 마스크가 '
                         '엉뚱한 방향으로 눕는다. '
                         '재는 법: 로봇을 대상과 나란히 세우고 '
                         '`ros2 topic echo /odometry/global` 의 heading 을 읽으면 된다 '
                         '(정지 상태 실측 흔들림 0.05도). '
                         '측량 기록이 있으면 그쪽 yaw 가 자동으로 쓰이므로 불필요하다.')
    ap.add_argument('--ship-size', nargs=2, type=float, metavar=('W', 'H'),
                    default=list(SHIP_SIZE_XY),
                    help='순찰 대상의 크기(m). keepout 마스크를 그리는 데 쓴다. '
                         '대상이 라이다에 안 잡혀(모형 배 등) 맵에서 크기를 '
                         '알 수 없을 때 --center 와 함께 준다. 측량 기록에 '
                         'size_xy 가 있으면 그것보다 이 값이 우선한다')
    ap.add_argument('--radius', type=float, default=None, metavar='M',
                    help='순찰 반지름을 직접 고른다 (m). 안 주면 여유가 가장 큰 값')
    ap.add_argument('--max-radius', type=float, default=None, metavar='M',
                    help='순찰 반지름을 여기까지만 훑는다 (기본 1.00 m)')
    ap.add_argument('--use-survey', action='store_true',
                    help='YOLO 측량이 낸 배 중심을 쓴다 (기본은 거부 — 부정확하다)')
    ap.add_argument('--no-mask', action='store_true',
                    help='keepout 마스크를 만들지 않는다. 순찰 대상이 라이다에 '
                         '잡히는 물체라면(코스트맵이 이미 안다) 마스크는 중복이며, '
                         '좁은 맵에서는 순찰 여유만 깎는다')
    ap.add_argument('--skip-check', action='store_true')
    ap.add_argument('--force', action='store_true',
                    help='정합 기록과 맵의 시각 차이 검사를 무시하고 진행')
    a = ap.parse_args()

    name = a.map_name.replace('.yaml', '').replace('.pgm', '')
    yaml_path = os.path.join(a.maps_dir, name + '.yaml')
    pgm_path = os.path.join(a.maps_dir, name + '.pgm')
    records = os.path.join(a.maps_dir, 'calibration_records')

    print('=' * 60)
    print(f'  맵 마무리: {name}')
    print('=' * 60)

    # ---- 전제 확인 ----
    missing = [p for p in (yaml_path, pgm_path) if not os.path.exists(p)]
    if missing:
        print('\n❌ 맵 파일이 없다:')
        for p in missing:
            print(f'   {p}')
        print('\n먼저 맵을 저장할 것:')
        print(f'   ros2 run nav2_map_server map_saver_cli -f {a.maps_dir}/{name}')
        return 1

    # ---- 1. 기록 보존 ----
    # ★ 이미 보정된 맵이면 [1/4] 도 건너뛴다 (2026-08-21).
    #   기록 보존은 처음 마무리할 때 이미 끝났다(maps/calibration_records/).
    #   그런데 이 단계가 /tmp 의 정합 기록을 찾으므로, 며칠 뒤 반지름만
    #   바꾸려고 다시 돌리면 "/tmp 에 정합 기록이 없다" 며 멈췄다.
    #   /tmp 는 재부팅으로 비워지니 사실상 매번 막힌다.
    already_baked = BAKED_MARKER in open(yaml_path).read()
    survey = None
    align_dst = None

    if already_baked:
        step(1, '정합·캘리브레이션 기록 보존 — 건너뜀')
        print('  이미 마무리된 맵이다. 기록은 처음 마무리할 때 보존했다:')
        for kind in ('align', 'calib', 'survey'):
            got = os.path.join(records, f'{kind}_{name}.json')
            if os.path.exists(got):
                print(f'    {kind:7s} maps/calibration_records/{kind}_{name}.json')
        print('  순찰 설정·마스크만 다시 만든다 (반지름을 바꿀 때 이 경로다).')

    if not already_baked:
        step(1, '정합·캘리브레이션 기록 보존')
    os.makedirs(records, exist_ok=True)

    pgm_mtime = os.path.getmtime(pgm_path)
    if a.align_file:
        if not os.path.exists(a.align_file):
            print(f'❌ 지정한 정합 기록이 없다: {a.align_file}')
            if already_baked:
                print('  (이미 마무리된 맵이라 무시하고 넘어간다)')
            else:
                return 1
        align_src, all_aligns = a.align_file, [a.align_file]
        print(f'  정합 기록을 직접 지정함: {a.align_file}')
    else:
        align_src, all_aligns = pick_align(a.align_dir, pgm_mtime)
    calib_src = newest(os.path.join(a.calib_dir, 'calib_*.json'))

    if not all_aligns:
        print(f'❌ 정합 기록을 못 찾았다: {a.align_dir}/align_*.json')
        print('   align 서비스를 호출했는지 확인할 것:')
        print('   ros2 service call /slam_map_alignment/align std_srvs/srv/Trigger')
        print()
        print('   이미 레포에 기록이 있다면 --align-dir 로 그 폴더를 지정해도 된다.')
        if already_baked:
            print('  (이미 마무리된 맵이라 무시하고 넘어간다)')
        else:
            return 1

    def show_candidates():
        print(f'  맵 저장 시각        {fmt_time(pgm_path)}  ({name}.pgm)')
        print('  정합 기록 목록:')
        for f in all_aligns:
            mark = '  <-- 선택' if f == align_src else ''
            rel = '이전' if os.path.getmtime(f) <= pgm_mtime + SAVE_TOLERANCE_S else '이후'
            print(f'    {fmt_time(f)}  [{rel}]  {os.path.basename(f)}{mark}')

    # 맵보다 나중에 만들어진 정합 기록만 있다면 어느 것이 맞는지 알 수 없다.
    if align_src is None and not a.force:
        show_candidates()
        print()
        print('  ⛔ 중단: 맵 저장보다 나중에 만들어진 정합 기록밖에 없다.')
        print('     이 맵의 정합 기록이 아닐 가능성이 높다.')
        print('     (맵을 저장한 뒤에 align을 다시 호출했다면 정상일 수 있다)')
        print()
        print_recovery_options(name)
        if already_baked:
            print('  (이미 마무리된 맵이라 무시하고 넘어간다)')
        else:
            return 1

    if align_src is None and all_aligns:   # --force 인 경우 최신 것으로 진행
        align_src = all_aligns[-1]
    if align_src is None:
        # 정합 기록이 아예 없다 (/tmp 가 비워졌을 때). 이미 마무리된 맵이면
        # [2/4] 를 건너뛸 것이라 문제없다 — 순찰 설정만 다시 만들면 된다.
        if not already_baked:
            print('❌ 정합 기록이 없어 origin 보정을 할 수 없다.')
            return 1
        print('  정합 기록이 없지만 이미 마무리된 맵이라 그냥 넘어간다.')
        align_dst = os.path.join(records, f'align_{name}.json')

    # ★ 2026-08-24: 여기부터는 정합 기록이 **실제로 있을 때만** 볼 수 있다.
    #   예전에는 이 블록이 무조건 돌아서, 위에서 "이미 마무리된 맵이라
    #   무시하고 넘어간다" 고 출력해놓고 정작 넘어가지 않고 그대로 내려와
    #   align_src=None 으로 os.path.getmtime(None) 을 불러 죽었다:
    #       TypeError: stat: path should be string, ... not NoneType
    #   /tmp 가 재부팅으로 비워진 뒤 반지름만 바꾸려고 돌리면 항상 이랬다
    #   (실측 2026-08-24). 안내 문구가 거짓말이 되지 않도록 실제로 건너뛴다.
    if align_src is not None:
        # 고른 기록이 맵보다 한참 오래됐으면 다른 세션의 것일 수 있다.
        # 잘못된 변환이 적용된 맵은 겉보기에 멀쩡해서 Nav2를 돌려봐야 알게 되므로
        # 경고만 하지 않고 중단한다 (아직 아무 파일도 고치지 않은 시점).
        gap_min = (pgm_mtime - os.path.getmtime(align_src)) / 60.0
        if gap_min > 30 and not a.force:
            show_candidates()
            print()
            print(f'  ⛔ 중단: 고른 정합 기록이 맵 저장보다 {gap_min:.0f}분 앞선다.')
            print('     다른 매핑 세션의 기록일 가능성이 있다.')
            print('     (정상 절차라면 align 직후 맵을 저장하므로 몇 분 이내여야 한다)')
            print()
            print_recovery_options(name)
            print()
            print('  판단 근거: 아래 값이 매핑 때 본 tf2_echo map slam_map 과 같은지 확인')
            print('  ' + '-' * 56)
            for line in open(align_src).read().splitlines():
                print('  ' + line)
            if already_baked:
                print('  (이미 마무리된 맵이라 무시하고 넘어간다)')
            else:
                return 1

        if len(all_aligns) > 1:
            show_candidates()
            print()

        align_dst = os.path.join(records, f'align_{name}.json')
        copy_record(align_src, align_dst)
        print(f'  정합    {align_src}\n       -> {align_dst}')

    if calib_src is not None:
        calib_dst = os.path.join(records, f'calib_{name}.json')
        copy_record(calib_src, calib_dst)
        print(f'  캘리브  {calib_src}\n       -> {calib_dst}')
    else:
        print(f'  ⚠️ 캘리브레이션 기록 없음 ({a.calib_dir}) — 보정에는 영향 없음')

    # 배 중심좌표 측량 기록 (젯슨의 ship_survey_node 가 남긴다).
    # 라이다에 안 잡히는 낮은 대상을 뎁스카메라로 측량한 결과다.
    # 없으면 맵에서 자동 탐지하므로 필수는 아니다.
    survey = pick_survey_center(a.survey_dir, records, name, pgm_mtime)

    # ---- 2·3. origin 보정 + free_thresh 교정 ----
    step(2, 'origin 보정 (slam_map -> map) + free_thresh 교정')
    # 이미 보정된 맵이면 [2/4] 를 건너뛴다 (2026-08-21).
    #   반지름만 바꾸려고 다시 돌리는 경우가 있는데, 예전에는 여기서
    #   "이미 보정된 맵이다" 로 멈춰서 [4/4] 가 아예 안 돌았다. 그래서
    #   --radius 를 줘도 아무 일도 안 일어났다.
    #   ※ 다시 굽는 것은 위험하다. 회전이 두 번 들어가 맵이 통째로 틀어진다.
    #     되굽고 싶으면 .orig 로 되돌린 뒤 실행할 것.
    if BAKED_MARKER in open(yaml_path).read():
        print('  이미 보정된 맵이다 — origin·회전은 그대로 두고 넘어간다.')
        print('  (순찰 설정·마스크만 다시 만든다. 반지름을 바꾸려면 이게 맞다)')
        print('  다시 구우려면 원본으로 되돌린 뒤 실행할 것:')
        base = os.path.splitext(yaml_path)[0]
        print(f'        cp {base}.yaml.orig {base}.yaml')
        print(f'        cp {base}.pgm.orig  {base}.pgm')
    else:
        r = subprocess.run(
            [sys.executable, os.path.join(SCRIPTS, 'bake_map_origin.py'),
             yaml_path, '--align', align_dst])
        if r.returncode != 0:
            print('\n❌ origin 보정 실패. 위 메시지를 확인할 것.')
            return 1

    step(3, '보정 결과')
    print(open(yaml_path).read())

    # ---- 4. 순찰 검사 ----
    if a.skip_check:
        print('\n[4/4] 순찰 검사 생략 (--skip-check)')
        return 0

    step(4, '순찰 가능 여부 검사 + 순찰 설정 생성')
    # ★ --emit-patrol 을 주면 순찰 원(중심·반지름·웨이포인트 개수)을 계산해
    #   config/patrol_<맵이름>.yaml 을 직접 만든다. 사람이 출력값을 보고
    #   손으로 옮겨 적는 단계를 없애기 위함이다(오타·누락 방지).
    #   순찰 불가 맵이면 파일을 만들지 않는다.
    patrol_out = os.path.join(PKG_ROOT, 'config', f'patrol_{name}.yaml')

    # ★ --emit-mask 는 Nav2 KeepoutFilter 용 마스크를 만든다.
    #   라이다 스캔 평면보다 낮은 대상은 코스트맵에 안 잡히므로, 대상 자리를
    #   코스트맵에 직접 박아 로봇이 가로지르지 못하게 한다.
    #   순찰 대상의 크기를 알 수 없으면 만들지 않고 그 이유를 출력한다.
    mask_out = os.path.join(PKG_ROOT, 'masks', f'keepout_{name}.yaml')

    cmd = [sys.executable, os.path.join(SCRIPTS, 'check_patrol_space.py'),
           '--map', yaml_path,
           '--emit-patrol', patrol_out]
    if a.radius is not None:
        cmd += ['--radius', str(a.radius)]
    if a.max_radius is not None:
        cmd += ['--max-radius', str(a.max_radius)]

    if a.no_mask:
        # 오래된 마스크가 남아 있으면 launch 가 그것을 켜 버린다. 같이 지운다.
        for p in (mask_out, os.path.splitext(mask_out)[0] + '.pgm'):
            if os.path.exists(p):
                os.remove(p)
                print(f'  기존 마스크 삭제: {p}')
        print('  keepout 마스크: 만들지 않음 (--no-mask)')
    else:
        cmd += ['--emit-mask', mask_out]

    # 중심 결정 우선순위: 사람이 직접 지정 > 측량 기록 > 맵에서 자동 탐지.
    # 사람이 준 값을 가장 위에 두는 이유는, 자동 선택이 틀렸을 때
    # 되돌릴 방법이 그것뿐이기 때문이다.
    if a.center:
        cmd += ['--center', str(a.center[0]), str(a.center[1])]
        cx_used, cy_used = a.center
        print(f'  순찰 중심: --center 로 실측해 지정됨 '
              f'({a.center[0]}, {a.center[1]})')
        if a.ship_size:
            cmd += ['--mask-size', str(a.ship_size[0]), str(a.ship_size[1])]
            print(f'  대상 크기: --ship-size {a.ship_size[0]} x {a.ship_size[1]} m')
            # ★ 방향까지 받는다 (2026-08-19 추가). 예전에는 --center 로 손수
            #   지정하면 마스크 방향이 무조건 0도로 그려졌다. 정사각형에 가까운
            #   대상이면 티가 안 나지만, 우리 모형 배처럼 0.80 x 0.18 m 로
            #   길쭉하면 90도 틀어진 마스크가 배를 절반도 못 덮는다.
            if a.ship_yaw_deg is not None:
                cmd += ['--mask-yaw', str(math.radians(a.ship_yaw_deg))]
                print(f'  대상 방향: --ship-yaw {a.ship_yaw_deg:.2f}도')
            if a.mask_pad is not None:
                cmd += ['--mask-pad', str(a.mask_pad)]
                print(f'  마스크 여유: --mask-pad {a.mask_pad} m (사방)')
            else:
                print('  대상 방향: 지정 안 됨 -> 0도로 그린다. '
                      '길쭉한 대상이면 --ship-yaw 를 줄 것')
    elif load_measured() is not None and not a.use_survey:
        mx, my, meta = load_measured()
        cx_used, cy_used = mx, my
        cmd += ['--center', str(mx), str(my)]
        print(f'  순찰 중심: 실측 기록에서 읽음 ({mx:.3f}, {my:.3f})')
        print(f'             config/{os.path.basename(MEASURED_PATH)} — '
              f"{measured_age_text(meta)}")
        print(f"             뎁스 {meta.get('depth_m', float('nan')):.3f} m, "
              f"검출 {meta.get('samples', '?')}개")
        cmd += ['--mask-size', str(SHIP_SIZE_XY[0]), str(SHIP_SIZE_XY[1]),
                '--mask-yaw', str(math.radians(a.ship_yaw_deg
                                               if a.ship_yaw_deg is not None
                                               else SHIP_YAW_DEG))]
        cmd += ['--mask-pad', str(a.mask_pad if a.mask_pad is not None
                                  else SHIP_MASK_PAD)]
        print(f'  대상 크기: 실측 상수 SHIP_SIZE_XY '
              f'{SHIP_SIZE_XY[0]:.2f} x {SHIP_SIZE_XY[1]:.2f} m')

    elif survey and not a.use_survey:
        print('  ⛔ 중단: --center 로 실측한 배 중심을 주지 않았다.')
        print()
        print('     YOLO 측량(ship_survey_node)이 낸 값이 있긴 하다:')
        print(f"       중심 ({survey['center'][0]:.3f}, {survey['center'][1]:.3f})"
              + (f"  크기 {survey['size'][0]:.2f} x {survey['size'][1]:.2f} m"
                 if survey.get('size') else ''))
        print('     하지만 이 값은 믿을 수 없어서 2026-08-20 부터 기본으로 안 쓴다.')
        print('     실측 0.77 x 0.14 m 인 배를 1.68 x 0.35 m 로 쟀다. 방향은 노이즈다.')
        print('     이 값 하나가 keepout 마스크 위치와 프론트엔드 화면 전체의')
        print('     기준 좌표계가 되므로, 틀리면 자율주행도 화면도 같이 틀어진다.')
        print()
        print('  ▶ 줄자로 재서 넣을 것:')
        print('     원점은 캘리브레이션을 **시작한** 순간의 로봇 위치,')
        print('     +x 는 로봇이 직진한 방향, +y 는 그 기준 왼쪽,')
        print('     로봇 기준점은 base_link = 좌우 구동 바퀴 축의 중점이다.')
        print('     (로봇을 배 왼편에 나란히 두는 약속이면 X 는 +, Y 는 -)')
        print()
        print(f'       python3 scripts/finalize_map.py {name} --center <X> <Y>')
        print()
        print('  ▶ 또는 뎁스로 1회 측정하면 이 명령이 알아서 읽는다:')
        print('       python3 scripts/measure_ship_center.py')
        print(f'       python3 scripts/finalize_map.py {name}')
        print()
        print('     굳이 측량값을 쓰겠다면:  --use-survey')
        return 1

    elif survey:
        cx, cy = survey['center']
        cx_used, cy_used = cx, cy
        cmd += ['--center', str(cx), str(cy)]
        print(f'  순찰 중심: 측량 기록에서 가져옴 ({cx:.3f}, {cy:.3f})')
        size = a.ship_size or survey['size']
        if size:
            # ★ 방향도 --ship-yaw 가 있으면 그쪽이 이긴다 (2026-08-19).
            #   측량 yaw 는 점 뭉치에 사각형을 맞춘 값이라, 점이 안 퍼지면
            #   (정지 측량) 11 cm 뭉치에서 나온 무의미한 각도가 된다.
            #   실측: 측량 85.7도 vs 로봇을 배와 나란히 세워 잰 -15.2도.
            use_yaw = (math.radians(a.ship_yaw_deg)
                       if a.ship_yaw_deg is not None else survey['yaw'])
            cmd += ['--mask-size', str(size[0]), str(size[1]),
                    '--mask-yaw', str(use_yaw)]
            if a.ship_yaw_deg is not None:
                print(f'  대상 방향: --ship-yaw {a.ship_yaw_deg:.2f}도 '
                      f'(측량값 {math.degrees(survey["yaw"]):.1f}도 대신)')
            else:
                print(f'  대상 방향: 측량 기록 '
                      f'{math.degrees(survey["yaw"]):.1f}도 '
                      f'— 길쭉한 대상이면 --ship-yaw 로 실측값을 줄 것')
            src_label = ('--ship-size' if a.ship_size != list(SHIP_SIZE_XY)
                         else f'실측 상수 SHIP_SIZE_XY')
            print(f'  대상 크기: {src_label} {size[0]:.2f} x {size[1]:.2f} m')
            if a.mask_pad is not None:
                cmd += ['--mask-pad', str(a.mask_pad)]
                print(f'  마스크 여유: --mask-pad {a.mask_pad} m (사방)')
    else:
        cx_used = cy_used = None
        print('  순찰 중심: 맵에서 자동 탐지')
    print()

    # 같은 값으로 다시 돌리면 3분짜리 반지름 훑기를 통째로 낭비한다.
    # 결과 파일도 똑같이 나온다. 미리 비교해서 건너뛴다 (2026-08-21).
    have = read_patrol_yaml(patrol_out) if not a.no_mask else None
    if (have and cx_used is not None and not a.force
            and abs(have[0] - cx_used) < 0.001
            and abs(have[1] - cy_used) < 0.001
            and (a.radius is None or abs(have[2] - a.radius) < 0.001)):
        print(f'  ✅ 이미 이 값으로 설정돼 있다 — 다시 만들지 않는다')
        print(f'       중심 ({have[0]:.3f}, {have[1]:.3f})   반지름 {have[2]:.2f} m')
        print(f'       config/{os.path.basename(patrol_out)}')
        print()
        if a.radius is None:
            print('     다른 반지름으로 바꾸려면:')
            print(f'       python3 scripts/finalize_map.py {name} --radius <반지름>')
        else:
            print('     그래도 다시 만들려면 --force')
        print()
        print(f'  ▶ 배 중심   X = {cx_used:+.3f}   Y = {cy_used:+.3f}')
        return 0

    r = subprocess.run(cmd)

    print()
    print('=' * 60)
    if r.returncode == 0:
        print('  ✅ 완료 — 이 맵으로 Nav2를 돌릴 수 있다')
        print()
        print(f'  순찰 설정이 자동으로 만들어졌다: config/patrol_{name}.yaml')
        print('  손으로 값을 옮겨 적을 필요 없다. 실행할 때 map 이름만 주면 된다:')
        print(f'    ros2 launch ship_ugv_navigation navigation.launch.py \\')
        print(f'        map:={name} patrol:=true space:=wide')
    elif r.returncode == 2:
        print('  🟡 완료 — 사용 가능하지만 여유가 빠듯하다')
        print()
        print(f'  순찰 설정이 자동으로 만들어졌다: config/patrol_{name}.yaml')
        print('  실행할 때 **space:=narrow** 를 반드시 함께 줄 것:')
        print(f'    ros2 launch ship_ugv_navigation navigation.launch.py \\')
        print(f'        map:={name} patrol:=true space:=narrow')
        print()
        print('  더 넉넉하게 하려면 대상 주변을 더 치우고 재매핑.')
    else:
        print('  ❌ 맵은 만들어졌지만 순찰 경로가 안 나온다')
        print()
        print('  바닥을 더 치우고 재매핑할 것. 필요한 공터 크기:')
        print('    python3 scripts/check_patrol_space.py --obstacle 0.127 0.127')
    print()
    print('  ⚠️  방금 만든 파일은 src/ 에만 있다. navigation.launch.py 로 켜기 전에:')
    print('        cd ~/smart-shipyard/edge/ros2_ws && \\')
    print('          colcon build --symlink-install --packages-select ship_ugv_navigation')
    print('      안 하면 install/ 에 안 복사돼서 "맵을 못 찾는다" 에러가 난다.')
    print()
    if cx_used is not None:
        print()
        print(f'  ▶ 배 중심   X = {cx_used:+.3f}   Y = {cy_used:+.3f}   '
              f'(yaw 는 0 으로 고정)')
        print()
        print('  📡 같은 배 위치를 프론트엔드에도 보내야 화면이 맞는다:')
        print('        python3 scripts/publish_ship_pose.py'
              + ('' if load_measured() else f' {cx_used:.3f} {cy_used:.3f}'))
        print('      (프론트는 배 pose 를 기준 좌표계로 써서 로봇·이벤트를 그린다.')
        print('       안 보내면 화면의 로봇 위치가 통째로 어긋난다)')
    print()
    print('      source 는 Nav2 를 띄울 터미널에서만 하면 된다:')
    print('        source ~/smart-shipyard/edge/ros2_ws/install/setup.bash')
    print('      (심볼릭 링크 install 이라, 기존 패키지에 파일만 추가된 이번 같은')
    print('       경우엔 사실 다시 안 해도 된다. 새 패키지를 만들었을 때만 필수다)')
    print('=' * 60)
    # 0 / 2 는 사용 가능이므로 성공으로 취급
    return 0 if r.returncode in (0, 2) else 1


if __name__ == '__main__':
    sys.exit(main())
