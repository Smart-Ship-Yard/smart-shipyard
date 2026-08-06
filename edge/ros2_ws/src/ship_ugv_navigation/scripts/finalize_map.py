#!/usr/bin/env python3
"""
finalize_map.py
================
매핑이 끝난 뒤 실행하는 **마무리 명령 하나**.
맵을 저장한 직후 이것만 돌리면 Nav2에 바로 쓸 수 있는 상태가 된다.

    python3 scripts/finalize_map.py shipyard_map_v3

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
import os
import shutil
import subprocess
import sys

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(PKG_ROOT, 'scripts')


def newest(pattern):
    """패턴에 맞는 파일 중 가장 최근에 수정된 것. 없으면 None.

    번호(align_001, align_002...)가 아니라 수정 시각으로 고르는 이유:
    /tmp가 지워지면 카운터가 001로 되돌아가므로 번호가 최신을 보장하지 않는다.
    """
    files = glob.glob(pattern)
    return max(files, key=os.path.getmtime) if files else None


def step(n, title):
    print(f'\n[{n}/4] {title}')
    print('-' * 60)


def main():
    ap = argparse.ArgumentParser(
        description='매핑 후 마무리: 기록 보존 + origin 보정 + 순찰 검사')
    ap.add_argument('map_name',
                    help='맵 이름 (확장자 없이). 예: shipyard_map_v3')
    ap.add_argument('--maps-dir', default=os.path.join(PKG_ROOT, 'maps'))
    ap.add_argument('--align-dir', default='/tmp/slam_map_alignment_results')
    ap.add_argument('--calib-dir', default='/tmp/uwb_calibration_results')
    ap.add_argument('--skip-check', action='store_true')
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
    step(1, '정합·캘리브레이션 기록 보존')
    os.makedirs(records, exist_ok=True)

    align_src = newest(os.path.join(a.align_dir, 'align_*.json'))
    calib_src = newest(os.path.join(a.calib_dir, 'calib_*.json'))

    if align_src is None:
        print(f'❌ 정합 기록을 못 찾았다: {a.align_dir}/align_*.json')
        print('   align 서비스를 호출했는지 확인할 것:')
        print('   ros2 service call /slam_map_alignment/align std_srvs/srv/Trigger')
        print()
        print('   이미 레포에 기록이 있다면 --align-dir 로 그 폴더를 지정해도 된다.')
        return 1

    align_dst = os.path.join(records, f'align_{name}.json')
    shutil.copy2(align_src, align_dst)
    print(f'  정합    {align_src}\n       -> {align_dst}')

    if calib_src is not None:
        calib_dst = os.path.join(records, f'calib_{name}.json')
        shutil.copy2(calib_src, calib_dst)
        print(f'  캘리브  {calib_src}\n       -> {calib_dst}')
    else:
        print(f'  ⚠️ 캘리브레이션 기록 없음 ({a.calib_dir}) — 보정에는 영향 없음')

    # ---- 2·3. origin 보정 + free_thresh 교정 ----
    step(2, 'origin 보정 (slam_map -> map) + free_thresh 교정')
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

    step(4, '순찰 가능 여부 검사')
    r = subprocess.run(
        [sys.executable, os.path.join(SCRIPTS, 'check_patrol_space.py'),
         '--map', yaml_path])

    print()
    print('=' * 60)
    if r.returncode == 0:
        print('  ✅ 완료 — 이 맵으로 Nav2를 돌릴 수 있다')
        print()
        print('  위에 출력된 center_x / center_y / radius 를')
        print('  순찰 노드 파라미터에 넣을 것.')
    elif r.returncode == 2:
        print('  🟡 완료 — 사용 가능하지만 여유가 빠듯하다')
        print()
        print('  위에 출력된 center_x / center_y / radius 를')
        print('  순찰 노드 파라미터에 넣을 것.')
        print('  더 넉넉하게 하려면 대상 주변을 더 치우고 재매핑.')
    else:
        print('  ❌ 맵은 만들어졌지만 순찰 경로가 안 나온다')
        print()
        print('  바닥을 더 치우고 재매핑할 것. 필요한 공터 크기:')
        print('    python3 scripts/check_patrol_space.py --obstacle 0.127 0.127')
    print('=' * 60)
    # 0 / 2 는 사용 가능이므로 성공으로 취급
    return 0 if r.returncode in (0, 2) else 1


if __name__ == '__main__':
    sys.exit(main())
