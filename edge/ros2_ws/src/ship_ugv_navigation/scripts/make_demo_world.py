#!/usr/bin/env python3
"""
make_demo_world.py
===================
시뮬용 Gazebo 월드(.world)와 그에 정확히 대응하는 Nav2 맵(.pgm/.yaml)을
"같은 파라미터 하나"로부터 동시에 생성한다.

왜 이렇게 하는가
----------------
월드를 손으로 짜고 맵을 따로 만들면 반드시 어긋난다(벽 위치 5cm 차이 같은 것).
그러면 시뮬에서 "Nav2가 벽을 못 피한다"는 현상이 나오는데, 원인이 Nav2인지
맵인지 구분이 안 돼 튜닝이 통째로 낭비된다. 한 곳에서 같이 만들면 정의상 일치한다.

왜 실측 JG방 맵을 안 쓰고 새로 만드는가
-------------------------------------
JG방 첫 매핑(당시 이름 shipyard_map_v1_JG_room)은 12.07 m^2 중 3.99 m^2만
탐사됐고, 그 자유공간이 대각선 띠 모양이라 배 주위로 원을 그릴 공간이 없었다.
실측 검증 결과 완주 가능한 반지름이 0.35m 하나뿐이고 안전여유가 0이었다
(2026-08-05 분석). 쓸 수 없는 맵이라 2026-08-12에 삭제했고, 폼롤러로 대상을
바꿔 다시 찍은 shipyard_map_JG_room_v2 가 그 자리를 대신한다.
(지운 맵이 필요하면 git 히스토리에서 꺼낼 수 있다)
=> 실측 맵은 "장애물 회피 동작 확인용"으로만 쓰고, 순찰 파라미터 튜닝은
   "시연장이 갖춰야 할 조건"을 만족하는 이 월드에서 한다.

시연장 요구 조건 (이 월드가 구현하는 것)
--------------------------------------
로봇이 반지름 R 원을 돌면 차체는 배 중심 기준
    안쪽 (R - 0.089) m  ~  바깥쪽 sqrt((R+0.089)^2 + 0.332^2) m
구간을 쓸고 지나간다 (base_link가 뒷바퀴 축이라 앞으로 0.332m 튀어나옴).
안팎 15cm 여유를 주면, 배 중심에서 사방 0.85m 이상이 자유공간이어야 한다.
=> 배를 중심으로 최소 1.7 x 1.7 m 의 트인 공간이 필요.

사용법
------
  python3 make_demo_world.py
  (패키지 루트에서 실행. worlds/ 와 maps/ 에 결과물이 생성됨)
"""

import os

# ======================================================================
# 파라미터 — 시연장이 달라지면 여기 숫자만 바꾸고 다시 실행하면 된다
# ======================================================================
ROOM_X = 3.5          # 방 내부 가로 (m) — JG방 실측 3.55m와 동급
ROOM_Y = 3.4          # 방 내부 세로 (m) — JG방 실측 3.40m와 동급
WALL_THICK = 0.05     # 벽 두께 (m)
WALL_HEIGHT = 0.5     # 벽 높이 (m) — 라이다 장착고 0.2m보다 충분히 높게

SHIP_X = 0.40         # 모형배 가로 (m) — JG방 맵에서 측정한 0.35 x 0.40 과 동급
SHIP_Y = 0.50         # 모형배 세로 (m)
SHIP_Z = 0.25         # 모형배 높이 (m) — 라이다(0.2m)에 반드시 잡히도록
SHIP_CX = 0.0         # 배 중심 x (방 한가운데)
SHIP_CY = 0.0         # 배 중심 y

RESOLUTION = 0.05     # 맵 해상도 (m/셀) — 실물 맵과 동일해야 파라미터가 이식됨

# 픽셀 값 (ROS 표준)
FREE, OCCUPIED, UNKNOWN = 254, 0, 205

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
NAME = 'demo_room'

hx, hy = ROOM_X / 2.0, ROOM_Y / 2.0          # 내부 반폭
wx, wy = hx + WALL_THICK, hy + WALL_THICK    # 벽 바깥면
shx, shy = SHIP_X / 2.0, SHIP_Y / 2.0


# ======================================================================
# 1. Gazebo 월드 (SDF)
# ======================================================================
def wall_link(name, x, y, sx, sy):
    """정적 벽 하나. Gazebo Classic SDF의 link 블록."""
    return f"""
      <link name="{name}">
        <pose>{x:.4f} {y:.4f} {WALL_HEIGHT / 2:.4f} 0 0 0</pose>
        <collision name="{name}_collision">
          <geometry><box><size>{sx:.4f} {sy:.4f} {WALL_HEIGHT:.4f}</size></box></geometry>
        </collision>
        <visual name="{name}_visual">
          <geometry><box><size>{sx:.4f} {sy:.4f} {WALL_HEIGHT:.4f}</size></box></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material>
        </visual>
      </link>"""


def build_world():
    walls = (
        # 벽 4개. 길이를 wx*2 / wy*2 로 잡아 모서리가 확실히 닫히게 한다
        wall_link('wall_north', 0.0,  hy + WALL_THICK / 2, wx * 2, WALL_THICK) +
        wall_link('wall_south', 0.0, -hy - WALL_THICK / 2, wx * 2, WALL_THICK) +
        wall_link('wall_east',   hx + WALL_THICK / 2, 0.0, WALL_THICK, wy * 2) +
        wall_link('wall_west',  -hx - WALL_THICK / 2, 0.0, WALL_THICK, wy * 2)
    )
    return f"""<?xml version="1.0"?>
<!-- 자동 생성됨: scripts/make_demo_world.py — 직접 편집하지 말 것 -->
<!-- 방 내부 {ROOM_X} x {ROOM_Y} m, 중앙에 모형배 {SHIP_X} x {SHIP_Y} m -->
<sdf version="1.6">
  <world name="{NAME}">

    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>

    <!-- 실시간 배속 1.0 기준. 물리가 불안정하면 max_step_size를 줄일 것.
         ★ solver 설정을 명시하는 이유 (2026-08-07 실측으로 추가)
           이 로봇은 무게중심이 0.233 m로 트랙 폭 0.2345 m와 맞먹어 매우
           뒤뚱거리는 형상이다. ODE 기본 솔버(iters 50)로는 접촉이 수렴하지
           않아 정지 상태에서도 좌우로 흔들리고(angular.x ≈ 0.024 rad/s),
           그 진동이 로봇을 초당 3 mm씩 옆으로 밀어냈다.
           iters를 올리고 접촉 파라미터(cfm/erp)를 완화해 안정화한다. -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <ode>
        <solver>
          <type>quick</type>
          <iters>150</iters>          <!-- 기본 50 -> 150. 접촉 수렴 개선 -->
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0.00001</cfm>          <!-- 접촉을 아주 약간 무르게 -> 튕김 감소 -->
          <erp>0.2</erp>
          <contact_max_correcting_vel>100</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <!-- 벽: static이라 물리 연산 대상이 아니고 충돌체로만 동작 -->
    <model name="room_walls">
      <static>true</static>{walls}
    </model>

    <!-- 모형배(레고). 실물에서는 이 자리에 배가 놓이고 로봇이 주위를 돈다.
         라이다 높이(0.2m)보다 높아야 스캔에 잡힌다. -->
    <model name="ship_block">
      <static>true</static>
      <link name="ship_link">
        <pose>{SHIP_CX:.4f} {SHIP_CY:.4f} {SHIP_Z / 2:.4f} 0 0 0</pose>
        <collision name="ship_collision">
          <geometry><box><size>{SHIP_X:.4f} {SHIP_Y:.4f} {SHIP_Z:.4f}</size></box></geometry>
        </collision>
        <visual name="ship_visual">
          <geometry><box><size>{SHIP_X:.4f} {SHIP_Y:.4f} {SHIP_Z:.4f}</size></box></geometry>
          <material><ambient>0.8 0.2 0.2 1</ambient><diffuse>0.9 0.3 0.3 1</diffuse></material>
        </visual>
      </link>
    </model>

    <gui>
      <camera name="user_camera">
        <pose>0 -4.5 4.0 0 0.75 1.5708</pose>
      </camera>
    </gui>

  </world>
</sdf>
"""


# ======================================================================
# 2. Nav2 맵 (.pgm + .yaml) — 위 월드와 정확히 같은 기하로 래스터화
# ======================================================================
def build_map():
    # 이미지가 벽 바깥까지 한 칸 여유를 두고 덮도록
    pad = WALL_THICK
    min_x, max_x = -(wx + pad), (wx + pad)
    min_y, max_y = -(wy + pad), (wy + pad)
    w = int(round((max_x - min_x) / RESOLUTION))
    h = int(round((max_y - min_y) / RESOLUTION))

    px = bytearray(w * h)
    for row in range(h):
        # 이미지 row 0 = 맵 y 최대 (pgm은 위에서 아래로 저장)
        y = max_y - (row + 0.5) * RESOLUTION
        for col in range(w):
            x = min_x + (col + 0.5) * RESOLUTION
            ax, ay = abs(x), abs(y)

            if ax <= hx and ay <= hy:
                # 방 내부: 배가 있으면 점유, 아니면 자유
                inside_ship = (abs(x - SHIP_CX) <= shx and abs(y - SHIP_CY) <= shy)
                v = OCCUPIED if inside_ship else FREE
            elif ax <= wx and ay <= wy:
                v = OCCUPIED            # 벽
            else:
                v = UNKNOWN             # 방 바깥은 미탐사 (실제 SLAM 맵과 동일하게)
            px[row * w + col] = v

    pgm = b'P5\n' + f'{w} {h}\n255\n'.encode() + bytes(px)

    yaml = f"""image: {NAME}.pgm
mode: trinary
resolution: {RESOLUTION}
origin: [{min_x:.3f}, {min_y:.3f}, 0.0]
negate: 0
occupied_thresh: 0.65
# ★ 0.196 (map_saver_cli 기본값 0.25 아님).
#   0.25면 미탐사 픽셀(205, p=0.19608)이 자유공간으로 분류되어
#   Nav2가 벽 바깥으로 경로를 뽑는다.
free_thresh: 0.196
"""
    return pgm, yaml, w, h, min_x, min_y


# ======================================================================
def main():
    os.makedirs(os.path.join(PKG, 'worlds'), exist_ok=True)
    os.makedirs(os.path.join(PKG, 'maps'), exist_ok=True)

    world_path = os.path.join(PKG, 'worlds', f'{NAME}.world')
    with open(world_path, 'w') as f:
        f.write(build_world())

    pgm, yaml, w, h, ox, oy = build_map()
    with open(os.path.join(PKG, 'maps', f'{NAME}.pgm'), 'wb') as f:
        f.write(pgm)
    with open(os.path.join(PKG, 'maps', f'{NAME}.yaml'), 'w') as f:
        f.write(yaml)

    # ---- 순찰 반지름 계산 (차체 실측 footprint 기준) ----
    # base_link 기준 차체: 앞 +0.332, 뒤 -0.069, 좌우 +-0.089
    front, half_w, margin = 0.332, 0.089, 0.15
    ship_reach = max(shx, shy)                       # 배 중심에서 배 끝까지
    r_min = ship_reach + half_w + margin             # 안쪽이 배에 닿지 않을 최소
    wall_reach = min(hx, hy)                         # 배 중심에서 벽까지
    # 바깥쪽 스침 반경 = sqrt((R+half_w)^2 + front^2) 가 (벽까지 - margin) 이하
    lim = wall_reach - margin
    r_max = (max(0.0, lim ** 2 - front ** 2)) ** 0.5 - half_w

    print(f'생성 완료')
    print(f'  {world_path}')
    print(f'  {os.path.join(PKG, "maps", NAME)}.pgm / .yaml  ({w}x{h}셀, origin [{ox:.3f}, {oy:.3f}])')
    print()
    print(f'방 내부 {ROOM_X} x {ROOM_Y} m / 배 {SHIP_X} x {SHIP_Y} m @ ({SHIP_CX}, {SHIP_CY})')
    print(f'순찰 반지름 허용 범위: {r_min:.2f} ~ {r_max:.2f} m  (안팎 여유 {margin}m 확보 기준)')
    if r_max <= r_min:
        print('  !! 공간 부족: 방을 넓히거나 배를 줄여야 함')
    else:
        print(f'  권장값: {(r_min + r_max) / 2:.2f} m')


if __name__ == '__main__':
    main()
