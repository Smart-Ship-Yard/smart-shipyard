# Nav2 작업 정리 — 결정사항 · 미해결 이슈 · 진행 계획

작성: 2026-08-05 · 담당: Nav2(정기) · 브랜치 `sim/nav2-gazebo`

이 문서는 Nav2 작업 착수 전에 정리한 **구조적 결정 + 팀 협의 필요 사항 + 작업 순서**다.
슬램/비전/백엔드 담당자와 공유해서 인터페이스 합의용으로 쓴다.

---

## 0. 한 줄 요약

Nav2 작업의 산출물은 URDF 수정이 아니라 **`nav2_params.yaml` + launch 파일 + 순찰 노드**다.
슬램 담당자 파일(`ship_ugv_localization`)은 **한 글자도 건드리지 않는다.**
시연장이 어디일지 모르므로 **"이 방에 맞춘 Nav2"가 아니라 "맵이 바뀌어도 도는 Nav2"** 를 만든다.

---

## 1. 확정된 결정사항

### 1-1. TF 발행 주체 — Option A 채택 (슬램 파일 불가침)

**배경.** 로봇의 각 부품 좌표계 간 변환(TF)은 **발행자가 반드시 하나**여야 한다.
같은 변환을 둘이 쏘면 `TF_REPEATED_DATA` 경고 + 센서 데이터 떨림이 발생한다.

같은 일을 하는 두 도구가 있다:

| 도구 | 하는 일 | 현재 쓰는 곳 |
|---|---|---|
| `static_transform_publisher` | 변환 **1개**를 고정값으로 발행. 한 줄에 하나 | 실물 `localization.launch.py` (`base_link→imu`, `base_link→laser`) |
| `robot_state_publisher` | **URDF를 읽어** 그 안의 모든 조인트를 한꺼번에 발행 | 시뮬 / `display.launch.py` |

**문제는 "지금 고장남"이 아니라 "나중에 합칠 때 충돌"이다.** 실물에서 static TF가 이미
`base_link→laser`를 쏘고 있는데 Nav2 launch가 `robot_state_publisher`를 추가로 켜면 이중 발행이 된다.

**결정: Option A — Nav2 launch에서 `robot_state_publisher`를 켜지 않는다.**

근거는 **Nav2가 URDF를 전혀 요구하지 않기 때문**이다. Nav2에 필요한 4가지는 전부 이미 존재한다:

| Nav2 요구사항 | 제공 주체 | 상태 |
|---|---|---|
| `map→odom→base_link` TF | `ekf_global` / `ekf_local` | ✅ 존재 |
| `base_link→laser` TF | `static_transform_publisher` | ✅ 존재 (PR #14에 실측값 반영) |
| `/scan` | rplidar + laser_filters (`/scan_filtered`) | ✅ 존재 |
| `/cmd_vel` 구독자 | `wheel_odom_bridge` | ⚠️ **PR #14 머지 필요** |

로봇 크기는 URDF가 아니라 `nav2_params.yaml`의 `footprint`에 숫자로 직접 적는다.

**실행 방법 (터미널 2개):**
```bash
터미널1: ros2 launch ship_ugv_localization localization.launch.py   # 슬램 담당자 파일, 그대로
터미널2: ros2 launch ship_ugv_navigation navigation.launch.py       # Nav2 담당자 파일
```

**유일한 손해:** RViz에서 로봇 3D 모델이 안 보이고 좌표축만 보인다. 주행에는 영향 없음.

**나중에(선택):** PR #14 머지 후 여유가 생기면, `localization.launch.py`의 static TF 2개를
`robot_state_publisher` 하나로 교체하는 게 더 표준적인 구조다. launch에
`use_robot_state_publisher` 인자를 기본 false로 넣어두어 나중에 스위치만 켜면 되게 한다.

### 1-2. 지도 파일 위치 이동 (완료)

```
edge/docs/maps/                             (이전)
  → edge/ros2_ws/src/ship_ugv_navigation/maps/   (현재)
```

**이유:** ROS2 launch의 표준 경로 조회(`get_package_share_directory`)는 **패키지 안에 있고
install 규칙에 등록된 파일만** 찾는다. `docs/`에 있으면 절대경로를 하드코딩해야 하는데,
노트북(`/home/lee/`)과 젯슨(`/home/jetson/`)의 홈 경로가 달라 그대로 깨진다.

- 슬램 담당자 확인 완료: 위치 이동 무관
- 원 커밋 `cbc996e`는 Nav2 담당자 본인 커밋 — 슬램 산출물 아님
- PR #14는 `edge/docs/`의 설치가이드 md만 수정, `maps/`는 미접촉 → **머지 충돌 없음**
- `git mv` 사용 → git이 rename으로 추적, 히스토리 유지

### 1-3. install 규칙이란 (setup.py)

ROS2는 폴더가 두 벌이다.

```
src/ship_ugv_navigation/maps/맵.yaml                              ← 편집하는 원본
      ↓  colcon build 가 복사
install/ship_ugv_navigation/share/ship_ugv_navigation/maps/맵.yaml ← 실행 시 읽는 것
```

`colcon build`는 **파이썬 코드만 자동 복사**하고 yaml/pgm/launch/world는 복사하지 않는다.
`setup.py`의 `data_files`가 "이것도 복사해라"라는 지시다.

```python
data_files=[
    (os.path.join('share', package_name, 'maps'), glob('maps/*')),   # (설치 위치, 원본 파일들)
]
```

빠뜨리면 **빌드는 성공하는데 실행할 때 "파일 없음"** 이 난다. ROS2 초보 함정 1위.

### 1-4. 이벤트 정지/재개는 순찰 완성 후에 붙인다 (Step 6 → Step 7)

**Step 6까지:** 웨이포인트 순찰 + 무한 반복 주행.
**Step 7:** 이벤트 감지 시 정지 / 관제 확인 시 재개.

**근거:** "경로 돌기"가 되면 "멈추기"는 쉽게 붙지만 반대는 성립하지 않는다.
또한 재개 신호가 프론트·백엔드·비전 세 팀에 걸쳐 있어 순찰 완성과 병렬로
진행하는 편이 일정상 유리하다.

**Step 6 순찰 노드는 `/event/active` (Bool) 구독을 미리 뚫어둔다.**
그 값을 누가 만드는지는 순찰 노드가 알 필요가 없으므로, Step 7이 늦어져도
Step 6은 독립적으로 완성·테스트할 수 있다.

> ⚠️ 이전 판단(`websocket_client.py`에 수신 코드가 없어 2단계로 미룸)은
> 설계 변경으로 해소됐다. 상세는 **4장**과 **Step 7** 참조.

---

## 2. 시연장 대응 전략 (핵심)

**전제:** 시연장 크기·형태 미정(방 정도 크기 예상). 시연 당일 Gazebo 튜닝 시간 없음.
최종 발표라 "바로 실행되는지"만 검사.

**따라서 맵이 바뀌어도 재튜닝 없이 도는 구조로 만든다.**

| ❌ 시연장에서 망하는 방식 | ✅ 채택 방식 |
|---|---|
| 웨이포인트 좌표를 코드에 하드코딩 | **배 중심(x,y) + 반지름 + 개수** 파라미터로 원을 자동 생성 → 시연장에선 숫자 3개만 변경 |
| 맵 경로를 코드에 박음 | launch 인자로 받음 (`map:=새맵.yaml`) |
| 이 방 크기에 맞춘 코스트맵 | 방 크기 무관한 상대값 (로컬 코스트맵은 로봇 주변만 관찰) |
| world 파일을 시연장에 맞춰 재제작 | **world는 시뮬 튜닝 전용. 시연장에서는 Gazebo를 켜지 않음** |

**시연장 당일 절차 (목표):**
1. 슬램 담당자가 매핑 → 맵 저장
2. `map:=` 인자로 새 맵 지정
3. 배 중심 좌표 + 반지름 파라미터 3개 입력
4. 실행

**좁은 방은 오히려 유리하다.** 좁은 공간 기준으로 맞춘 파라미터는 넓은 곳에서도 돌지만,
반대는 안 된다.

---

## 3. 미해결 이슈 / 팀 협의 필요

### ✅ 해결됨 1. 저장된 지도가 `slam_map` 좌표계로 저장되는 문제

**증상.** `slam_toolbox`의 `map_frame`이 `slam_map`(프로젝트 규칙)이고
`map_saver_cli`는 TF를 적용하지 않고 `/map` 토픽 값을 그대로 파일에 쓴다.
따라서 저장된 맵은 `slam_map` 좌표계이며, `map` 좌표계로 도는 Nav2·EKF와 어긋난다.
RViz가 정상으로 보이는 것은 RViz가 TF를 실시간 적용하기 때문이고 파일엔 반영되지 않는다.

**실측 확인 (2026-08-06, shipyard_map_v2).** `tf2_echo map slam_map` =
평행이동 0.94 m + **회전 138.19°**. 보정 없이 썼다면 사용 불가였다.

**해결.** 맵 yaml `origin`의 3번째 값이 yaw이므로 이미지 재렌더링 없이 숫자만 고치면 된다.
`scripts/bake_map_origin.py`가 `align_*.json`을 읽어 자동 처리하며,
`scripts/finalize_map.py`가 기록 보존·`free_thresh` 교정·순찰 검사까지 한 번에 수행한다.
절차는 [재매핑 체크리스트](재매핑_체크리스트.md) 9단계.

**책임 소재.** 슬램 담당자의 코드 버그가 아니라 "저장 시 TF를 굽는 단계"가
운영 절차에서 빠져 있던 것이다. 맵 파일의 유일한 소비자가 Nav2이므로 Nav2 담당이 처리한다.

> **참고 — 실무와의 차이.** 일반적으로는 저장된 SLAM 맵이 `map` 프레임을 정의하고
> UWB는 EKF에 들어가는 센서 하나로 쓴다. 이 프로젝트는 반대로 UWB 캘리브레이션이
> `map`을 정의하므로 **저장된 맵이 세션에 종속**된다. "앵커가 매일 재배치된다"는
> 전제에서는 합리적인 선택이며 설계 오류가 아니다.

### ✅ 해결됨 2. 맵 `free_thresh` 값

`map_saver_cli`는 항상 `0.25`로 저장하는데, 미탐사 픽셀(205)의 점유확률이
**0.19608**이라 `0.25`면 **미탐사 영역이 자유공간으로 분류되어** Nav2가
매핑 안 된 곳으로 경로를 뽑는다. `0.196`으로 고쳐야 한다.

**튜닝값이 아니라 상수다.** 205라는 PGM 인코딩에서 나오는 값이라 방 크기와 무관하게 항상 같다.
`finalize_map.py`가 자동 교정하므로 수동 조치 불필요.

### ✅ 해결됨 3. 순찰 가능한 맵 확보 (v2 재매핑)

v1(JG방)은 **어떤 반지름으로도 완주 불가**였다. 원인 분석 결과 미탐사 때문이 아니라
**실제 벽·가구** 때문이었다(미탐사를 전부 빈 바닥으로 가정해도 결과 동일).
→ 매핑을 다시 하는 것으로는 해결되지 않고 바닥을 치워야 했다.

폼롤러(⌀12.7 세움)로 중앙 물체를 바꾸고 재매핑한 `shipyard_map_v2`로 해결.

| 항목 | 값 |
|---|---|
| 맵 크기 | 69 × 46 셀 = 3.45 × 2.30 m |
| 탐사 자유공간 | 4.09 m² (52%) — v1의 33%에서 개선 |
| **순찰 중심** | **map (3.46, 0.54)** |
| **순찰 반지름** | **0.30 m** (가용 0.25~0.40) |
| 여유 | 0.075 ~ 0.125 m, 원 전체에 균일 |
| 정합 품질 | 대응점 65개, inlier 60% |

여유가 균일한 것은 안쪽(롤러)과 바깥쪽(벽)이 동시에 조이기 때문이다. 한 지점만
치워서는 개선되지 않는다. 0.075 m는 좁지만 사용 가능 — 로컬 코스트맵이 라이다로
실시간 회피하므로 UWB 오차(±15 cm)와 무관하게 충돌하지 않는다.
`inflation_radius`를 0.10 m로 낮출 것.

#### 시연장 재매핑 요구 사양

| 중앙 물체 | 물체 반경 | 권장 반지름 | **치워야 할 공터** |
|---|---|---|---|
| 폼롤러 ⌀12.7 (세움) | 0.090 m | 0.28 m | **1.19 × 1.19 m** |
| 레고 배 0.35 × 0.40 | 0.266 m | 0.45 m | **1.47 × 1.47 m** |

⚠️ **물체는 반드시 세워서 라이다 높이(0.20 m)에 걸치게 한다.** 눕히면 스캔에 안 잡혀
맵에도 안 찍히고 주행 중에도 안 보인다. 높이가 0.20 m에 못 미치면 받침대를 쓰되
**받침은 물체보다 작아야 한다**(받침이 더 넓으면 안 보이는 부분에 부딪힌다).

⚠️ **중앙 물체 주위를 완전한 한 바퀴 이상 돌며 매핑할 것.** 자유공간이 고리를
이루지 않으면 순찰 경로가 아예 생성되지 않는다(v1 실패의 직접 원인).

검증: `python3 scripts/check_patrol_space.py --map maps/<맵>.yaml`

### 🔴 이슈 2. UWB 캘리브레이션 원점 재현성

캘리브레이션 시작 지점이 `map` 원점 (0,0), 직진 방향이 `+x`축이 된다.
**저장된 맵을 다음 세션에 재사용하려면 같은 자리·같은 방향에서 캘리브레이션해야 한다.**
방향이 5°만 틀어져도 2.5 m 떨어진 지점에서 22 cm 어긋난다.
→ 시작점과 1.5 m 끝점 **두 곳을 바닥에 테이프로 표시**한다. 앵커도 옮기지 않는다.

### 🟡 이슈 3. PR #14 머지 필요

`wheel_odom_bridge`(`/cmd_vel` 소비자), `laser_filter`, 실측 static TF가 전부
PR #14에 있다. **머지 전에는 실물에서 Nav2가 로봇을 움직일 수 없다.**
머지 후 이 브랜치를 main에 rebase한다.

### 🔴 이슈 4. 백엔드 확인(ack) 경로 신설 — Step 7 선결 조건

**2026-08-07 결정으로 필수 작업이 됐다.** 이벤트 재개를 관제 확인 버튼으로 하기로
했으므로 **서버 → 젯슨 경로가 반드시 필요하다.** 현재 `docs/interface.md`의
서버→젯슨 메시지는 `stream_boost` 하나뿐이고, 젯슨 쪽 `websocket_client.py`에는
수신 코드(`ws.recv()`) 자체가 없다.

**방침: 기존 `/ws/jetson` 중계 경로를 재사용한다. 새 엔드포인트를 만들지 않는다.**
프론트→젯슨 중계 코드는 [backend/main.py:318](../../backend/main.py#L318)에
이미 동작 중이므로 백엔드 수정은 상수 2줄이면 끝난다.

| 구간 | 할 일 | 담당 |
|---|---|---|
| 프론트 | 이벤트 팝업에 "확인" 버튼 → 기존 `/ws/frontend` 소켓으로 ack 전송 | 프론트 담당 |
| 백엔드 | `JETSON_BOUND_TYPES`에 `event_ack` 추가 (상수 2줄) | **본인** |
| 비전 | `websocket_client.py`에 수신 루프 → `/server/inbound`로 발행 (~10줄) | 비전 담당 |
| 젯슨 | `event_gate_node`가 `/server/inbound` 구독 | **본인** |

```json
{"event_type": "event_ack"}
```

**프론트엔드도 새 엔드포인트가 불필요하다.** `/ws/frontend`는 이미 양방향이며
`webrtc_signal`이 그 경로로 다니고 있다.

> 초기에는 `/ws/nav` 엔드포인트 신설을 검토했으나, 프론트→젯슨 중계 경로가 이미
> 존재해 백엔드 작업량이 오히려 적고(엔드포인트 수십 줄 → 상수 2줄),
> 단일 게이트웨이가 정석 아키텍처이므로 철회했다.
> 상세 근거는 **Step 7**의 "확인(ack) 신호 경로" 참조.
`docs/interface.md`에 서버→Nav2 메시지로 기재할 것.

---

## 4. 이벤트 정지/재개 설계 (2026-08-07 확정)

> **정지는 로봇 혼자서 즉시. 재개는 관제에서 사람이 확인 버튼으로.**

- **정지를 로컬로 하는 이유:** 감지→서버→판단→명령 왕복은 지연이 붙고, 와이파이가 끊기면
  로봇이 영영 멈추지 않는다. 안전 로직은 무조건 로컬.
- **재개를 사람이 하는 이유:** 화재 진화에는 오래 걸리는데 그동안 로봇이 묶여 있으면
  다른 문제 상황을 놓친다. 관제에서 **"확인했으니 계속 순찰하라"**를 눌러
  로봇을 풀어준다. 안전 판단을 사람이 한다는 점도 실제 운영 방식에 부합한다.
- **따라서 버튼의 의미는 "해결"이 아니라 "확인(ack)"이다.** 프론트 버튼 라벨도
  "처리 완료"가 아니라 **"확인"**으로 한다.

```
[YOLO 감지] ──/event_detection/uvd──> [event_gate_node]  ← 즉시 정지 (로컬, 지연 0)
                    │                        │
                    └─> websocket_client ──> 백엔드 ──> 프론트 (알림 팝업)
                            │                  │                    │
                            │  /server/inbound │   event_ack        │
[event_gate_node] <─────────┘ <────────────────┘ <── 관리자 "확인" 클릭
     └─> /event/active = false ──> 주행 재개
```

**정지 방법 선택:**

| 방법 | 평가 |
|---|---|
| `/cmd_vel`에 0 덮어쓰기 | ❌ Nav2가 "진행 없음"으로 판단 → **복구 행동(제자리 회전/후진) 발동.** 좁은 방에서 위험 |
| lifecycle deactivate | ❌ 과함, 재개가 느림 |
| **`cancelTask()` + 0속도 발행** | ✅ 채택. `nav2_simple_commander` 표준 방법 |

**기존 3중 안전장치 활용:** `wheel_odom_bridge` cmd_vel 0.5초 타임아웃,
아두이노 펌웨어 500ms 워치독 → 순찰 노드가 죽어도 0.5초 안에 정지.

**시연 폴백 (필수 준비):** 백엔드 연동 실패 시 수동 재개 가능하게.
```bash
ros2 topic pub --once /event/ack std_msgs/msg/Empty "{}"
```
또는 `fallback_auto_resume_s`를 켜서 "N초 후 자동 재개"로 전환한다.

---

## 5. 로봇/맵 실측 데이터 (파라미터 근거)

### 맵
| 항목 | 값 |
|---|---|
| 크기 | 71 × 68 셀 × 0.05 m = **3.55 m × 3.40 m** |
| resolution | 0.05 m/셀 |
| origin | `[0.814, -1.14, 0]` |

### 로봇 (URDF 실측값)
| 항목 | 값 |
|---|---|
| 차체 | 0.401 m (길이, 전선 돌출부 포함) × 0.178 m (폭) |
| `base_link` 위치 | **차체 중심이 아니라 뒷바퀴 축** |
| base_link 기준 전방 | +0.332 m |
| base_link 기준 후방 | −0.069 m |
| base_link 기준 좌우 | ±0.089 m |

**⚠️ 원형 근사(`robot_radius`) 금지.** base_link가 뒷바퀴 축이라 외접원 반경이 **0.344 m**가
나와 3.5 m 방에서 과보호가 심각하다. **반드시 다각형 footprint를 쓴다:**

```yaml
footprint: "[[0.332, 0.089], [0.332, -0.089], [-0.069, -0.089], [-0.069, 0.089]]"
```

### Gazebo diff_drive 파라미터 (⚠️ URDF 기하값과 다름 — 의도적)
| 항목 | 시뮬에 쓸 값 | URDF 기하값 (쓰면 안 됨) |
|---|---|---|
| wheel_radius | **0.0308** | 0.0335 |
| wheel_separation | **0.22568** | 0.2345 |

PR #14 `wheel_odom_node`의 실주행 보정값을 써야 실물과 거동이 일치한다.
추가로 **`publish_odom_tf: false` 필수** (EKF 단일 TF 권위자 원칙).

---

## 6. 파라미터 선택 (좁은 방 + 원형 순회)

### ⚠️ 기숙사 방 기준으로 튜닝하지 말 것 — 기본값은 시연장(넓은 공간) 기준

좁은 방 설정을 넓은 공간에 그대로 가져가면 **더 좋아지는 것이 아니라 나빠진다.**

| 좁은 방 설정 | 넓은 공간에서의 문제 |
|---|---|
| `inflation_radius: 0.10` | 벽에 10 cm까지 붙어 다닌다. 위험하고 불안해 보인다 |
| `local_costmap 2×2 m` | 시야가 좁아 먼 장애물을 늦게 인지한다 |
| 복구 행동 `spin` 비활성화 | 끼었을 때 빠져나올 수단이 없다 |

**파라미터를 두 부류로 나누어 관리한다:**

| 로봇이 결정 (장소 무관, 한 번 정하면 끝) | 장소가 결정 (현장에서 조정) |
|---|---|
| `footprint` (차체 실측) | `inflation_radius` |
| `max_vel_x`, `max_vel_theta` | `local_costmap` 크기 |
| 가속도 한계 | 복구 행동 on/off |
| 컨트롤러 게인 | 순찰 중심·반지름 |

**구현 방침: `nav2_params.yaml` 맨 위에 "장소 의존" 블록을 몰아넣고
두 프리셋을 주석으로 병기한다.**

```yaml
# ===== 장소 의존 (여기만 바꾸면 됨) =====
# 좁은 방(기숙사)   : inflation 0.10 / costmap 2x2 / spin off
# 넓은 공간(시연장) : inflation 0.25 / costmap 5x5 / spin on
```

**기본값은 넓은 공간 기준으로 둔다.** 시연이 거기서 이뤄지기 때문이다.
파일을 둘로 나누지 않는 이유는 **발표 당일 엉뚱한 파일을 로드하는 사고를 막기
위해서**다. 한 파일 안에서 몇 줄만 바꾸는 편이 안전하다.

**회피 시연 계획 (2026-08-07 확정):** 배에서 직선으로 뻗은 방향(공간 여유가 있는
구간)에만 작은 장애물을 하나 두고 "웨이포인트가 원래 여기인데 장애물 때문에
우회합니다"를 보여준다. 나머지 구간은 공간이 없어 회피 시연을 하지 않는다.


| 항목 | 선택 | 이유 |
|---|---|---|
| Controller | **Regulated Pure Pursuit** | 원 궤도 추종이 부드럽고 파라미터가 적음. DWB는 좁은 공간에서 튜닝 난이도 급상승 |
| Planner | **NavFn** | 3.5 m 방에 Smac Hybrid는 과함 |
| `max_vel_x` | 0.15 m/s | `motion_controller`와 동일하게 시작 |
| `max_vel_theta` | 0.6 rad/s | 위와 동일 |
| `inflation_radius` | 0.25~0.3 | 기본 0.55는 이 방에서 통로를 통째로 막음 |
| local_costmap 크기 | 2 × 2 m | 기본 5 × 5는 맵보다 큼 |
| 복구 행동 | `spin` 비활성, `backup` 거리 축소 | 좁은 공간에서 제자리 회전은 벽을 긁음 |

### 시뮬 로컬라이제이션 (AMCL 문제)

실물은 AMCL을 쓰지 않고 **EKF가 `map→odom`을 제공**한다. 시뮬에서 Nav2 기본 구성대로
AMCL을 쓰면 구조가 달라져 이식이 어긋난다.

**2단계 접근:**
1. **초반:** AMCL로 빠르게 굴러가게 만들어 planner/controller/costmap 튜닝
   (이 파라미터들은 그대로 이식 가능)
2. **최종 검증:** AMCL을 끄고, Gazebo 참값을 `map→odom` TF로 발행하는
   **가짜 글로벌 로컬라이제이션 노드**로 교체해 실물 구조를 그대로 재현

프로젝트에 이미 `dev_tools/fake_sensor_publisher.py`, `fake_slam_publisher.py` 등
가짜 노드로 스택을 검증하는 패턴이 자리잡혀 있어 같은 방식을 따른다.

---

## 7. 패키지 구조 및 산출물

```
edge/ros2_ws/src/
├── ship_ugv_description/urdf/
│   ├── ship_ugv_core.urdf.xacro      # ← 절대 미수정 (슬램 실측값)
│   └── ship_ugv_gazebo.xacro         # ✅ 완료: <gazebo> 태그, diff_drive, 센서 플러그인
└── ship_ugv_navigation/              # 신규 패키지 (ament_python)
    ├── config/nav2_params.yaml       # ★ 진짜 산출물
    ├── maps/                         # ✅ 완료 (shipyard_map_v2 보정 완료)
    ├── worlds/demo_room.world        # ✅ 완료
    ├── scripts/                      # ✅ 완료 (매핑 후처리 3종 + 월드 생성기)
    ├── launch/
    │   ├── sim_bringup.launch.py     # Gazebo + 스폰 + 가짜 로컬라이제이션
    │   └── navigation.launch.py      # map_server + Nav2 (시뮬/실물 공용, use_sim_time 인자)
    ├── ship_ugv_navigation/
    │   ├── patrol_mission_node.py    # ★ 순찰 순회 (Nav2 클라이언트)
    │   └── event_gate_node.py        # ★ 이벤트 판정 -> /event/active 발행
    └── rviz/nav2.rviz
```

`navigation.launch.py`를 **시뮬/실물 공용**으로 만드는 것이 핵심.
`use_sim_time`과 `scan_topic`만 인자로 빼면 젯슨에서 인자만 바꿔 그대로 실행된다.

**젯슨으로 보낼 것:** `nav2_params.yaml`, `navigation.launch.py`,
`patrol_mission_node.py`, `event_gate_node.py`
**젯슨에 안 보낼 것:** world 파일, `ship_ugv_gazebo.xacro`, `sim_bringup.launch.py`

---

## 7-1. 작업 단계 (진행 상황)

| Step | 산출물 | 상태 |
|---|---|---|
| 1 | `ship_ugv_gazebo.xacro` — 시뮬 전용 물리/센서 | ✅ 완료 |
| 2 | 월드 + 맵 + 매핑 후처리 스크립트 3종 | ✅ 완료 |
| 3 | `sim_bringup.launch.py` — Gazebo 기동 + 로봇 스폰 + TF 트리 | ✅ 완료 |
| **4** | **`nav2_params.yaml`** — footprint·속도·코스트맵·planner·controller | ← 다음 |
| 5 | `navigation.launch.py` — map_server + Nav2 (시뮬/실물 공용) | |
| **6** | **`patrol_mission_node.py`** — 원형 순찰 무한 순회 | |
| **7** | **`event_gate_node.py`** — 이벤트 감지 시 정지 / 관제 확인 시 재개 | |
| 8 | 실물 이식 + 튜닝 | |
| **9** | **문서 최종 정리** — 아래 목록 갱신 | ★ 각 Step 종료 시마다 부분 수행 |

#### Step 3 완료 기록 (2026-08-07) — 시뮬 기동 검증 결과

| 검증 항목 | 결과 |
|---|---|
| Gazebo 기동 + 로봇 스폰 | ✅ |
| TF 트리 `map→odom→base_link→{chassis,wheels,caster,imu,laser}` | ✅ 완성 |
| `fake_global_localization` 계산 정확도 | ✅ 참값 (0.8972, 0.2203) vs TF (0.897, 0.223) 일치 |
| `/scan` 10 Hz · `/imu/data` 100 Hz · `/wheel/odom` 50 Hz | ✅ |
| 전진 정확도 | ✅ 0.15 m/s × 6.2초 → 0.930 m |
| 회전 정확도 | ✅ 명령 0.6 rad/s → 실측 0.611 rad/s |

**해결한 문제 — 정지 상태 드리프트**

초기에는 정지 상태에서도 로봇이 초당 3 mm씩 옆으로 밀렸다. 원인은 이 로봇의
**무게중심 0.233 m가 트랙 폭 0.2345 m와 맞먹는 뒤뚱한 형상**이라 접촉이 수렴하지
않은 것이었다. 세 가지를 조정해 1.6 mm/s로 줄였다.

| 조치 | 값 | 파일 |
|---|---|---|
| ODE 솔버 반복 | 50 → 150, cfm/erp 완화 | `make_demo_world.py` |
| 캐스터 마찰 | 0.0 → 0.05 (실물이 **고무 바퀴**임을 반영) | `ship_ugv_gazebo.xacro` |
| 접촉 강성 | kp 1e6 → 1e5, kd 100 → 1000 (바퀴·캐스터 공통) | 〃 |

**남은 1.6 mm/s는 그대로 둔다.** 주행 속도 150 mm/s의 1 %라 실제 주행 중에는
무의미하고, 전진 0.930 m·회전 0.611 rad/s 정확도로 영향이 없음을 확인했다.
Nav2는 매 주기 맵 기준으로 위치를 보정하므로 누적되지도 않는다.

**⚠️ 실행 환경 주의 — 백엔드 venv**

셸에 `backend/venv`가 활성화돼 있으면 시스템 `numpy`가 가려져
`spawn_entity.py`가 `ModuleNotFoundError`로 죽는다. ROS를 쓸 때는 venv를 벗어날 것.

```bash
deactivate        # 또는 새 터미널에서 venv 없이 시작
```

---

#### Step 9: 문서 갱신 체크리스트 (매 Step 종료 시 확인)

Step을 하나 끝낼 때마다 아래를 훑는다. 마지막에 몰아서 하면 반드시 빠진다.

| 문서 | 갱신할 내용 |
|---|---|
| [전체_실행_명령어_요약본](전체_실행_명령어_요약본%20—%20매핑부터_자율주행까지.md) | **새로 생긴 launch·노드의 실행 명령 추가**, 상단 "진행 상황" 줄 수정 |
| [nav2_작업_정리.md](nav2_작업_정리.md) (이 문서) | 7-1 단계표 상태 갱신, 결정이 바뀌면 해당 절 수정·폐기분 정리 |
| [재매핑_체크리스트.md](재매핑_체크리스트.md) | 매핑 절차에 영향이 있을 때만 |
| `docs/interface.md` | 팀 간 메시지 스펙이 바뀔 때만 |

**특히 `전체_실행_명령어_요약본`을 빠뜨리기 쉽다.** Step 5(navigation.launch.py),
Step 6·7(순찰·이벤트 노드)이 완성되면 3부 명령어가 실제로 동작하게 되므로
상단 경고 문구를 지우고 확인 명령을 채워야 한다.

### 7-2. 노드 인터페이스 요약 (Step 6·7 구현 기준)

두 노드가 주고받는 것 전부. **이 표대로만 만들면 다른 팀 작업과 자동으로 맞물린다.**

**`event_gate_node.py`** (Step 7)

| 방향 | 토픽 | 타입 | 내용 |
|---|---|---|---|
| 구독 | `/event_detection/uvd` | `std_msgs/String` | 욜로 감지 결과(JSON). 위험 클래스면 정지 |
| 구독 | `/server/inbound` | `std_msgs/String` | 서버 수신분(JSON). `event_ack`면 재개 |
| 구독 | `/event/ack` | `std_msgs/Empty` | 수동 재개(시연 폴백·디버깅용) |
| 발행 | `/event/active` | `std_msgs/Bool` | true=정지, false=주행 |

**`patrol_mission_node.py`** (Step 6)

| 방향 | 대상 | 타입 | 내용 |
|---|---|---|---|
| 구독 | `/event/active` | `std_msgs/Bool` | true면 `cancelTask()`, false면 순찰 재개 |
| 액션 | `navigate_to_pose` | Nav2 | `BasicNavigator.goToPose()` 로 호출 |
| 발행 | `/cmd_vel` | `geometry_msgs/Twist` | 정지 시 0속도 (Nav2와 별개로 확실히 멈추기) |
| 발행 | `/patrol/status` | `std_msgs/String` | 현재 상태·웨이포인트 번호 (디버깅/발표용) |

**경계 원칙:** `patrol_mission_node`는 `/event/active`가 **어디서 왔는지 모른다.**
욜로든 관제 버튼이든 수동 명령이든 상관없이 true/false만 본다.
그래서 Step 6은 Step 7 없이도 완성·테스트할 수 있다.

### 7-3. Step 7 동작 확인 방법

**Step 6까지 끝나면, 팀원 작업을 기다리지 않고도 단계별로 확인할 수 있다.**

| 단계 | 확인 방법 | 필요한 것 |
|---|---|---|
| ① 정지/재개 로직 | `ros2 topic pub /event/active std_msgs/msg/Bool "{data: true}"` → 로봇 정지 확인, `false` → 재개 확인 | **우리만** |
| ② 이벤트 감지 → 정지 | `/event_detection/uvd`에 가짜 감지 JSON을 pub → 정지 확인 | **우리만** |
| ③ 수동 재개 | `ros2 topic pub --once /event/ack std_msgs/msg/Empty "{}"` → 재개 확인 | **우리만** |
| ④ 서버 경로 재개 | `/server/inbound`에 `{"event_type":"event_ack"}` pub → 재개 확인 | **우리만** |
| ⑤ 전체 연동 | 실제 이벤트 감지 → 팝업 → "확인" 클릭 → 재개 | 프론트 + 젯슨 작업 완료 필요 |

**①~④는 팀원 작업과 무관하게 전부 검증 가능하다.** 그들의 작업이 끝나면 ⑤로
한 번만 이어보면 되고, 이때 문제가 생겨도 어느 구간인지 즉시 좁혀진다.

**준비 상태 (2026-08-07):**

| 구간 | 상태 |
|---|---|
| 백엔드 중계 (`event_ack`) | ✅ 완료 (커밋 `b275609`) |
| `docs/interface.md` ⑧ 스펙 | ✅ 완료 (v1.4) |
| 프론트 "확인" 버튼 | ⏳ 요청 전달됨 |
| 젯슨 수신 루프 | ⏳ 요청 전달됨 |
| `event_gate_node` / `patrol_mission_node` | ⏳ Step 6·7에서 작성 |

### Step 6: `patrol_mission_node.py` — 왜 필요한가

**Nav2는 "실행하면 돌아다니는 프로그램"이 아니다.** "여기로 가"라고 지시하면
거기까지 가고 멈추는 엔진이며, 순찰·반복·웨이포인트 순회 개념이 없다.
목표를 하나씩 계속 주는 주체가 반드시 따로 있어야 한다.

```
patrol_mission_node  ──goToPose()──>  Nav2 스택  ──/cmd_vel──>  wheel_odom_bridge
   (운전자)                            (자동차)
```

- 원 위의 웨이포인트 N개(12개 권장)를 순서대로 `goToPose()`
- 마지막까지 가면 처음으로 되돌아가 무한 순회
- `/event/active` 를 구독해 `cancelTask()` / 재개
- 구현: `nav2_simple_commander`의 `BasicNavigator`
  (`goToPose` / `isTaskComplete` / `cancelTask`)

주요 파라미터 (검사 스크립트가 출력하는 값을 그대로 사용):
```yaml
center_x: 3.457      # check_patrol_space.py 출력
center_y: 0.543
radius:   0.30
num_waypoints: 12    # 12개면 경로가 중심에서 0.97R 이상 떨어져 배를 안 스침
direction: cw        # 시계방향 = 카메라(오른쪽 90도)가 배를 향함

goal_retry_count: 2          # 목표 실패 시 재시도 횟수
on_goal_fail: skip           # skip | wait — 실패 시 다음 웨이포인트로 건너뜀
max_consecutive_fails: 4     # 연속 실패가 이만큼이면 건너뛰기를 멈추고 대기
wait_retry_interval_s: 5.0   # 대기 상태에서 이 간격으로 재시도 (길이 열렸는지 확인)
```

**대기 상태에서 빠져나오는 방법 = 주기적 재시도.** 막혀서 대기로 전환된 뒤에도
`wait_retry_interval_s`마다 같은 목표를 다시 시도한다. 성공하면 자동으로 순찰
재개, 실패하면 다시 대기. **길이 열렸는지 알려주는 별도 신호가 없으므로
"해보고 되면 간다"가 유일한 방법이다.**

간격을 5초로 둔 이유: 1초마다 재시도하면 Nav2가 계속 경로 계획을 돌려
CPU를 낭비하고, 실패 로그가 폭주해 진짜 문제를 못 본다.

> **실제 현장 시나리오:** 조선소 통로가 대형 차량이나 철제 운반으로 막히는 경우.
> 로봇이 제자리 회전 같은 헛짓을 하지 않고 대기하다가, 통로가 비면 5초 안에
> 스스로 원래 경로로 복귀한다. 산업용 AMR의 표준 동작 방식이다.

**`max_consecutive_fails`가 필요한 이유 — 두 가지 실패를 구분해야 한다:**

| | 상황 | 건너뛰기 효과 |
|---|---|---|
| **A** | 웨이포인트 지점만 막힘, 옆으로 지나갈 공간 있음 | ✅ 다음 목표로 가면서 Nav2가 우회 |
| **B** | 통로 자체가 막힘 (좁은 링) | ❌ 다음 목표도 그 사람을 지나야 함 → 또 실패 |

A는 넓은 공간, B는 좁은 방에서 발생한다. B에서 건너뛰기만 반복하면 로봇이
**웨이포인트를 순서대로 계속 실패하며 헛돈다.** 연속 실패가 누적되면
"길이 막혔다"로 판단하고 제자리 대기로 전환한다.

#### 동적 장애물 (사람이 경로를 지나가거나 서 있는 경우)

**회피 자체는 Nav2가 처리한다.** `/scan_filtered` -> local costmap 실시간 갱신 ->
controller가 경로를 수정한다. 우리가 코드를 짤 부분이 아니라 `nav2_params.yaml`
설정 영역이다.

**단, 이 방에서는 "옆으로 돌아가기"가 물리적으로 불가능하다:**

```
롤러 바깥면 0.125 m ────────── 벽 0.60 m      링 폭 = 0.475 m
       └─ 로봇 점유 띠 0.30 m ─┘
남는 여유 0.175 m (안팎으로 쪼개져 각 ~0.09 m)  <  로봇 폭 0.178 m
```

| 상황 | 실제 동작 |
|---|---|
| 사람이 **지나감** (일시적) | 감속/정지 -> 지나가면 자동 재개 ✅ |
| 사람이 **서 있음** (지속) | **멈춰서 대기.** 우회 경로 없음 |

**이는 버그가 아니라 올바른 동작이다.** 좁은 통로에서 비집고 가는 편이 더 위험하다.

**그래서 실제로 구현할 것은 "목표 실패 처리"다.** Nav2가 계속 못 가면 목표 실패를
반환하므로, `patrol_mission_node`가 재시도 후 **다음 웨이포인트로 건너뛴다.**
순찰의 목적은 특정 지점 도달이 아니라 계속 도는 것이므로, 건너뛰어도 문제없고
발표 중 로봇이 한 지점에서 굳어있지 않는다.

**관련 nav2_params 설정:**
- **복구 행동 `spin` 비활성화** — 막히면 기본적으로 제자리 회전을 시도하는데
  이 공간에서는 벽을 긁는다. `backup`도 거리를 축소한다.
- **RPP 컨트롤러의 장애물 근접 감속 활성화** — 사람 근처에서 부드럽게 느려지도록.

⚠️ **후방 180도는 감지되지 않는다.**
[laser_filter.yaml](../ros2_ws/src/ship_ugv_localization/config/laser_filter.yaml)이
뒤쪽 스캔을 제거하고 있다(`lower_angle: -1.1990` ~ `upper_angle: 1.9425`).
전진 순찰이므로 대부분 문제없으나, **뒤에서 다가오는 사람은 보이지 않는다.**
시연 시 인지하고 있을 것.

### Step 7: `event_gate_node.py` — 이벤트 정지/재개

**설계 확정 (2026-08-07): 정지는 젯슨 로컬, 재개는 관제 확인 버튼.**

```
[정지]  yolo_depth_publisher ──/event_detection/uvd──> event_gate_node
                                (엣지 트리거, 객체당 1번)      │
                                                              ├─> /event/active = true
[재개]  프론트 "확인" ──> 백엔드 ──> websocket_client ──/server/inbound──> event_gate_node
                                                              └─> /event/active = false
                                                                        │
                                                              patrol_mission_node
```

#### 왜 "해결"이 아니라 "확인(ack)"인가

슬램·비전 담당자 의견(2026-08-07)이 채택됐다:

> 실제 화재는 진화에 오래 걸리는데 그동안 로봇이 묶여 있으면 안 된다.
> 다른 문제 상황을 즉각 발견하려면 계속 순찰해야 한다.

즉 버튼의 의미는 *"불이 꺼졌다"*가 아니라 **"관제에서 확인했으니 순찰을 계속하라"**다.
따라서 메시지 이름과 프론트 버튼 라벨 모두 **"해결"이 아니라 "확인"**으로 한다.
사람이 판단한다는 점도 실제 안전 운영 방식에 부합한다.

```yaml
event_gate_node:
  uvd_topic: /event_detection/uvd
  trigger_classes: [fire, fallen_person, no_helmet]   # ship_defect 제외 확정
  min_confidence: 0.5

  inbound_topic: /server/inbound   # websocket_client가 서버 수신분을 발행하는 토픽
  ack_event_type: event_ack        # 이 event_type이 오면 재개

  fallback_auto_resume_s: 0   # 0=비활성. 백엔드 미연결 시연용 자동 재개 시간
```

#### 미처리 이벤트 재감지 시 다시 정지한다 (2026-08-07 팀 결정)

**결론: 쿨다운을 두지 않는다. 재감지되면 다시 정지하고 관제에 다시 알린다.**

**동작 흐름 (의도된 것):**
```
불 감지 -> 정지 -> 관제 "확인" -> 재개 -> 순찰
  -> 배 반대편으로 이동 (6초 이상 미검출) -> ByteTrack이 트랙 삭제
  -> 한 바퀴 돌아옴 -> 불이 아직 그대로 -> 새 track ID -> 다시 보고 -> 다시 정지
```

**근거:** 한 바퀴를 돌 동안에도 처리되지 않은 위험은 **여전히 미처리 상태**이므로
관제에 다시 알리는 것이 맞다. 알림을 억제하면 관제가 "치워졌겠지"라고 오인할 수 있다.

**대가:** 위험물이 치워지기 전까지 **한 바퀴마다 관제에서 "확인"을 다시 눌러야 한다**
(시연장 기준 약 40초에 한 번). 팀이 이 비용을 인지하고 선택했다.

**필요한 작업: 없음.** 지금 코드가 이미 이렇게 동작한다. 욜로·Nav2 양쪽 모두 수정 불필요.

> 초기에는 `ack_cooldown_s`(확인 후 N초간 재감지 무시)를 검토했으나 위 결정으로 폐기했다.
> 발표 중 재확인이 번거롭다고 판단되면 그때 `event_gate_node`에 파라미터로 추가하면 된다.
> **욜로 쪽 수정이 아니라 Nav2 노드 쪽 작업이다.**

#### 왜 6초 뒤에는 "다른 이벤트"가 되는가 — 추적 동작 확인 (2026-08-07)

비전 담당자 문의에 대한 확인 결과 **추적은 이미 켜져 있고 정상 동작한다.**

| 항목 | 위치 | 값 / 내용 |
|---|---|---|
| 추적 실행 | `yolo_depth_publisher.py:218` | `model.track(persist=True, tracker=...)` — `predict()` 아님 |
| 중복 제거 | `yolo_depth_publisher.py:255` | `reported_tids`에 있는 track ID는 발행 안 함 (영구 유지, 삭제 안 됨) |
| 트래커 설정 | `custom_tracker.yaml` | ByteTrack, **`track_buffer: 60`** |
| 설정 파일 설치 | `setup.py:16` | `share/ship_ugv_perception/config/`에 설치 → 코드의 조회 경로와 일치 |

**`track_buffer: 60` = 60프레임 = 추론 주기 0.1초 × 60 = 6초.**
6초 넘게 미검출이면 ByteTrack이 트랙을 삭제하고, 재검출 시 **새 track ID**를 발급한다.
`reported_tids`에 없는 ID이므로 다시 보고된다.

**순찰 중 실제로 6초를 넘는가 (배가 시야를 가리는 시간):**

| 순찰 반지름 | 한 바퀴 | 가려지는 시간(약 절반) | 6초 초과 |
|---|---|---|---|
| 0.30 m (기숙사) | 12.6초 | 약 6.3초 | 🟡 경계 |
| 1.0 m (시연장 예상) | 41.9초 | 약 21초 | ✅ 확실히 초과 |

*(속도 0.15 m/s 기준)*

**빌드 최신 여부 확인 명령 (젯슨):**
```bash
ls ~/smart-shipyard/edge/ros2_ws/install/ship_ugv_perception/share/ship_ugv_perception/config/custom_tracker.yaml
```
파일이 없으면 `colcon build --packages-select ship_ugv_perception` 후 재실행.

#### 정지 판정에 시간 안정화가 필요 없는 이유

`/event_detection/uvd`는 이미 두 겹의 필터를 통과한 결과다.

- `yolo_depth_publisher`: 추적 ID 기준 중복 제거 + `fallback_confirm_frames: 3`
- `min_confidence: 0.5`

따라서 **메시지 1개 = 정지**로 충분하다. 이전 설계(비전 clear 판정)에서 쓰던
`detect_hold_s` / `clear_hold_s` / `signal_timeout_s` / `on_signal_loss` 는
모두 불필요해져 제거한다.

#### 확인(ack) 신호 경로 — 팀 분담 (2026-08-07 확정)

**기존 프론트→젯슨 중계 경로를 그대로 재사용한다.** 새 엔드포인트를 만들지 않는다.
[backend/main.py:318](../../backend/main.py#L318)에 중계 코드가 이미 동작 중이다.

```
프론트 "확인" 클릭
   │  {"event_type": "event_ack"}   (기존 /ws/frontend 소켓)
   ▼
백엔드  JETSON_BOUND_TYPES 에 포함되면 젯슨으로 그대로 배달 (기존 코드)
   │
   ▼
websocket_client.py  수신 -> /server/inbound (std_msgs/String) 로 발행
   │
   ▼
event_gate_node  event_ack 필터 -> /event/active = false -> 순찰 재개
```

| 담당 | 할 일 | 분량 |
|---|---|---|
| 프론트 (고명재) | 이벤트 팝업에 "확인" 버튼 → 기존 소켓으로 `event_ack` 전송 | 버튼 1개 |
| 비전 (이주현) | `websocket_client.py`에 수신 루프 추가 → 받은 메시지를 `/server/inbound`로 그대로 발행 | ~10줄 |
| **본인** | 백엔드 상수 2줄 + `event_gate_node` 구현 | 2줄 + 노드 |

백엔드 수정분 (전부):
```python
EVENT_ACK = "event_ack"
JETSON_BOUND_TYPES = {STREAM_BOOST, WEBRTC_SIGNAL, EVENT_ACK}
```

**프론트엔드는 새 엔드포인트가 필요 없다.** `/ws/frontend`는 이미 양방향이며
`webrtc_signal`이 그 경로로 다니고 있다.

**비전 담당자에게는 우리 이벤트 도메인을 몰라도 되게 일반적으로 요청한다:**
받은 메시지를 해석하지 말고 `/server/inbound`로 그대로 발행만 하면 된다.
서버→젯슨 메시지가 나중에 늘어나도 이 통로 하나로 전부 처리된다.

**왜 `/ws/nav` 신설이 아니라 이 방식인가**
- 백엔드 작업량이 더 적다 (엔드포인트 신설 수십 줄 → 상수 2줄)
- 단일 게이트웨이가 정석 아키텍처다. 연결 관리(재접속·인증)가 한 곳에 모인다
- 소켓이 늘지 않는다

**의존성 관리:** 담당자 일정이 늦어져도 Step 7은 막히지 않는다.
`fallback_auto_resume_s`와 수동 재개로 개발·시연이 가능하다.
```bash
ros2 topic pub --once /event/ack std_msgs/msg/Empty "{}"
```

#### 폐기된 설계 — 비전 연속 신호 방식 (2026-08-07 채택 후 철회)

`/event_detection/present` 토픽(매 프레임 현재 보이는 클래스 목록) 추가를
비전 담당자에게 요청했으나, **관제 확인 방식으로 결정되면서 불필요해져 철회했다.**
비전 담당자 작업량은 0이 됐다.

철회 이유: 재개 판단을 사람이 하므로 "이벤트가 아직 진행 중인가"를
로봇이 알 필요가 없다. 정지에는 엣지 트리거 1회로 충분하다.

⚠️ **참고 — 비전 쪽 "3초"는 위험 이벤트용이 아니다.** 코드 확인 결과:
- `fallback_confirm_frames: 3` = 3**프레임** ≈ 0.3초 (추론 주기 0.1초)
- `block_level_stability_s: 3.0` = 3**초**, **조립단계 전용**
- `_handle_danger_event()` = **시간 조건 없음.** conf 0.5 넘으면 즉시 전송

다만 상위의 `yolo_depth_publisher`가 추적 ID로 이미 1번으로 줄이므로
**서버로 중복 전송되는 문제는 없다** (실기 테스트로 확인됨).

---

## 8. 기타 확인 사항

**뎁스카메라(`edge/vision/third_party/pyorbbecsdk`)**
`.gitmodules`에 등록된 **git submodule**이며 orbbec 공식 SDK 저장소다.
제조사 코드이므로 수정 대상이 아니고 **Nav2와 무관**하다.
장애물 회피는 라이다(`/scan_filtered`)가 담당하며, 뎁스를 코스트맵에 넣으면
젯슨 부하만 증가하므로 **사용하지 않는다.**

**개발 환경 확인 완료**
ROS 2 Humble / Gazebo Classic (`gazebo_ros_pkgs`) / nav2 전체 / `nav2_simple_commander` /
`slam_toolbox` / `robot_localization` / `xacro` — 모두 설치됨.

---

## 9. 실행 순서 (시뮬 / 실물)

### 시뮬 (개발용)

```bash
# 터미널 1 — Gazebo + 로봇 + TF 트리
ros2 launch ship_ugv_navigation sim_bringup.launch.py use_rviz:=true

# 터미널 2 — Nav2 (Step 5 완성 후)
ros2 launch ship_ugv_navigation navigation.launch.py use_sim_time:=true

# 터미널 3 — 키보드 주행 (Nav2 없이 물리 확인만 할 때)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 실물 (시연장) — ★ 끄고 켜는 시점이 중요하다

```bash
# ── 터미널 1 : 처음부터 끝까지 계속 켜둔다 ─────────────────────────
ros2 launch ship_ugv_localization localization.launch.py

# ── 터미널 2 : 매핑할 때만 켰다가 맵 저장 후 반드시 끈다 ───────────
ros2 launch ship_ugv_localization mapping.launch.py
#   … 매핑 주행 → align → map_saver_cli → finalize_map.py …
#   여기서 Ctrl+C  ★필수★

# ── 터미널 2 : 매핑을 끈 뒤에 Nav2를 띄운다 ────────────────────────
ros2 launch ship_ugv_navigation navigation.launch.py map:=<맵경로>
```

**`localization.launch.py`는 매핑 시작부터 시연 종료까지 한 번도 끄지 않는다.**
Nav2가 필요로 하는 것을 전부 이 launch가 제공하기 때문이다.

| Nav2가 필요한 것 | 제공 노드 | 소속 |
|---|---|---|
| `map→odom` TF | `ekf_global` | localization.launch.py |
| `odom→base_link` TF | `ekf_local` | 〃 |
| `base_link→laser` TF | `static_transform_publisher` | 〃 |
| `/scan_filtered` | `rplidar_node` + `laser_filter` | 〃 |
| `/cmd_vel` 소비자 | `wheel_odom_bridge` | 〃 |

끄면 로봇이 자기 위치도 모르고 바퀴도 안 돈다.

**반대로 `mapping.launch.py`는 맵 저장 후 반드시 끈다.**
`slam_toolbox`가 `/map`을 계속 발행하는데 Nav2의 `map_server`도 `/map`을 발행하므로
**두 노드가 같은 토픽을 다투게 되어** 코스트맵에 엉뚱한 지도가 들어갈 수 있다.

> `map_saver_cli`와 `finalize_map.py`는 한 번 실행하고 끝나는 일회성 명령이라
> 끄고 켤 대상이 아니다. 계속 떠 있는 것은 위 세 launch뿐이다.

**시뮬 전용이라 실물에서 실행하지 않는 것**

| 대상 | 이유 |
|---|---|
| `sim_bringup.launch.py` | Gazebo·스폰·시뮬 센서. 실물엔 진짜 하드웨어가 있다 |
| `fake_global_localization` | `map→odom`을 발행. 실물에서 켜면 `ekf_global`과 이중 발행 |
| `ship_ugv_gazebo.xacro` | 시뮬 물리/플러그인 서술 |
| `worlds/*.world` | 시뮬 환경 |

`patrol_mission_node`와 `event_gate_node`는 **시뮬·실물 공용**이다.

---
