# Nav2 실물 명령어 (compact)

> **이 문서의 범위** — 맵이 **이미 있는 상태에서 실물 로봇으로 Nav2 순찰만 돌릴 때** 필요한 명령어 전부.
> 매핑 / align / 맵 저장은 여기 없다. 그건 `재매핑_체크리스트.md` 와
> `전체_실행_명령어_요약본 — 매핑부터_자율주행까지.md` 를 볼 것.

---

## 0. 모든 터미널 첫 세팅

```bash
cd ~/smart-shipyard/edge/ros2_ws && source install/setup.bash
```

---

## 젯슨

### 터미널 1 — 로컬라이제이션

```bash
ros2 launch ship_ugv_localization localization.launch.py
```

> 서버 연결(`websocket_client`), 라이다, UWB, IMU, 휠 오도메트리, EKF 가 **전부 여기서** 뜬다.
> 뜨자마자 나오는 **센서 배너(✅/❌)** 를 반드시 확인할 것. `❌ /dev/lidar` 상태로 진행하면
> 아무 에러 없이 조용히 아무것도 안 된다.

### 터미널 2 — 모션 컨트롤

```bash
ros2 launch ship_ugv_motion_control motion_control.launch.py
```

> ⚠️ 키보드 조종(teleop)과 **동시 실행 금지**. 두 노드가 `/cmd_vel` 을 같이 쏴서 로봇이 흔들린다.

### 터미널 3 — UWB 캘리브레이션 (로봇을 시작 위치에 놓은 후)

```bash
ros2 service call /uwb_map_calibration/calibrate std_srvs/srv/Trigger
ros2 topic pub --once /motion/move_distance std_msgs/msg/Float64 "{data: 1.5}"
```

**⚠️ 로봇이 1.5 m 직진한다. 앞을 비우고 사람에게 알린 뒤 칠 것.**

캘리브레이션이 실제로 됐는지 확인 (안 하면 **에러 없이** 엉뚱한 곳을 믿는다):

```bash
ros2 run tf2_ros tf2_echo map uwb_frame
```

`Translation: [0.000, 0.000, 0.000]` 이면 **안 된 것**이다. 0 이 아닌 값이 나와야 정상.

### 터미널 3 — RViz (캘리브레이션 끝난 뒤 같은 터미널에서)

```bash
rviz2 -d install/ship_ugv_navigation/share/ship_ugv_navigation/rviz/nav.rviz
```

### 터미널 2 — Nav2 (모션 컨트롤을 `Ctrl+C` 로 끈 다음)

```bash
ros2 launch ship_ugv_navigation navigation.launch.py \
    space:=wide map:=shipyard_map_<장소>_v<버전번호> patrol:=true
```

현재 쓰는 맵:

| 맵 | `space` | 반지름 | 여유 | 비고 |
|---|---|---|---|---|
| `shipyard_map_hall_v1` | `wide` | 0.75 m | 0.30 m | 2바퀴 완주 검증됨 |
| `shipyard_map_JG_room_v5` | `narrow` | 0.30 m | 0.075 m | 좁아서 사람이 막으면 대기(BLOCKED) |

> `space` 를 맵에 안 맞게 주면 경로가 아예 안 나온다. 표대로 줄 것.

### 터미널 5 — 이벤트 정지 / 재개

```bash
# 정지 (이벤트 발생)
~/smart-shipyard/edge/ros2_ws/src/ship_ugv_navigation/scripts/estop.sh

# 재개 (서버 없을 때의 폴백. 정식 경로는 관제 화면 "확인" 버튼)
~/smart-shipyard/edge/ros2_ws/src/ship_ugv_navigation/scripts/resume.sh
```

> 손으로 `ros2 topic pub --once` 를 치면 **첫 번째가 씹히는** 일이 잦다 (DDS 디스커버리가
> 끝나기 전에 프로세스가 죽어서 메시지가 사라진다). 스크립트는 3회 발행하고
> `/patrol/status` 를 폴링해서 실제로 먹었는지까지 확인해 준다.

---

## 백엔드 노트북

### 서버 터미널 1 — 서버 실행

```bash
cd ~/smart-shipyard/backend && venv/bin/uvicorn main:app --host 0.0.0.0 --port 8000
```

### 서버 터미널 2 — 이벤트 확인 후 재개 (프론트 "확인" 버튼 대역)

```bash
cd ~/smart-shipyard/backend && venv/bin/python tools/fake_send_event_ack.py
```

---

## 꼭 기억할 것

1. **캘리브레이션은 젯슨/터미널 1 을 껐다 켤 때마다 다시** 해야 한다.
   값이 노드 메모리에만 있어서 재시작하면 항등변환(0,0,0)으로 초기화된다.
   같은 세션에서 매핑 직후 바로 Nav2 를 돌리는 경우에만 생략 가능.
   **빼먹어도 에러가 안 난다** — 위의 `tf2_echo` 로 직접 확인할 것.

2. **`nav2_params.yaml` 같은 설정 파일은 심볼릭 링크 install** 이라 고치면 바로 반영된다.
   단 **새 노드 파일을 추가**했거나 **`setup.py` 를 고쳤으면** `colcon build` 가 필요하다.

3. **사람은 로봇 앞 0.5 m 밖에서 가로지를 것. 뒤로는 다가가지 말 것.**
   코스트맵은 라이다에서 **0.20 m 안쪽을 장애물로 찍지 않는다** — 앞 범퍼 기준 6 cm.
   그 안은 완전한 사각지대라 발을 들이밀면 로봇이 그대로 민다.
   정후방 150도도 안 보이고, BackUp 복구는 뒤를 안 보고 후진한다.
   보이기만 하면 10 cm 안에 서므로 문제는 제동이 아니라 **보이느냐**다.
   상세 근거와 실측: 설치가이드 6-3절.

4. **모터 폭주 시 `estop.sh` 로는 못 멈춘다.** 그건 "설계된 정상 정지"라 Nav2 목표만 취소한다.
   한쪽 바퀴가 전속으로 계속 도는 상황이면 **배터리를 뽑을 것.** (설치가이드 3-5절)
