# ju_ws 설치 및 구성 가이드 (v2)
### Jetson Orin Nano · Ubuntu 22.04 · ROS2 Humble (이미 설치됨 가정)

> **⚠️ 이 가이드는 `ship_ugv_ws_v2.zip`을 기준으로 한다.**
> 이전에 배포된 파일 중 일부(특히 `complementary_filter_node.py`,
> `calibration_node.py`)는 **버그 수정 이전 버전이 섞여 돌아다니고 있으므로**,
> 반드시 v2 zip의 파일만 사용할 것. 구버전 여부를 구분하는 방법은
> 아래 "v2 변경 이력 / 구버전 판별법" 참고.

확정된 하드웨어:

| 장치 | 모델 | 연결 | ROS2 드라이버 | 실제 소스 위치 |
|---|---|---|---|---|
| 2D LiDAR | SLAMTEC RPLIDAR A1M8 | USB (micro USB) | `rplidar_ros` | `shipyard/drivers` (링크) |
| IMU | Hiwonder IM10A (10축) | USB-C | `wit_ros2_imu` | `shipyard/drivers` (링크) |
| UWB | DWM1001-DEV ×5 (앵커4+태그1) | USB (micro USB) | 자작 `uwb_dwm1001_driver` | `ju_ws/src` (실물) |
| 모터+엔코더 | JGB37-520 (1320 CPR) ×N | **USB 아님** — MCU 브리지 필요 | 자작 필요 (미작성) | `ju_ws/src` (실물, 예정) |

---

## v2 변경 이력 / 구버전 판별법

### 🔴 회귀 복원 2건 (구버전이 유통 중 — 반드시 v2 사용)

**1. `heading_complementary_filter/complementary_filter_node.py`**
- **올바른 동작 (v2)**: `yaw_est`는 최초 UWB course-over-ground로만 초기화되고,
  초기화 전에는 `/heading/imu_uwb_fused`를 **발행하지 않는다.**
  (시동 직후 정지 상태에서 이 토픽이 안 나오는 건 **정상**)
- **구버전 버그**: 첫 IMU 콜백에서 `yaw_est = 0.0`으로 미리 채우고 즉시 발행
  → ekf_global에 틀린 절대방향(0 rad)이 주입됨.
- **판별법**: `_imu_cb` 안에 `self.yaw_est = 0.0` 이 있으면 구버전.
  v2에는 `if self.yaw_est is None: return` 만 있다.

**2. `uwb_map_calibration/calibration_node.py`**
- **올바른 동작 (v2)**: `~/calibrate` 서비스는 수집 **시작만 트리거하고 즉시
  리턴**한다 (응답: "수집 시작됨..."). 수집 종료 감지와 계산은 0.2초 주기
  타이머 `_check_collection_done`이 담당하고, 결과는 **로그와 저장 파일로
  확인**한다.
- **구버전 버그**: 서비스 콜백 안에서 `rclpy.spin_once()`로 5초 블로킹 대기
  → SingleThreadedExecutor에서 콜백 재진입으로 데드락/무응답 위험.
- **판별법**: `_calibrate_cb` 안에 `while time.time() < deadline:
  rclpy.spin_once(...)` 가 있으면 구버전. v2에는 `_check_collection_done`
  타이머 메서드가 별도로 존재한다.

### 🟡 v2에서 새로 수정/개선된 것

**3. `uwb_dwm1001_driver/package.xml`**: `<exec_depend>python3-serial</exec_depend>` 추가.
(기존엔 setup.py의 pip 의존성만 있어서, 새 환경에서 `rosdep install`로는
pyserial이 안 잡혔음. 기존 Jetson에서 빌드가 됐던 건 apt로 이미 깔려 있었기 때문.)

**4. `slam_map_alignment/package.xml`**: `<depend>tf2_geometry_msgs</depend>` 추가.
(`_uwb_cb`가 `do_transform_point`를 쓰는데 선언 누락 — 기존 환경에선
`ship_ugv_perception`이 대신 끌어와서 우연히 빌드됐던 것.)

**5. `slam_map_alignment_node.py` 주석 정정**: `uwb_pose_topic` 선언부 옆에
"uwb_frame 원시값을 map 근사로 취급"이라는 **버그 수정 이전 상태를 설명하는
낡은 주석**이 남아 있었음. 실제 코드는 TF 변환을 제대로 하고 있으므로 주석만 정정.

**6. `heading_complementary_filter` 파라미터명 변경**:
`yaw_rate_variance` → **`yaw_variance`**.
이 값은 발행 메시지의 `orientation_covariance[8]`, 즉 yaw **각도** 분산에
들어가는데 이름이 "rate"라 각속도 분산으로 오인할 소지가 있었음.
**launch나 yaml에서 이 파라미터를 오버라이드하고 있었다면 이름을 바꿔야 함.**

**7. `ship_ugv_perception/change_point.py` — depth 해석 파라미터 추가**:
새 파라미터 `depth_is_radial` (기본 `False`).
- `False` (기본): depth = **Z-depth** (광축 수직 평면까지의 거리).
  RealSense 등 대부분의 뎁스카메라 표준. `z_cam = depth`, `x_cam = depth·tan(θ)`
- `True`: depth = **radial** (카메라 원점에서 픽셀 방향 빗변 거리).
  `x_cam = depth·sin(θ)`, `z_cam = depth·cos(θ)` (기존 v1 수식)
- 기존 v1은 radial 고정이었는데, 실제 채택 카메라가 Z-depth 출력이면 화각
  가장자리에서 위치 오차가 생기는 구조였음. **카메라 모델 확정 시 datasheet에서
  depth 출력 방식을 확인하고 이 파라미터를 맞출 것.**

**8. `ekf_global.yaml`에 pose1 블록 추가 (주석 상태)**:
운영 모드용 slam_toolbox pose 입력이 yaml에 **주석 처리된 채로 준비**되어 있음.
매핑 모드에서는 절대 주석 해제 금지 (아래 5-4 참고).

**9. 테스트 디렉토리에 `__init__.py` 추가** (두 패키지):
`python3 -m unittest test.test_...` 실행 시 모듈 로딩 실패하던 문제 해소.

### ⚠️ 전 구간 공통 주의: slam_toolbox pose 토픽명 확인 필수

이 스택에서 slam_toolbox의 pose를 구독하는 곳은 두 군데다:
`slam_map_alignment`(매핑 모드, 기본 파라미터 `/slam_toolbox/pose`)와
`ekf_global`의 pose1(운영 모드, 같은 이름). 그런데 **slam_toolbox는 자기
네임스페이스 기준 `pose`라는 이름으로 발행하므로, launch 구성에 따라 실제
토픽이 `/pose`로 잡힐 수 있다.** 그 경우 두 구독자 모두 빈 토픽을 물고
조용히 아무 일도 안 하게 된다 (에러도 안 남 → 발견이 늦어짐).

slam_toolbox를 처음 띄운 뒤 반드시:

```bash
ros2 topic list | grep -i pose
```

로 실제 이름을 확인하고, `/pose`라면 launch에서 리매핑으로 통일한다:

```python
Node(package='slam_toolbox', executable='...', name='slam_toolbox',
     remappings=[('pose', '/slam_toolbox/pose')], ...)
```

(리매핑 대신 `slam_map_alignment`의 `slam_pose_topic` 파라미터와
ekf_global.yaml의 `pose1` 값을 `/pose`로 바꿔도 되지만, 한 곳만 고치고
한 곳을 빠뜨리는 사고를 막으려면 발행 측 리매핑 한 방이 안전하다.)

---

## 0. 디렉토리 구조 및 워크스페이스 생성

전체 배치 원칙:

- **우리가 직접 짜고 이 프로젝트 전용인 패키지** (자작 6개, 향후
  `wheel_odom_bridge`) → `ju_ws/src`에 **실물**로 둔다.
- **남이 만든 ROS2 드라이버**(`rplidar_ros`, `wit_ros2_imu`)는 **실제 파일은
  `shipyard/drivers`에 두고, `ju_ws/src`에는 심볼릭 링크만 생성**한다.
  colcon은 링크를 실물 폴더처럼 인식해 정상 빌드하고, 파일 실체가 하나라
  어디서 고치든 같은 파일이 바뀐다.
- **ROS와 아예 무관한 것**(설정 툴, 데이터시트, MCU 펌웨어)은 `shipyard/sdk`에 둔다.

```
~/shipyard/
├── sdk/                   # ROS와 무관: 설정 툴, 데이터시트, MCU 펌웨어
│   ├── witmotion/         #   IM10A 설정 툴/프로토콜 문서 (115200/100Hz 설정용)
│   ├── dwm1001/           #   DWM1001 데이터시트, 앵커 설정 자료
│   └── encoder_firmware/  #   아두이노 스케치(.ino) - MCU에 올라가는 코드
├── drivers/               # 남이 만든 ROS2 드라이버의 '실제 소스' (실물)
│   ├── rplidar_ros/       #   실물 — ju_ws/src에는 링크만 존재
│   └── wit_ros2_imu/      #   실물 (ElettraSciComp/witmotion_IMU_ros)
├── maps/                  # 매핑 산출물 (yard.yaml, yard.pgm)
├── calib/                 # 캘리브레이션/정합 결과 JSON (result_save_dir로 지정)
└── ju_ws/                 # ROS2 워크스페이스 (colcon 빌드 대상)
    └── src/
        ├── uwb_dwm1001_driver/           ← 자작, 실물
        ├── heading_complementary_filter/ ← 자작, 실물
        ├── uwb_map_calibration/          ← 자작, 실물
        ├── ship_ugv_perception/          ← 자작, 실물
        ├── ship_ugv_localization/        ← 자작, 실물
        ├── slam_map_alignment/           ← 자작, 실물
        ├── rplidar_ros@         ← shipyard/drivers/rplidar_ros 심볼릭 링크
        └── wit_ros2_imu@        ← shipyard/drivers/wit_ros2_imu 심볼릭 링크
```

```bash
mkdir -p ~/shipyard/{sdk,drivers,maps,calib} ~/shipyard/ju_ws/src
```

자작 패키지 6개를 복사 (zip 겉껍데기 폴더는 버리고 **알맹이만** `ju_ws/src`로).
**기존에 v1 패키지가 이미 있다면 반드시 먼저 삭제하고 덮어쓴다** —
`cp -r`만으로는 v1에만 있던 파일이 남아 뒤섞일 수 있다:

```bash
cd ~/shipyard
unzip ship_ugv_ws_v2.zip

# 기존 자작 패키지가 있으면 완전 교체 (구버전 잔재 방지)
for p in uwb_dwm1001_driver heading_complementary_filter uwb_map_calibration \
         ship_ugv_perception ship_ugv_localization slam_map_alignment; do
  rm -rf ju_ws/src/$p
done

cp -r ship_ugv_ws/src/* ju_ws/src/
rm -rf ship_ugv_ws ship_ugv_ws_v2.zip
```

---

## 1. apt 패키지 설치

```bash
sudo apt update

# 핵심 ROS 패키지
sudo apt install -y \
  ros-humble-robot-localization \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-imu-tools

# 빌드 도구 및 파이썬 의존성
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-numpy \
  python3-pytest \
  python3-serial \
  git
```

`python3-serial`이 pip의 pyserial 대신 apt로 들어간다 (ROS 환경 충돌 방지).
v2부터는 `uwb_dwm1001_driver/package.xml`에도 선언되어 있어
`rosdep install`만으로도 잡힌다.

rosdep 초기화 (한 번만):

```bash
sudo rosdep init   # 이미 했으면 "already initialized" 에러 무시
rosdep update
```

---

## 2. 센서 드라이버 소스 클론 (shipyard/drivers에 실물, ju_ws/src엔 링크)

### 2-1. RPLIDAR A1M8

```bash
# 1) 실제 소스는 shipyard/drivers에 클론
cd ~/shipyard/drivers
git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git

# 2) ju_ws/src에는 심볼릭 링크만 생성
cd ~/shipyard/ju_ws/src
ln -s ~/shipyard/drivers/rplidar_ros rplidar_ros
```

- 발행 토픽: `/scan` (sensor_msgs/LaserScan)
- A1M8 시리얼 보드레이트: **115200** (launch 파라미터 기본값 확인)
- launch: `ros2 launch rplidar_ros rplidar_a1_launch.py`

### 2-2. Hiwonder IM10A (WitMotion 계열)

**저장소 주의**: `WITMOTION/wit_ros2_imu`는 존재하지 않는 저장소명(404).
검증된 저장소는 `ElettraSciComp/witmotion_IMU_ros`(ros2 브랜치) — WitMotion
계열(내부적으로 IM10A와 같은 JY901B급 칩) 범용 드라이버.

```bash
# 의존성: QtSerialPort(Qt 5.2+)
sudo apt install -y qtbase5-dev libqt5serialport5-dev

# 1) 실제 소스 클론 (--recursive 필수! 서브모듈 포함)
cd ~/shipyard/drivers
git clone -b ros2 --recursive https://github.com/ElettraSciComp/witmotion_IMU_ros.git wit_ros2_imu
# --recursive 없이 받았다면: cd wit_ros2_imu && git submodule update --init --recursive

# 2) 심볼릭 링크 생성
cd ~/shipyard/ju_ws/src
ln -s ~/shipyard/drivers/wit_ros2_imu wit_ros2_imu
```

- 발행 토픽/노드명은 `config.yml` 및 launch 인자에 따라 다름 — 클론 후
  README와 `config.yml`을 확인해 실제 토픽명 파악 (5-2 리매핑 참고)
- **중요**: 이 IMU 메시지의 `orientation` 필드는 자력계 융합 결과이므로
  조선소 환경에서 **절대 신뢰 금지**. `angular_velocity`,
  `linear_acceleration`만 사용한다. (ekf_local.yaml의 imu0_config가 이미
  orientation을 fuse하지 않도록 설정됨 — yaw는 heading_complementary_filter가 공급)

**확인**: `ls -l ~/shipyard/ju_ws/src`에서 `rplidar_ros`, `wit_ros2_imu` 옆에
`->` 화살표(원본 경로)가 보이면 링크 정상.

### 2-3. UWB (DWM1001-DEV)

자작 `uwb_dwm1001_driver`가 있으므로 추가 클론 불필요.
USB 연결 시 J-Link 가상 시리얼 포트로 잡힘 (보통 `/dev/ttyACM0`).

---

## 3. 시리얼 포트 권한 및 고정 이름 (udev)

USB 장치 3개(LiDAR, IMU, UWB)의 `/dev/ttyUSB*`, `/dev/ttyACM*` 번호가 부팅
순서에 따라 뒤바뀔 수 있음 → udev 규칙으로 고정 이름 부여가 사실상 필수.

```bash
sudo usermod -aG dialout $USER
# 이후 로그아웃/재로그인
```

각 장치를 하나씩만 꽂고
`udevadm info -a -n /dev/ttyUSB0 | grep -E "idVendor|idProduct"`로 VID/PID
확인 후 `/etc/udev/rules.d/99-ju-sensors.rules` 생성:

```
# RPLIDAR A1M8 (CP2102: VID 10c4, PID ea60)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="rplidar", MODE="0666"

# Hiwonder IM10A (CH340: VID 1a86, PID 7523 — 실측 확인 필요)
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="imu", MODE="0666"

# DWM1001-DEV (SEGGER J-Link: VID 1366 — 실측 확인 필요)
SUBSYSTEM=="tty", ATTRS{idVendor}=="1366", SYMLINK+="uwb", MODE="0666"
```

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

이후 각 노드의 시리얼 포트 파라미터를 `/dev/rplidar`, `/dev/imu`, `/dev/uwb`로 지정.

### 3-1. VID/PID 충돌 시 예비책: 물리 포트 경로(KERNELS) 고정

RPLIDAR와 IM10A가 같은 범용 시리얼 칩(CP2102/CH340)을 쓰면 VID/PID가 겹쳐
부팅 때마다 이름이 뒤바뀌는 사고가 남. 이 경우 물리 포트 경로로 구분:

```bash
udevadm info -a -n /dev/ttyUSB0 | grep 'KERNELS=="[0-9]'
```

```
SUBSYSTEM=="tty", KERNELS=="1-2.1", SYMLINK+="rplidar", MODE="0666"
SUBSYSTEM=="tty", KERNELS=="1-2.2", SYMLINK+="imu", MODE="0666"
```

트레이드오프: **항상 같은 USB 구멍에 같은 장치를 꽂아야 함** —
케이블/포트 라벨링까지가 세트. (USB 허브 사용 시 허브 포트 위치까지
경로에 포함되므로 허브 구성도 고정할 것.)

---

## 4. 빌드

```bash
cd ~/shipyard/ju_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
echo "source ~/shipyard/ju_ws/install/setup.bash" >> ~/.bashrc
```

(`/opt/ros/humble/setup.bash`가 `.bashrc`에서 먼저 source되는지 확인)

빌드 후 유닛테스트로 정상 여부 확인 (선택):

```bash
cd ~/shipyard/ju_ws/src/slam_map_alignment
PYTHONPATH=. python3 -m unittest test.test_rigid_transform_2d -v
cd ../heading_complementary_filter
PYTHONPATH=. python3 -m unittest test.test_complementary_filter -v
```

---

## 5. 만들어야 할 파일 (미완성 항목)

### 5-1. 엔코더 브리지 — 반드시 자작 필요

JGB37-520 엔코더는 USB 장치가 아님:

```
[JGB37-520 엔코더 신호] → [모터드라이버 + MCU(아두이노 등)]
   → USB 시리얼 → [Jetson: wheel_odom_bridge 노드] → /wheel/odom
```

만들 것 두 가지:
1. **MCU 펌웨어**: 엔코더 인터럽트 카운트 + 모터 PWM 제어. 주기적으로
   `<좌측카운트,우측카운트,dt>` 시리얼 송신, cmd_vel 명령 수신.
2. **ROS2 노드** (`ju_ws/src/wheel_odom_bridge/`): 시리얼 파싱 →
   차동구동 기구학으로 `/wheel/odom` (nav_msgs/Odometry) 발행,
   `/cmd_vel` 구독 → 시리얼로 모터 명령 전달.
   - 엔코더 1320 CPR, 바퀴 지름 65mm, 좌우 바퀴 간격(실측) 파라미터화

### 5-2. IMU 설정 및 토픽 리매핑

**보드레이트 주의 (중요)**: IM10A는 출고 기본 **9600bps / 10Hz** 출력.
이대로는 EKF에 필요한 50~100Hz를 감당 못 함 (데이터 밀림 → 방향 추정 붕괴).

1. **센서 자체 설정 변경 (1회)**: WitMotion 설정 툴(Windows) 또는 설정 명령
   패킷으로 **115200bps + 100Hz(최소 50Hz)** 출력으로 변경·저장.
   센서 내부 플래시에 저장되는 설정이므로 ROS 파라미터보다 먼저 해둘 것.
2. **ROS 파라미터를 그에 맞춤**: `baud: 115200` (9600 절대 금지)

```python
Node(
    package='wit_ros2_imu', executable='wit_ros2_imu', name='imu',
    parameters=[{'port': '/dev/imu', 'baud': 115200}],
    remappings=[('/wit/imu', '/imu/data')],
)
```

발행 토픽명은 패키지 버전에 따라 다를 수 있으니 `ros2 topic list`로
실제 이름 확인 후 리매핑을 맞출 것.

### 5-3. launch 파일 두 벌 (매핑/운영 모드 분리)

**파라미터명 주의**: `publish_tf`는 robot_localization 파라미터고,
**slam_toolbox에는 그런 파라미터가 없다.** slam_toolbox의 TF 발행을 끄는 건
**`transform_publish_period: 0.0`**. 이중 안전장치로 운영 모드에서도
`map_frame: slam_map`을 유지한다 — 설령 TF가 새어나가도 `slam_map→odom`이라
ekf_global의 진짜 `map→odom`과 충돌하지 않는다.

**토픽 리매핑 주의**: 두 launch 모두 slam_toolbox 노드에
`remappings=[('pose', '/slam_toolbox/pose')]`를 넣어 pose 토픽명을
자작 스택의 기대값과 통일할 것 (상단 "⚠️ 전 구간 공통 주의" 참고).

`ship_ugv_localization/launch/`에 추가:

- **mapping.launch.py**: rplidar + 엔코더 브리지 + IMU 드라이버 + ekf_local
  + slam_toolbox(mapping 모드, `map_frame: slam_map`, TF 발행 켬 = 기본값,
  pose 리매핑) + uwb 드라이버 + uwb_map_calibration + slam_map_alignment
  + `base_link→laser` static_transform_publisher
  → 매핑 종료 후: `ros2 service call /slam_map_alignment/align std_srvs/srv/Trigger`
  → `ros2 run nav2_map_server map_saver_cli -f ~/shipyard/maps/yard`
  ※ **ekf_global의 pose1은 반드시 주석 상태** (아래 5-4)

- **operation.launch.py**: rplidar + 엔코더 브리지 + IMU 드라이버
  + ekf_local + ekf_global(`publish_tf: true`, **pose1 활성화**)
  + uwb 드라이버 + uwb_map_calibration + heading_complementary_filter
  + slam_toolbox(localization 모드, `map_frame: slam_map`,
  **`transform_publish_period: 0.0`**, pose 리매핑)
  + Nav2(정적 지도 로드) + change_point
  + `base_link→laser` static_transform_publisher

### 5-4. ekf_global.yaml의 pose1 활성화 (운영 모드 전용)

v2의 `ekf_global.yaml`에는 pose1 블록이 **주석 처리된 채로 이미 들어 있다.**
운영 모드 진입 시 주석을 해제하거나, 운영용 yaml 오버레이를 별도로 만들어 켠다.

**매핑 모드에서 절대 켜면 안 되는 이유**: 매핑 중 slam pose는 `slam_map`
프레임인데, `map→slam_map` 정합 TF는 매핑이 끝나고 `~/align`을 호출해야
생긴다. 그 전에 pose1을 켜면 EKF가 변환 불가하거나 (TF 트리 상태에 따라)
틀린 좌표가 흘러들어간다.

**활성화 전 사전 확인 3가지**:

1. **실제 토픽명**: slam_toolbox는 네임스페이스 기준 `pose`로 발행하므로
   실제 토픽이 `/pose`일 수 있음. `ros2 topic list | grep -i pose`로 확인 후
   launch 리매핑으로 `/slam_toolbox/pose`에 통일 (상단 공통 주의 참고).
2. **메시지 타입**: robot_localization의 poseN 입력은
   `geometry_msgs/PoseWithCovarianceStamped`만 인식.
   `ros2 topic info <실제토픽명>`으로 타입 확인.
   `PoseStamped`라면 고정 covariance를 채워 변환하는 릴레이 노드 필요.
3. **covariance 현실성**: slam_toolbox가 채우는 covariance가 비현실적으로
   작으면 ekf_global이 LiDAR를 UWB 대비 과신함.
   `ros2 topic echo <실제토픽명> --field pose.covariance`로 확인,
   필요 시 릴레이 노드에서 재설정.

### 5-5. 뎁스카메라 확정 시 — `depth_is_radial` 파라미터 확인 (v2 신규)

`change_point.py`의 새 파라미터 `depth_is_radial`은 기본 `False`(Z-depth 가정,
RealSense류 표준)다. **카메라 모델이 확정되면 datasheet에서 depth 출력이
Z-depth인지 radial인지 확인하고 파라미터를 맞출 것.** 잘못 설정하면 화각
가장자리에서 감지 물체의 map 좌표가 어긋난다 (중심부는 차이 미미).
아울러 `camera_hfov_deg`, `image_width`, `camera_offset_*`도 실측/스펙 값으로 갱신.

---

## 6. 실행 순서 요약 (조립 완료 후)

### 최초 1회 (지도 만들기)
```bash
ros2 launch ship_ugv_localization mapping.launch.py
# 로봇을 기준점에서 known_heading 방향으로 직진시키며:
ros2 service call /uwb_map_calibration/calibrate std_srvs/srv/Trigger
#   → 서비스는 "수집 시작됨"으로 즉시 응답하고, 5초 뒤 결과는
#     노드 로그와 result_save_dir의 calib_XXX.json으로 확인 (v2 동작)
# 야드 전체를 주행하며 매핑 (회전 구간 포함! — RANSAC 정합의 조건수 확보)
ros2 service call /slam_map_alignment/align std_srvs/srv/Trigger
ros2 run nav2_map_server map_saver_cli -f ~/shipyard/maps/yard
```

**전제조건 리마인드**: 매핑 시작 지점과 UWB 캘리브레이션 시작 지점이 **동일한
물리적 기준점**이어야 두 좌표계 원점이 일치한다. 어긋나면 정합 결과에
계통 오차가 고정된다.

### 매일 운영 (앵커 재배치 후)
```bash
ros2 launch ship_ugv_localization operation.launch.py map:=~/shipyard/maps/yard.yaml
# 같은 물리적 기준점에서 직진하며:
ros2 service call /uwb_map_calibration/calibrate std_srvs/srv/Trigger
# 이후 Nav2 자율주행 운용
```

**동작 확인 팁 (v2 heading filter)**: 시동 직후 정지 상태에서는
`/heading/imu_uwb_fused`가 **발행되지 않는 게 정상**이다. 로봇이 약 30cm 이상
이동해 UWB course로 yaw가 초기화되면 로그에
`yaw 초기화 완료 (UWB course 기반): XX.Xdeg`가 찍히고 그때부터 발행이 시작된다.

**결과물 저장 경로 통일**: 두 노드의 결과 JSON 기본 경로는 `/tmp/...`인데
/tmp는 재부팅 시 삭제됨. launch에서 `~/shipyard/calib` 지정 권장:

```python
Node(package='uwb_map_calibration', executable='calibration_node',
     parameters=[{'result_save_dir': '/home/<사용자명>/shipyard/calib'}]),
Node(package='slam_map_alignment', executable='slam_map_alignment_node',
     parameters=[{'result_save_dir': '/home/<사용자명>/shipyard/calib'}]),
```

---

## 7. 남은 작업 체크리스트

- [ ] **v1 패키지 잔재 제거 후 v2로 완전 교체** (섹션 0의 `rm -rf` 포함 복사 절차)
- [ ] `shipyard/drivers` 클론 + `ju_ws/src` 심볼릭 링크 확인 (`ls -l`)
- [ ] 엔코더 MCU 펌웨어 + wheel_odom_bridge 노드 작성 (하드웨어 확정 후)
- [ ] udev VID/PID 실측 및 rules 확정 (충돌 시 KERNELS 방식 + 포트 라벨링)
- [ ] IM10A 센서 설정 115200bps + 100Hz 변경 (WitMotion 설정 툴, 1회)
- [ ] wit_ros2_imu 빌드 확인 (QtSerialPort 의존성) 및 실제 토픽명 확인 후 리매핑 확정
- [ ] mapping.launch.py / operation.launch.py 작성 (base_link→laser static TF, slam_toolbox pose 리매핑 포함)
- [ ] **slam_toolbox pose 실제 토픽명 확인** (`ros2 topic list | grep -i pose`) → launch 리매핑으로 `/slam_toolbox/pose` 통일
- [ ] 운영 모드에서 ekf_global.yaml의 pose1 주석 해제 (매핑 모드에선 유지!)
- [ ] slam_toolbox pose 메시지 타입·covariance 확인 (`ros2 topic info`)
- [ ] pose0/pose1_rejection_threshold 실측 튜닝
- [ ] 뎁스카메라 모델 확정 → depth 출력 방식(Z/radial) 확인 후 `depth_is_radial` 설정, base_link→camera_link TF 편입
- [ ] launch/yaml에서 `yaw_rate_variance`를 오버라이드하던 곳이 있으면 `yaw_variance`로 개명 (v2 파라미터명 변경)
- [ ] DWM1001 lec 프로브 로직 실기 검증: `\r\r` shell 진입이 기존 lec 스트림을 끊는지 확인 (Firmware User Guide 참조)
