#!/usr/bin/env bash
# ============================================================================
#  stop_all.sh — 시뮬·Nav2 관련 프로세스를 전부 정리한다
# ============================================================================
#  시뮬을 다시 띄우기 전에 반드시 실행한다.
#
#      ~/smart-shipyard/edge/ros2_ws/src/ship_ugv_navigation/scripts/stop_all.sh
#
#  ── 왜 스크립트로 만들었나 ─────────────────────────────────────────────
#  pkill 패턴을 터미널에 직접 치면 **그 명령줄 자체가 패턴에 걸려**
#  셸이 스스로를 죽이는 일이 생긴다. 그러면 뒤쪽 명령이 실행되지 않아
#  일부 프로세스가 살아남는다. 스크립트 안에서는 명령줄이 스크립트 경로뿐이라
#  이 문제가 없다.
#
#  ── 왜 정리가 필요한가 ─────────────────────────────────────────────────
#  launch 를 Ctrl+C 로 껐어도 자식 노드가 남는 경우가 있다. 남은 ekf_local 과
#  fake_global_localization 은 죽은 Gazebo 의 옛 시계로 얼어붙은 TF 를 계속
#  쏜다. 그러면 "Tf has two or more unconnected trees" 가 뜨면서
#  **Gazebo 에서는 로봇이 움직이는데 RViz 에서는 멈춰 있는** 증상이 나온다.
#
#  ── pkill -x 를 쓰면 안 되는 것들 ──────────────────────────────────────
#  리눅스는 프로세스 이름을 15글자까지만 저장한다. -x 는 그 잘린 이름과
#  완전 일치를 요구하므로 아래는 -x 로 **영원히 안 죽는다.**
#      robot_state_publisher(21) fake_global_localization(24)
#      controller_server(17) velocity_smoother(17) lifecycle_manager(17)
#  그래서 이 스크립트는 실행 파일 경로(-f)로 잡는다.
# ============================================================================
set -u

patterns=(
  '/opt/ros/humble/lib/nav2_'                 # Nav2 6개 노드 + lifecycle_manager
  '/opt/ros/humble/lib/robot_state_publisher' # URDF -> TF
  '/opt/ros/humble/lib/robot_localization'    # ekf_local
  'ship_ugv_navigation/lib'                   # fake_global_localization, 순찰, 이벤트게이트
  'ros2 launch ship_ugv_navigation'           # launch 부모 프로세스
  'teleop_twist_keyboard'
  # ★ 노트북에서 시험용으로 띄운 인지 패키지 노드도 정리한다.
  #   특히 websocket_client 는 **재연결 루프**가 있어서, 죽이지 않으면
  #   백엔드를 다시 띄우는 순간 알아서 다시 붙는다. 그러면 띄운 적도 없는데
  #   "젯슨 연결됨" 로그가 찍히고, 실물 젯슨을 붙였을 때 노트북 쪽 연결이
  #   젯슨을 밀어낼 수 있다 (백엔드는 마지막 접속 하나만 들고 있다).
  #   2026-08-12 실제로 물려서 websocket_client 가 2개 붙어 있었다.
  'ship_ugv_perception/lib'
  'ros2 run ship_ugv_perception'
)
names=( gzserver gzclient rviz2 )             # 15글자 이하라 -x 가능

for p in "${patterns[@]}"; do pkill -9 -f "$p" 2>/dev/null; done
for n in "${names[@]}";    do pkill -9 -x "$n" 2>/dev/null; done

sleep 2

# ros2 node list 는 데몬 캐시라 죽은 노드를 한동안 유령으로 보여준다.
# 확인은 실제 프로세스로 한다.
left=$(pgrep -af 'gzserver|gzclient|rviz2|/opt/ros/humble/lib/nav2_|robot_state_publisher|robot_localization|ship_ugv_navigation/lib|ship_ugv_perception/lib' 2>/dev/null)

if [ -z "$left" ]; then
  echo "✅ 정리 완료 — 남은 프로세스 없음"
else
  echo "⚠️  아직 남아 있다:"
  echo "$left"
  echo "   위 PID 를 kill -9 <PID> 로 직접 정리할 것"
  exit 1
fi
