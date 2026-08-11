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
  'ship_ugv_navigation/lib'                   # fake_global_localization 등 우리 노드
  'ros2 launch ship_ugv_navigation'           # launch 부모 프로세스
  'teleop_twist_keyboard'
)
names=( gzserver gzclient rviz2 )             # 15글자 이하라 -x 가능

for p in "${patterns[@]}"; do pkill -9 -f "$p" 2>/dev/null; done
for n in "${names[@]}";    do pkill -9 -x "$n" 2>/dev/null; done

sleep 2

# ros2 node list 는 데몬 캐시라 죽은 노드를 한동안 유령으로 보여준다.
# 확인은 실제 프로세스로 한다.
left=$(pgrep -af 'gzserver|gzclient|rviz2|/opt/ros/humble/lib/nav2_|robot_state_publisher|robot_localization|ship_ugv_navigation/lib' 2>/dev/null)

if [ -z "$left" ]; then
  echo "✅ 정리 완료 — 남은 프로세스 없음"
else
  echo "⚠️  아직 남아 있다:"
  echo "$left"
  echo "   위 PID 를 kill -9 <PID> 로 직접 정리할 것"
  exit 1
fi
