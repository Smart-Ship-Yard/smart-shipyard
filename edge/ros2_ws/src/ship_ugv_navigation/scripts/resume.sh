#!/usr/bin/env bash
# ==========================================================================
#  resume.sh — 이벤트로 정지한 로봇을 다시 순찰시킨다 (수동 폴백)
# ==========================================================================
#  원래 재개는 **관제 화면의 "확인" 버튼**으로 한다
#  (프론트 -> 백엔드 -> /ws/jetson -> websocket_client -> /server/inbound).
#  이 스크립트는 서버가 없거나 프론트가 아직 없을 때 쓰는 폴백이다.
#
#  estop.sh 와 같은 이유로 여러 번 보낸다 — `--once` 는 DDS 전달 전에
#  프로세스가 끝나 메시지가 유실되는 일이 잦다 (2026-08-18 실측).
#  /event/ack 은 여러 번 와도 무해하다.
#
#  사용법
#  ------
#      edge/ros2_ws/src/ship_ugv_navigation/scripts/resume.sh
# ==========================================================================
# ⚠️ set -u 를 절대 쓰지 말 것 (2026-08-18 사고).
#   /opt/ros/humble/setup.bash 는 AMENT_TRACE_SETUP_FILES 같은 **정의되지 않은
#   변수를 참조**한다. set -u 상태에서 소싱하면 셸이 그 줄에서 즉시 죽는다:
#       /opt/ros/humble/setup.bash: line 8: AMENT_TRACE_SETUP_FILES: unbound variable
#   게다가 그 소싱 줄에 2>/dev/null 이 붙어 있어서 에러 메시지까지 사라진다.
#   -> **아무 출력도 없이, 신호를 한 번도 발행하지 않고 종료**했다.
#   비상정지 스크립트가 조용히 죽는 것보다 나쁜 일은 없다.
set -o pipefail

# 이미 소싱돼 있으면 건드리지 않는다. 안 돼 있을 때만 소싱하되,
# 위 이유로 -u 없이 한다 (이 스크립트는 애초에 -u 를 켜지 않는다).
if [ -z "${ROS_DISTRO:-}" ]; then
  source /opt/ros/humble/setup.bash
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "❌ ros2 명령을 찾을 수 없다. ROS 환경을 소싱한 터미널에서 실행할 것:"
  echo "   cd ~/smart-shipyard/edge/ros2_ws && source install/setup.bash"
  exit 1
fi

# ★ estop.sh 와 같은 이유로 "3초 동안" 보낸다. 0.6초로는 DDS 디스커버리를
#   못 기다려서 통째로 유실된다 (2026-08-18 실측).
TIMES="${1:-6}"
RATE=2

echo "▶️ 재개 신호 발행 (${TIMES}회) ..."
ros2 topic pub -t "$TIMES" -r "$RATE" /event/ack std_msgs/msg/Empty "{}" >/dev/null 2>&1

# ★ 확인 방법에 두 번 데였다 (2026-08-18). 지금 방식이 유일하게 동작한다.
#   ① `timeout 2 ros2 topic echo --once` 반복  -> 매번 노드를 새로 만들어
#      디스커버리하느라 2초 안에 한 건도 못 받는다. 로봇은 멀쩡히 멈췄는데
#      스크립트만 "확인 실패"를 뱉었다.
#   ② `ros2 topic echo | grep -m1 -q`          -> grep 이 파이프를 블록 단위로
#      읽어서, 초당 130바이트짜리 스트림으로는 버퍼가 안 차 영영 판정을 못 한다.
#   -> --once 를 쓰되 **타임아웃을 5초로 넉넉히** 주고 여러 번 시도한다.
echo -n "   확인 중 ... "
for i in $(seq 1 6); do
  st=$(timeout 5 ros2 topic echo /patrol/status --once 2>/dev/null | head -1)
  case "$st" in
    *'"event_active": false'*)
      echo "✅ 재개됨"
      echo "   $st"
      exit 0
      ;;
  esac
done
echo "⚠️ 확인 실패 — /patrol/status 의 event_active 가 아직 true 다"
echo "   순찰 노드가 떠 있는지 확인할 것: pgrep -af patrol_mission"
exit 1
