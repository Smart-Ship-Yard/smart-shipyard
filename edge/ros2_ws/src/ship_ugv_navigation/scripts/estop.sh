#!/usr/bin/env bash
# ==========================================================================
#  estop.sh — 순찰 중인 로봇을 즉시 정지시킨다 (설계된 정상 정지 경로)
# ==========================================================================
#  왜 스크립트인가 (2026-08-18 실측)
#  --------------------------------
#  손으로 치던 `ros2 topic pub --once ...` 가 **첫 번째에 안 먹고 두세 번째에야
#  먹는** 일이 잦았다. `--once` 는 발행 직후 프로세스가 끝나는데, DDS 가 아직
#  전달을 마치지 못한 상태에서 종료되면 메시지가 그대로 사라진다.
#  (디스커버리가 끝나기 전에 쏘는 경쟁 상황이다.)
#
#  시연 중 "정지가 안 먹는다"는 사고를 막기 위해 **여러 번 보낸다.**
#  event_gate_node 는 한 번 걸리면 ack 까지 상태를 래치하므로 중복 수신은
#  무해하다 (nav2_작업_정리.md Step 7 참고).
#
#  사용법
#  ------
#      edge/ros2_ws/src/ship_ugv_navigation/scripts/estop.sh
#
#  재개는 resume.sh 를 쓴다.
#
#  ⚠️ 이것은 "설계된 정상 정지"다 — Nav2 목표를 취소하고 0속도를 유지한다.
#     모터가 물리적으로 폭주하는 상황(한쪽 바퀴가 전속으로 계속 돎)에서는
#     이걸로 안 멈춘다. 그때는 배터리를 뽑을 것. 상세는 설치가이드 3-5절.
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

# ★ 중요한 건 "몇 번"이 아니라 "몇 초 동안"이다 (2026-08-18 실측).
#   -t 3 -r 5 는 0.6초 만에 3개를 다 쏘고 프로세스가 끝난다. DDS 디스커버리가
#   그 안에 안 끝나서 **3개가 통째로 유실됐다** — 정지가 전혀 안 먹었다.
#   -t 6 -r 2 는 3초에 걸쳐 보내므로 디스커버리가 끝난 뒤에도 여러 개가 남는다.
#   event_gate_node 는 한 번 걸리면 래치하므로 중복 수신은 무해하다.
TIMES="${1:-6}"          # 기본 6회 발행
RATE=2                   # 초당 2회 -> 총 3초

echo "🛑 정지 신호 발행 (${TIMES}회) ..."
ros2 topic pub -t "$TIMES" -r "$RATE" /event_detection/uvd std_msgs/msg/String \
  '{data: "{\"class_id\":\"fire\",\"confidence\":0.99,\"depth\":0.5}"}' >/dev/null 2>&1

# 실제로 정지 상태가 됐는지 확인해서 알려준다 (조용히 실패하지 않도록)
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
    *'"event_active": true'*)
      echo "✅ 정지됨"
      echo "   $st"
      exit 0
      ;;
  esac
done
echo "⚠️ 확인 실패 — /patrol/status 의 event_active 가 아직 true 가 아니다"
echo "   event_gate_node 가 떠 있는지 확인할 것: pgrep -af event_gate"
exit 1
