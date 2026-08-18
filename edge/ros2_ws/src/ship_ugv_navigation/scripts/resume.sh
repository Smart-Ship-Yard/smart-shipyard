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
set -u

TIMES="${1:-3}"
RATE=5

source /opt/ros/humble/setup.bash 2>/dev/null

echo "▶️ 재개 신호 발행 (${TIMES}회) ..."
ros2 topic pub -t "$TIMES" -r "$RATE" /event/ack std_msgs/msg/Empty "{}" >/dev/null 2>&1

echo -n "   확인 중 ... "
for i in $(seq 1 10); do
  st=$(timeout 2 ros2 topic echo /patrol/status --once 2>/dev/null | head -1)
  if echo "$st" | grep -q '"event_active": false'; then
    echo "✅ 재개됨"
    echo "   $st"
    exit 0
  fi
  sleep 0.5
done
echo "⚠️ 확인 실패 — /patrol/status 의 event_active 가 아직 true 다"
exit 1
