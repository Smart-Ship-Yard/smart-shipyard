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
set -u

TIMES="${1:-3}"          # 기본 3회 발행
RATE=5                   # 초당 5회

source /opt/ros/humble/setup.bash 2>/dev/null

echo "🛑 정지 신호 발행 (${TIMES}회) ..."
ros2 topic pub -t "$TIMES" -r "$RATE" /event_detection/uvd std_msgs/msg/String \
  '{data: "{\"class_id\":\"fire\",\"confidence\":0.99,\"depth\":0.5}"}' >/dev/null 2>&1

# 실제로 정지 상태가 됐는지 확인해서 알려준다 (조용히 실패하지 않도록)
echo -n "   확인 중 ... "
for i in $(seq 1 10); do
  st=$(timeout 2 ros2 topic echo /patrol/status --once 2>/dev/null | head -1)
  if echo "$st" | grep -q '"event_active": true'; then
    echo "✅ 정지됨"
    echo "   $st"
    exit 0
  fi
  sleep 0.5
done
echo "⚠️ 확인 실패"
echo "   /patrol/status 에서 event_active 가 true 로 안 바뀌었다."
echo "   event_gate_node 가 떠 있는지 확인할 것: ros2 node list | grep event_gate"
exit 1
