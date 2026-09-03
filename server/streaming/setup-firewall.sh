#!/usr/bin/env bash
# 중앙 미디어 서버가 쓰는 포트를 ufw 에 연다. 한 번만 실행하면 된다.
#
#   sudo bash server/streaming/setup-firewall.sh
#
# ★ 8189/udp 를 빠뜨리기 쉽다.
#   8889/tcp 만 열면 재생 페이지는 뜨는데 영상이 영원히 안 나온다. WebRTC 는
#   시그널링만 8889 로 하고 실제 영상은 ICE 로 뚫은 UDP(8189)로 흐르기 때문이다.
#   에러도 안 뜨고 그냥 검은 화면이라 원인을 찾기 어렵다.
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
  echo "sudo 로 실행하세요: sudo bash $0" >&2
  exit 1
fi

# 같은 공유기 안에서만 허용한다. 인터넷 전체에 여는 것이 아니다.
LAN=192.168.0.0/24

ufw allow from "$LAN" to any port 8554 proto tcp comment 'mediamtx RTSP push (젯슨 -> 서버)'
ufw allow from "$LAN" to any port 8889 proto tcp comment 'mediamtx WebRTC 시그널링/재생 페이지'
ufw allow from "$LAN" to any port 8189 proto udp comment 'mediamtx WebRTC ICE 미디어 (빠뜨리면 검은 화면)'
# 녹화 재생 API. 이벤트 시각으로 그 순간 영상을 뽑을 때 쓴다.
# 녹화를 안 쓸 거면 이 줄은 없어도 된다.
ufw allow from "$LAN" to any port 9996 proto tcp comment 'mediamtx 녹화 재생 API'

echo
echo "== 적용된 규칙 =="
ufw status | grep -E '8554|8889|8189|9996' || true
echo
echo "젯슨에서 확인:  nc -zv 192.168.0.5 8554   -> succeeded 가 나와야 함"
