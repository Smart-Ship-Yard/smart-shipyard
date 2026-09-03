#!/usr/bin/env bash
# mediamtx 를 ~/mediamtx 에 설치한다. 이미 있으면 아무것도 하지 않는다.
#
#   bash server/streaming/install.sh
#
# ★ 왜 저장소에 넣지 않나
#   바이너리가 55MB 다. git 에 넣으면 clone 이 무거워지고, 리눅스/맥/윈도우가
#   각각 다른 파일이라 팀원마다 다른 것을 받아야 한다. 그래서 "clone 하면 딸려
#   온다" 는 안 되고, 이 스크립트로 한 번 받는다.
#
#   ★ 나중에는 도커로 간다. 그때는 이 스크립트도, 버전 고정도 필요 없어진다.
#     (mediamtx · 백엔드 · mongo 를 한 번에 띄우는 compose 파일 하나로)
set -euo pipefail

VERSION="v1.20.1"        # 설정 파일이 이 버전 기준으로 쓰여 있다
DEST="$HOME/mediamtx"

if [[ -x "$DEST/mediamtx" ]]; then
  echo "이미 설치돼 있습니다: $("$DEST/mediamtx" --version)"
  echo "다시 받으려면 $DEST 를 지우고 실행하세요."
  exit 0
fi

case "$(uname -m)" in
  x86_64)  ARCH=amd64 ;;
  aarch64) ARCH=arm64v8 ;;
  *) echo "이 아키텍처($(uname -m))는 자동 설치를 지원하지 않습니다." >&2
     echo "https://github.com/bluenviron/mediamtx/releases 에서 직접 받으세요." >&2
     exit 1 ;;
esac

URL="https://github.com/bluenviron/mediamtx/releases/download/${VERSION}/mediamtx_${VERSION}_linux_${ARCH}.tar.gz"
echo "받는 중: $URL"
mkdir -p "$DEST"
curl -fsSL -o "$DEST/mediamtx.tar.gz" "$URL"
tar xzf "$DEST/mediamtx.tar.gz" -C "$DEST"
rm -f "$DEST/mediamtx.tar.gz"
echo "설치 완료: $("$DEST/mediamtx" --version)"
echo
echo "다음:"
echo "  sudo bash server/streaming/setup-firewall.sh"
echo "  sudo cp server/streaming/mediamtx.service /etc/systemd/system/"
echo "  sudo systemctl daemon-reload && sudo systemctl enable --now mediamtx"
