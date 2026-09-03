"""
fake_frontend.py — 가짜 프론트엔드 (백엔드 검증용 뷰어)

브라우저 대시보드 없이 백엔드가 뿌리는 데이터를 눈으로 확인하기 위한 스크립트.
docs/interface.md 의 프론트엔드 역할을 흉내 낸다:

    - /ws/frontend : 이벤트 JSON 수신해서 출력 (재접속 복원 포함)

영상은 이 스크립트가 받지 않는다 — 실물에서도 영상은 백엔드를 거치지 않고
서버의 mediamtx 가 받아 브라우저로 보낸다 (docs/interface.md ⑤).

실행 방법 (백엔드 venv 사용):
    cd backend
    venv/bin/python tools/fake_frontend.py
    venv/bin/python tools/fake_frontend.py --server ws://192.168.0.100:8000
"""

import argparse
import asyncio
import json
import time

import websockets


async def json_channel(server: str, duration: float):
    """이벤트 수신해서 출력."""
    async with websockets.connect(f"{server}/ws/frontend") as ws:
        print("🖥️ [가짜 프론트] JSON 채널 접속 완료")

        start = time.monotonic()
        try:
            # 남은 시간만큼만 수신을 기다려서 전체 실행이 duration을 넘지 않게 함.
            while (remaining := duration - (time.monotonic() - start)) > 0:
                raw = await asyncio.wait_for(ws.recv(), timeout=remaining)
                data = json.loads(raw)
                etype = data.get("event_type")

                if etype == "position":
                    # 위치 핑은 너무 많아서 좌표만 간단히 출력
                    print(f"← position {data.get('ekf_global')}")
                else:
                    print(f"← ✅ {etype} 수신: {data}")
        except asyncio.TimeoutError:
            pass  # duration 동안 조용하면 정상 종료
        except websockets.ConnectionClosed as e:
            print(f"⚠️ [가짜 프론트] JSON 채널이 서버 쪽에서 닫힘: {e!r}")

    print("🖥️ [가짜 프론트] JSON 채널 종료")


async def main():
    parser = argparse.ArgumentParser(description="가짜 프론트엔드 (백엔드 검증용)")
    parser.add_argument("--server", default="ws://127.0.0.1:8000",
                        help="백엔드 서버 주소 (기본: ws://127.0.0.1:8000)")
    parser.add_argument("--duration", type=float, default=15.0,
                        help="실행 시간(초), 기본 15초")
    args = parser.parse_args()

    await json_channel(args.server, args.duration)


if __name__ == "__main__":
    asyncio.run(main())
