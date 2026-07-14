"""
fake_frontend.py — 가짜 프론트엔드 (백엔드 검증용 뷰어)

브라우저 대시보드 없이 백엔드가 뿌리는 데이터를 눈으로 확인하기 위한 스크립트.
통신 스펙 v1.2의 프론트엔드 역할을 흉내 낸다:

    - /ws/frontend        : 이벤트 JSON 수신해서 출력.
                            첫 위험 이벤트를 받으면 팝업이 열린 것처럼
                            stream_boost start를 보내고, 3초 후 stop을 보냄
                            (→ 서버가 젯슨으로 잘 중계하는지 검증)
    - /ws/frontend-stream : 영상 프레임 수신, 1초마다 수신 fps/크기 요약 출력
                            (→ 부스트 전환 시 fps가 5→15로 뛰는지 검증)

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
    """이벤트 수신 + 위험 이벤트를 계기로 stream_boost start/stop 송신."""
    async with websockets.connect(f"{server}/ws/frontend") as ws:
        print("🖥️ [가짜 프론트] JSON 채널 접속 완료")

        DANGER = {"fallen_person", "fire", "no_helmet", "ship_defect"}
        boost_sent = False

        async def stop_later():
            """팝업을 3초 보다가 닫기 버튼을 누른 상황 재현."""
            await asyncio.sleep(3)
            await ws.send(json.dumps(
                {"event_type": "stream_boost", "action": "stop"}))
            print("→ 🎥 stream_boost stop 전송 (닫기 버튼 재현)")

        stop_task = None
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

                # 첫 위험 이벤트 → 팝업이 열린 것처럼 부스트 시작
                if etype in DANGER and not boost_sent:
                    await ws.send(json.dumps(
                        {"event_type": "stream_boost", "action": "start"}))
                    print("→ 🎥 stream_boost start 전송 (이벤트 팝업 재현)")
                    boost_sent = True
                    stop_task = asyncio.create_task(stop_later())
        except asyncio.TimeoutError:
            pass  # duration 동안 조용하면 정상 종료
        except websockets.ConnectionClosed as e:
            print(f"⚠️ [가짜 프론트] JSON 채널이 서버 쪽에서 닫힘: {e!r}")
        finally:
            if stop_task:
                await asyncio.gather(stop_task, return_exceptions=True)

    print("🖥️ [가짜 프론트] JSON 채널 종료")


async def stream_channel(server: str, duration: float):
    """영상 프레임 수신 — 1초마다 fps와 프레임 크기를 요약 출력."""
    async with websockets.connect(f"{server}/ws/frontend-stream") as ws:
        print("🖥️ [가짜 프론트] 영상 채널 접속 완료")

        start = time.monotonic()
        window_start = start
        count = 0
        last_size = 0
        try:
            # 남은 시간만큼만 수신을 기다려서 전체 실행이 duration을 넘지 않게 함.
            while (remaining := duration - (time.monotonic() - start)) > 0:
                frame = await asyncio.wait_for(ws.recv(), timeout=remaining)
                count += 1
                last_size = len(frame)

                now = time.monotonic()
                if now - window_start >= 1.0:
                    print(f"🎞️ 영상 수신: {count}fps, 프레임 크기 {last_size:,}B")
                    window_start = now
                    count = 0
        except asyncio.TimeoutError:
            pass  # duration 동안 조용하면 정상 종료
        except websockets.ConnectionClosed as e:
            print(f"⚠️ [가짜 프론트] 영상 채널이 서버 쪽에서 닫힘: {e!r}")

    print("🖥️ [가짜 프론트] 영상 채널 종료")


async def main():
    parser = argparse.ArgumentParser(description="가짜 프론트엔드 (백엔드 검증용)")
    parser.add_argument("--server", default="ws://127.0.0.1:8000",
                        help="백엔드 서버 주소 (기본: ws://127.0.0.1:8000)")
    parser.add_argument("--duration", type=float, default=15.0,
                        help="실행 시간(초), 기본 15초")
    args = parser.parse_args()

    await asyncio.gather(
        json_channel(args.server, args.duration),
        stream_channel(args.server, args.duration),
    )


if __name__ == "__main__":
    asyncio.run(main())
