"""
fake_jetson.py — 가짜 젯슨 (통신 스펙 v1.2 준수 목업)

실물 젯슨/RC카 없이 백엔드 서버를 테스트하기 위한 스크립트.
통신 스펙 v1.2(docs/interface.md)의 젯슨 역할을 그대로 흉내 낸다:

    ① position    : 0.5초마다 위치 핑 (원을 그리며 움직이는 가짜 궤적)
    ② 위험 이벤트  : 시작 2초 후 fallen_person, 4초 후 fire 전송
    ③ block_level : 접속 직후 현재 단계(2) 1번 전송
    ④ ship_pose   : 접속 직후 배 위치 측량 결과 1번 전송
    ⑤ 영상        : 가짜 JPEG 프레임을 평상시 5fps / 부스트 시 15fps로 전송
    ⑥ stream_boost: 서버에서 오는 start/stop 명령을 수신해 부스트 모드 전환

실행 방법 (백엔드 venv 사용):
    cd backend
    venv/bin/python tools/fake_jetson.py                     # 기본: 127.0.0.1:8000, 12초
    venv/bin/python tools/fake_jetson.py --duration 60       # 60초 동안
    venv/bin/python tools/fake_jetson.py --server ws://192.168.0.100:8000
"""

import argparse
import asyncio
import json
import math
import time

import websockets

# 가짜 JPEG 프레임: 서버는 디코딩하지 않으므로 JPEG 시그니처 + 채움 바이트면 충분.
# 평상시/부스트의 크기 차이를 흉내 내서 수신 측에서 모드 전환을 확인할 수 있게 함.
FAKE_FRAME_NORMAL = b"\xff\xd8\xff" + b"N" * 20_000   # ~20KB (480x360/q60 흉내)
FAKE_FRAME_BOOST = b"\xff\xd8\xff" + b"B" * 50_000    # ~50KB (640x480/q80 흉내)

# 부스트 모드 여부 (stream_boost 수신 태스크와 영상 전송 태스크가 공유)
boost_mode = False


async def json_channel(server: str, duration: float):
    """①~④ 전송 + ⑥ 수신을 담당하는 JSON 채널."""
    global boost_mode

    async with websockets.connect(f"{server}/ws/jetson") as ws:
        print("🚗 [가짜 젯슨] JSON 채널 접속 완료")

        # --- 접속 직후 1번: 현재 조립 단계(③) + 배 위치 측량 결과(④) ---
        await ws.send(json.dumps(
            {"event_type": "block_level", "block_id": "B1", "level": 2}))
        print("→ block_level 전송 (level=2)")

        await ws.send(json.dumps(
            {"event_type": "ship_pose", "block_id": "B1",
             "map_xy": [5.1, 4.8], "yaw": 1.57}))
        print("→ ship_pose 전송 (map_xy=[5.1, 4.8], yaw=1.57)")

        async def send_loop():
            """0.5초마다 위치 핑(①), 정해진 시점에 위험 이벤트(②) 전송."""
            start = time.monotonic()
            sent_fall = sent_fire = False

            while time.monotonic() - start < duration:
                t = time.monotonic() - start

                # ① 위치 핑: 반지름 2m 원을 도는 가짜 궤적
                x = 5.0 + 2.0 * math.cos(t * 0.5)
                y = 5.0 + 2.0 * math.sin(t * 0.5)
                await ws.send(json.dumps(
                    {"event_type": "position",
                     "ekf_global": [round(x, 2), round(y, 2)]}))

                # ② 위험 이벤트: 2초 시점 fallen_person, 4초 시점 fire (각 1번)
                if t >= 2.0 and not sent_fall:
                    await ws.send(json.dumps(
                        {"event_type": "fallen_person", "confidence": 0.91,
                         "depth_xyz": [1.2, 0.4, 2.1],
                         "ekf_global": [round(x, 2), round(y, 2)]}))
                    print("→ 🚨 fallen_person 이벤트 전송")
                    sent_fall = True

                if t >= 4.0 and not sent_fire:
                    await ws.send(json.dumps(
                        {"event_type": "fire", "confidence": 0.87,
                         "depth_xyz": [0.8, 0.1, 3.0],
                         "ekf_global": [round(x, 2), round(y, 2)]}))
                    print("→ 🚨 fire 이벤트 전송")
                    sent_fire = True

                await asyncio.sleep(0.5)

        async def recv_loop():
            """⑥ stream_boost 명령 수신 → 부스트 모드 전환."""
            global boost_mode
            async for raw in ws:
                data = json.loads(raw)
                if data.get("event_type") == "stream_boost":
                    boost_mode = (data.get("action") == "start")
                    mode = "부스트(15fps)" if boost_mode else "평상시(5fps)"
                    print(f"← 🎥 stream_boost {data.get('action')} 수신 → {mode} 전환")

        recv_task = asyncio.create_task(recv_loop())
        await send_loop()
        recv_task.cancel()

    print("🚗 [가짜 젯슨] JSON 채널 종료")


async def stream_channel(server: str, duration: float):
    """⑤ 가짜 JPEG 프레임을 부스트 모드에 따라 5fps/15fps로 전송."""
    async with websockets.connect(f"{server}/ws/jetson-stream") as ws:
        print("🎥 [가짜 젯슨] 영상 채널 접속 완료")

        start = time.monotonic()
        sent = 0
        while time.monotonic() - start < duration:
            if boost_mode:
                await ws.send(FAKE_FRAME_BOOST)
                await asyncio.sleep(1 / 15)  # 15fps
            else:
                await ws.send(FAKE_FRAME_NORMAL)
                await asyncio.sleep(1 / 5)   # 5fps
            sent += 1

        print(f"🎥 [가짜 젯슨] 영상 채널 종료 (총 {sent}프레임 전송)")


async def main():
    parser = argparse.ArgumentParser(description="가짜 젯슨 (통신 스펙 v1.2)")
    parser.add_argument("--server", default="ws://127.0.0.1:8000",
                        help="백엔드 서버 주소 (기본: ws://127.0.0.1:8000)")
    parser.add_argument("--duration", type=float, default=12.0,
                        help="실행 시간(초), 기본 12초")
    args = parser.parse_args()

    await asyncio.gather(
        json_channel(args.server, args.duration),
        stream_channel(args.server, args.duration),
    )


if __name__ == "__main__":
    asyncio.run(main())
