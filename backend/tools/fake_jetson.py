"""
fake_jetson.py — 가짜 젯슨 (목업)

실물 젯슨/RC카 없이 백엔드 서버를 테스트하기 위한 스크립트.
docs/interface.md 의 젯슨 역할을 흉내 낸다:

    ①   position       : 0.5초마다 위치 핑 (배 주위를 도는 가짜 궤적)
    ②   위험 이벤트     : 2초 fallen_person, 4초 fire, 6초 no_helmet
    ②-b event_cleared  : 10초에 fallen_person 을 치웠다고 통보
    ②-c event_snapshot : 위험 이벤트 **직후** 그 순간 사진 (base64)
    ③   block_level    : 접속 직후 현재 단계(2) 1번
    ④   ship_pose      : 접속 직후 배 위치 측량 결과 1번

영상은 이 스크립트가 흉내 내지 않는다 — 실물에서도 영상은 백엔드를 거치지 않고
서버의 mediamtx 가 받아 브라우저로 보낸다 (docs/interface.md ⑤).

좌표는 실제 값의 축척을 따른다. 배는 0.77 m × 0.14 m 짜리 모형이므로,
이벤트를 배에서 몇 미터 떨어뜨려 놓으면 대시보드가 전부 "배 밖"으로 판정해서
구획에 핑이 안 붙는다. 그래서 위험 이벤트를 배 위/배 근처 센티미터 단위로 둔다.

실행 방법 (백엔드 venv 사용):
    cd backend
    venv/bin/python tools/fake_jetson.py                     # 기본: 127.0.0.1:8000, 12초
    venv/bin/python tools/fake_jetson.py --duration 60       # 60초 동안
    venv/bin/python tools/fake_jetson.py --server ws://192.168.0.100:8000
"""

import argparse
import asyncio
import base64
import json
import math
import time

import websockets

# 배 중심과 뱃머리 방향.
# yaw=0 = 뱃머리가 map +x — docs/interface.md ④ 의 운영 규칙("배는 항상 map +x
# 방향으로 놓는다")을 그대로 따른다. 1.57(90도)로 두면 실물 규칙과 어긋난다.
SHIP_XY = (0.0, 0.0)
SHIP_YAW = 0.0
SHIP_LENGTH_M = 0.77          # 프론트의 SHIP_REAL_LENGTH_M 과 같은 값

# 가짜 스냅샷: 16×16 짜리 진짜 JPEG. 서버가 base64 를 풀어 파일로 떨구므로
# 아무 바이트나 쓰면 브라우저에서 깨진 이미지가 된다.
FAKE_JPEG = base64.b64decode(
    "/9j/4AAQSkZJRgABAQEAYABgAAD/2wBDAAgGBgcGBQgHBwcJCQgKDBQNDAsLDBkSEw8UHRofHh0a"
    "HBwgJC4nICIsIxwcKDcpLDAxNDQ0Hyc5PTgyPC4zNDL/wAALCAAQABABAREA/8QAHwAAAQUBAQEB"
    "AQEAAAAAAAAAAAECAwQFBgcICQoL/8QAtRAAAgEDAwIEAwUFBAQAAAF9AQIDAAQRBRIhMUEGE1Fh"
    "ByJxFDKBkaEII0KxwRVS0fAkM2JyggkKFhcYGRolJicoKSo0NTY3ODk6Q0RFRkdISUpTVFVWV1hZ"
    "WmNkZWZnaGlqc3R1dnd4eXqDhIWGh4iJipKTlJWWl5iZmqKjpKWmp6ipqrKztLW2t7i5usLDxMXG"
    "x8jJytLT1NXW19jZ2uHi4+Tl5ufo6erx8vP09fb3+Pn6/9oACAEBAAA/APn+iiiv/9k=")

# 어떤 위험을 언제, 배의 어디에 띄울지.
#   (보낼 시각[초], 클래스, 배 중심 기준 [앞뒤, 좌우] 오프셋 m, confidence)
# 앞뒤 +가 뱃머리 쪽. **좌우 +가 좌현(왼쪽)** —— map 이 오른손 좌표계라
# 뱃머리(+x)를 볼 때 +y 가 왼쪽이다 (docs/interface.md ④ "반시계").
DANGER_SCRIPT = [
    (2.0, "fallen_person", (+0.10, +0.04), 0.91),   # 배 위, 중앙 살짝 앞 좌현
    (4.0, "fire",          (-0.20, -0.05), 0.87),   # 배 위, 선미 쪽 우현
    (6.0, "no_helmet",     (+0.55, +0.06), 0.78),   # 뱃머리를 넘어선 자리(좌현)
]

# 몇 초에 어느 이벤트를 "치워졌다"고 통보할지 (②-b)
CLEAR_AT = (10.0, "fallen_person")


def event_id_of(cls: str, xy) -> str:
    """docs/interface.md ② 의 규칙 그대로: "<class>@<x>,<y>" (소수 2자리).

    ②-b·②-c 가 이 문자열로 이벤트를 가리키므로 글자 단위로 같아야 한다.
    """
    return f"{cls}@{xy[0]:.2f},{xy[1]:.2f}"


def ship_local_to_map(offset):
    """배 기준 (앞뒤, 좌우) 오프셋을 map 절대좌표로."""
    fwd, beam = offset
    cos, sin = math.cos(SHIP_YAW), math.sin(SHIP_YAW)
    return (round(SHIP_XY[0] + fwd * cos - beam * sin, 2),
            round(SHIP_XY[1] + fwd * sin + beam * cos, 2))


async def json_channel(server: str, duration: float):
    """①~④ 를 전송하는 JSON 채널."""
    async with websockets.connect(f"{server}/ws/jetson") as ws:
        print("🚗 [가짜 젯슨] JSON 채널 접속 완료")

        # --- 접속 직후 1번: 현재 조립 단계(③) + 배 위치 측량 결과(④) ---
        await ws.send(json.dumps(
            {"event_type": "block_level", "block_id": "B1", "level": 2}))
        print("→ ③ block_level 전송 (level=2)")

        await ws.send(json.dumps(
            {"event_type": "ship_pose", "block_id": "B1",
             "map_xy": list(SHIP_XY), "yaw": SHIP_YAW}))
        print(f"→ ④ ship_pose 전송 (map_xy={list(SHIP_XY)}, yaw={SHIP_YAW} = 뱃머리 +x)")

        start = time.monotonic()
        done = set()
        cleared = False

        while (t := time.monotonic() - start) < duration:
            # ① 위치 핑 — 배 주위 반경 0.6m 를 도는 가짜 순찰 궤적.
            #   배(0.77×0.14m)를 도는 실제 순찰 반경과 비슷한 크기로 맞춘다.
            ekf = [round(SHIP_XY[0] + 0.6 * math.cos(t * 0.5), 2),
                   round(SHIP_XY[1] + 0.6 * math.sin(t * 0.5), 2)]
            await ws.send(json.dumps({"event_type": "position", "ekf_global": ekf}))

            # ② 위험 이벤트 + ②-c 그 순간 사진
            for at, cls, offset, conf in DANGER_SCRIPT:
                if t < at or cls in done:
                    continue
                done.add(cls)
                map_xy = ship_local_to_map(offset)
                eid = event_id_of(cls, map_xy)

                await ws.send(json.dumps({
                    "event_type": cls,
                    "confidence": conf,
                    "map_xy": list(map_xy),
                    "event_id": eid,
                    "ekf_global": ekf,
                }))
                print(f"→ ② 🚨 {cls} 전송 (map_xy={list(map_xy)}, event_id={eid})")

                # ②-c 는 위험 이벤트 **직후** 별도 메시지로, 같은 event_id 로 온다.
                # 서버는 base64 를 파일로 떨구고 프론트에는 image_url 만 보낸다.
                await ws.send(json.dumps({
                    "event_type": "event_snapshot",
                    "block_id": "B1",
                    "event_id": eid,
                    "cls": cls,
                    "image_b64": base64.b64encode(FAKE_JPEG).decode(),
                }))
                print(f"→ ②-c 📸 event_snapshot 전송 ({len(FAKE_JPEG)}B)")

            # ②-b 치워짐 — 로봇이 그 자리를 다시 지나갔는데 안 보이더라는 통보.
            # 프론트는 이 event_id 의 핑을 지우고, 서버는 복원 대상에서 뺀다.
            at, cls = CLEAR_AT
            if t >= at and not cleared and cls in done:
                cleared = True
                offset = next(o for _, c, o, _ in DANGER_SCRIPT if c == cls)
                map_xy = ship_local_to_map(offset)
                await ws.send(json.dumps({
                    "event_type": "event_cleared",
                    "block_id": "B1",
                    "cls": cls,
                    "map_xy": list(map_xy),
                    "event_id": event_id_of(cls, map_xy),
                }))
                print(f"→ ②-b 🧹 event_cleared 전송 ({cls} — 핑이 사라져야 정상)")

            await asyncio.sleep(0.5)

    print("🚗 [가짜 젯슨] JSON 채널 종료")


async def main():
    parser = argparse.ArgumentParser(description="가짜 젯슨")
    parser.add_argument("--server", default="ws://127.0.0.1:8000",
                        help="백엔드 서버 주소 (기본: ws://127.0.0.1:8000)")
    parser.add_argument("--duration", type=float, default=12.0,
                        help="실행 시간(초), 기본 12초")
    args = parser.parse_args()

    await json_channel(args.server, args.duration)


if __name__ == "__main__":
    asyncio.run(main())
