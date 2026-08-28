"""
main.py — 스마트 조선소 FastAPI 백엔드 서버

프로젝트   : 스마트 조선소 선박 건조 공정 트래킹 및 디지털 트윈 관제 시스템
모듈 설명  : 젯슨(UGV) ↔ 서버 ↔ 프론트엔드 간 실시간 이벤트 중계 서버.
            - WebSocket으로 젯슨의 실시간 센서/AI 감지 이벤트를 수신하여
              프론트엔드(React+Three.js 대시보드)로 즉시 브로드캐스트
            - 위험 이벤트 4종 + block_level + ship_pose는 MongoDB에 영구 로그 저장
            - WebRTC 시그널링(webrtc_signal) 쪽지를 프론트↔젯슨 양방향 중계
            - 감지 순간 스냅샷을 파일로 저장하고 /snapshots 로 서빙
            - REST API로 대시보드 초기 로딩 데이터 및 과거 이벤트 이력 제공

웹소켓 채널 2개 (통신 스펙 = docs/interface.md 참조):
    /ws/jetson           젯슨 JSON 채널 (이벤트 수신 + 서버→젯슨 쪽지 송신)
    /ws/frontend         프론트 JSON 채널 (이벤트 브로드캐스트 + 쪽지 수신)

★ 영상은 이 FastAPI 프로세스를 거치지 않는다.
  젯슨이 H.264 로 한 번만 인코딩해 rtsp://192.168.0.5:8554/ugv1 로 밀어올리면,
  같은 노트북에서 도는 **별개 프로세스 mediamtx** 가 받아 브라우저들에게 WebRTC 로
  뿌린다. 이 서버는 영상 바이트를 아예 만지지 않는다.
  설치·방화벽은 server/streaming/README.md, 배경은 docs/interface.md ⑤ 참조.

  ※ mediamtx 가 UDP 8000 을 잡는다. 이 서버는 TCP 8000 이라 충돌하지 않는다.

작성자     : 이정기 (Backend & Streaming Engineer)
작성일     : 2026-07-06
최근 수정일 : 2026-07-10

의존성     : Python 3.10+, FastAPI 0.138.2, motor 3.7.1 (requirements.txt 참조)
실행 방법  : venv/bin/uvicorn main:app --host 0.0.0.0 --port 8000
             (--host 0.0.0.0 필수. 생략하면 이 노트북에서만 접속됨)
환경 변수  : .env 파일에 MONGO_URL 필요 (.env.example 참조, CONTRIBUTING.md 참고)

탐지 이벤트 (2026-07-09 팀 확정 — 위험 이벤트는 YOLO 클래스 이름과 동일):
    - ship_defect   : 선박(블록) 결함 — 모델은 추가 학습 예정, 이름만 선확정
    - no_helmet     : 안전모 미착용
    - fallen_person : 작업자 쓰러짐
    - fire          : 화재
    - block_level   : 선박 블록 조립 단계 변화 — 단계가 '바뀔 때만' 젯슨이 전송.
                      단계 숫자는 이름이 아니라 level 필드에 담는다.
                      (예: {"event_type": "block_level", "block_id": "B1", "level": 2})
    - ship_pose     : 배 위치 측량 결과 — 세션 시작 + 조립 단계 변경 시마다 젯슨이 전송.
                      (예: {"event_type": "ship_pose", "block_id": "B1",
                            "map_xy": [5.1, 4.8], "yaw": 1.57})
"""

import asyncio
# 젯슨이 보내는 스냅샷은 base64 문자열이라 원래 바이트로 되돌려야 하고,
# 파일 이름은 event_id 를 해시해서 만든다 (아래 [이벤트 스냅샷 구역] 참조).
import base64
import hashlib
import json
import os
# python-dotenv 패키지: .env 파일에 적힌 키=값 쌍을 읽어서
# os.environ(환경변수)에 등록해주는 역할. 아래 load_dotenv() 호출과 짝을 이룸.
from dotenv import load_dotenv

# FastAPI 핵심 클래스와, 웹소켓 연결 객체 / 연결 끊김 예외를 가져옴
from fastapi import FastAPI, WebSocket, WebSocketDisconnect

# 다른 도메인(프론트엔드)에서의 API 요청을 허용해주는 미들웨어
from fastapi.middleware.cors import CORSMiddleware

# 저장해둔 스냅샷 사진을 그냥 URL 로 꺼내 쓸 수 있게 해주는 정적 파일 서빙
from fastapi.staticfiles import StaticFiles

# MongoDB를 비동기(async)로 다루기 위한 motor 라이브러리의 클라이언트
from motor.motor_asyncio import AsyncIOMotorClient

# 타입 힌트용 List/Optional (연결 목록, 젯슨 단일 연결 참조 타입 표기에 사용)
from collections import deque
from contextlib import asynccontextmanager
from typing import List, Optional

# 이벤트 저장 시각을 기록하기 위한 datetime (시간대 명시 기록용 timedelta/timezone 포함)
from datetime import datetime, timedelta, timezone

# .env 파일(금고)에 적힌 값들을 실제로 읽어들여 환경변수로 등록.
# 이 줄이 없으면 아래 os.getenv("MONGO_URL")이 None을 반환함.
load_dotenv()

# =========================================================
# [이벤트 타입 상수 정의 구역]
# 팀이 합의한 이벤트만 DB에 영구 저장 대상으로 취급한다.
# 여기 한 곳만 고치면 아래 로직 전체에 반영되도록 상수로 분리함.
# 위험 이벤트 이름은 YOLO 모델 클래스 이름과 동일하게 맞춤 (2026-07-09 확정).
# =========================================================
SHIP_DEFECT = "ship_defect"      # 선박(블록) 결함 — 모델 추가 학습 예정
NO_HELMET = "no_helmet"          # 안전모 미착용
FALLEN_PERSON = "fallen_person"  # 작업자 쓰러짐
FIRE = "fire"                    # 화재
BLOCK_LEVEL = "block_level"      # 블록 조립 단계 변화 (block_id, level 필드 포함)
SHIP_POSE = "ship_pose"          # 배 위치 측량 결과 (block_id, map_xy, yaw 필드 포함)
# 위험 이벤트가 치워졌음(더 이상 안 보임) 확인 (interface.md v1.6, 2026-08-21)
# block_id, cls, map_xy, event_id 필드 포함. cls 로 어느 위험 종류였는지 담는다
# (이 메시지 자체의 event_type 은 "event_cleared" 고정이라 종류를 따로 실어야 함).
EVENT_CLEARED = "event_cleared"

# 감지 순간의 사진 (젯슨→서버, 2026-08-28).
# 위험 이벤트 3종(fire/fallen_person/no_helmet)을 보낸 **직후 별도 메시지**로
# 오며, 그 이벤트와 event_id 가 정확히 같다. 필드: block_id, event_id, cls,
# image_b64(검출 박스 + 사방 40px 여백을 자른 JPEG, 보통 10~40KB).
# 중복 제거를 통과한 새 이벤트에만 오므로 같은 자리 불로 계속 오지 않는다.
#
# ⚠️ LOGGED_EVENT_TYPES 에 넣지 말 것 — 아래 [이벤트 스냅샷 구역]에 이유가 있다.
EVENT_SNAPSHOT = "event_snapshot"


# WebRTC 시그널링 쪽지 (영상 P2P 직결용, 양방향, DB 저장 대상 아님).
# payload 안의 내용(SDP/ICE)은 WebRTC 라이브러리가 자동 생성한 것 —
# 서버는 열어보지 않고 반대편에 그대로 배달만 한다.
# (예: {"event_type": "webrtc_signal", "payload": {...}})
WEBRTC_SIGNAL = "webrtc_signal"

# 위험 이벤트 확인(ack) — 관제자가 팝업의 "확인" 버튼을 눌렀을 때 프론트가 보낸다.
# 젯슨의 Nav2가 이 신호를 받으면 정지해 있던 자율주행을 재개한다.
#
# ★ "해결"이 아니라 "확인"이다.
#   화재 진화에는 오래 걸리는데 그동안 로봇이 묶여 있으면 다른 문제 상황을
#   즉각 발견할 수 없다. 관제자가 "봤다"고 알려주면 로봇은 순찰을 계속한다.
#   따라서 프론트 버튼 라벨도 "처리 완료"가 아니라 "확인"으로 한다.
#
# 서버는 내용을 판단하지 않고 그대로 배달만 하므로, 프론트가 어떤 이벤트를
# 확인했는지 등 부가 필드를 넣어 보내도 젯슨까지 그대로 전달된다.
# (예: {"event_type": "event_ack"})
EVENT_ACK = "event_ack"

# 이벤트 기억 초기화 (프론트 → 서버 → 젯슨, DB 저장 안 함).
#
# 유령 핑이나 엉뚱한 자리의 이벤트가 생겼을 때, 프로세스를 껐다 켜지 않고
# 화면과 로봇 기억을 한 번에 정리한다. 젯슨은 이걸 받으면 change_point 의
# 보고 이력과 재통보 거울을 비운다.
#
# ★ 서버도 같이 움직여야 한다.
#   젯슨만 비우면 서버는 계속 옛 이벤트를 복원한다. 아래 복원 기준선을
#   지금 시각으로 밀어야 새로고침해도 안 돌아온다.
#
# ★ DB 는 지우지 않는다.
#   기록은 증빙이다. "안 보이게 하는 것" 과 "지우는 것" 은 다르다.
#   기준선만 밀면 화면에서 사라지고, 기록은 감사용으로 남는다.
RESET_EVENTS = "reset_events"

# 로봇 연결 상태 알림 (서버 → 프론트, DB 저장 안 함).
#
# ★ 왜 필요한가 (2026-08-29)
#   로봇이 꺼져도 화면의 위험 핑은 그대로 남는다(위험이 사라진 게 아니니 맞는
#   동작이다). 문제는 **관제사가 그 사실을 모른다**는 것이다. 화면은 평소와
#   똑같은데 실시간 순찰 정보만 조용히 멈춘다. 그래서 서버가 알려준다.
#
#   {"event_type": "jetson_status", "connected": true/false}
JETSON_STATUS = "jetson_status"

# 프론트→서버→젯슨 방향으로 '그대로 전달'하는 메시지 종류 모음.
# (젯슨→프론트 방향은 기존 브로드캐스트가 모든 메시지를 전달하므로 목록 불필요)
JETSON_BOUND_TYPES = {WEBRTC_SIGNAL, EVENT_ACK, RESET_EVENTS}

# 이벤트 timestamp 기록용 한국 표준시.
# 시간대 정보 없는(naive) 시각은 환경마다 해석이 달라지므로 +09:00을 명시한다.
KST = timezone(timedelta(hours=9))

# DB 저장 대상 이벤트 목록 — 위험 이벤트 4종 + 공정 단계 변화 + 배 위치 측량.
# block_level/ship_pose는 '바뀔 때만' 오는 희소 이벤트라 저장량 부담이 없고,
# 최신 값을 init-data 상태 복원에 쓰므로 저장 대상에 포함.
LOGGED_EVENT_TYPES = {SHIP_DEFECT, NO_HELMET, FALLEN_PERSON, FIRE, BLOCK_LEVEL, SHIP_POSE, EVENT_CLEARED}
# ※ EVENT_SNAPSHOT 이 여기 없는 것은 빠뜨린 게 아니라 의도된 것이다.
#   사진은 파일로 저장하고 문서에는 URL 만 붙인다 ([이벤트 스냅샷 구역] 참조).

# =========================================================
# [서버 수명 주기 구역]
# 서버가 뜰 때 배경에서 계속 돌아야 하는 일을 등록하고,
# 내려갈 때 정리한다.
#
# @app.on_event("startup") 방식은 FastAPI 0.138 기준 폐기 예정이라
# (DeprecationWarning) 권장 방식인 lifespan 을 쓴다.
# =========================================================
@asynccontextmanager
async def lifespan(app: FastAPI):
    # 인터넷이 끊긴 동안 쌓인 저장 실패분을 주기적으로 다시 밀어 넣는 작업.
    # 아래 retry_failed_saves 는 DB 구역에 정의돼 있다.
    retry_task = asyncio.create_task(retry_failed_saves())
    print(f"🔁 [DB] 재시도 작업 시작 ({RETRY_INTERVAL_S}초 주기)")
    try:
        yield                       # 여기서 서버가 돌아간다
    finally:
        retry_task.cancel()
        try:
            await retry_task
        except asyncio.CancelledError:
            pass

        # ★ 진행 중인 저장 작업도 정리한다.
        #   백그라운드로 던진 작업(_save_tasks)을 그냥 두고 종료하면
        #   "Task was destroyed but it is pending!" 경고가 뜨고, 저장이
        #   중간에 잘려 이벤트가 소리 없이 사라질 수 있다.
        #
        #   짧게 기다려주되(2초) 무한정 붙들지는 않는다. DB 가 불통이면
        #   각 작업이 타임아웃(5초)까지 걸릴 수 있는데, 그 때문에 서버 종료가
        #   느려지면 Ctrl+C 가 안 먹는 것처럼 보인다.
        if _save_tasks:
            pending = list(_save_tasks)
            print(f"⏳ [DB] 진행 중인 저장 {len(pending)}건 마무리 대기 (최대 2초)")
            done, still = await asyncio.wait(pending, timeout=2.0)
            for t in still:
                t.cancel()
            if still:
                print(f"   {len(still)}건은 시간 내 못 끝나 취소함")

        if failed_saves:
            print(f"⚠️ [DB] 저장하지 못한 이벤트 {len(failed_saves)}건이 "
                  f"남은 채로 서버가 종료된다 (메모리 큐라 사라진다)")


# FastAPI 서버 객체 생성 (우리의 백엔드 서버 본체).
# title/description/version은 자동 생성되는 API 문서(/docs)에 표시됨.
app = FastAPI(
    title="Smart Shipyard Digital Twin Backend",
    description="젯슨 UGV ↔ 서버 ↔ 프론트엔드 실시간 이벤트 중계 API",
    version="0.1.0",
    lifespan=lifespan,
)

# =========================================================
# [CORS 설정 구역]
# React 프론트엔드(다른 포트/도메인)가 이 서버에 접근할 수 있도록 허용.
# TODO: 배포 시 allow_origins를 실제 프론트엔드 도메인으로 제한할 것
#       (지금은 "*"라서 개발 단계 전용, 운영 배포 전 반드시 수정)
# =========================================================
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],       # 모든 출처 허용 (개발용, 배포 시 제한 필요)
    allow_credentials=True,    # 쿠키/인증 정보 포함 요청 허용
    allow_methods=["*"],       # GET/POST 등 모든 HTTP 메서드 허용
    allow_headers=["*"],       # 모든 요청 헤더 허용
)

# =========================================================
# [스냅샷 정적 서빙 구역]
# 저장해둔 감지 사진을 브라우저가 그냥 주소로 꺼내 볼 수 있게 한다.
#   http://192.168.0.5:8000/snapshots/<해시>.jpg
# =========================================================

# 서버를 어느 폴더에서 띄우든 항상 backend/snapshots 를 가리키게 절대경로로 잡는다.
# (상대경로 "snapshots" 로 두면 uvicorn 을 다른 폴더에서 실행했을 때 엉뚱한 곳에 쌓인다)
SNAPSHOT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "snapshots")

# ★ 이 한 줄이 없으면 바로 아래 StaticFiles 가 폴더를 못 찾고 **서버가 시작조차 못 한다.**
#   .gitignore 가 사진만 막고 .gitkeep 으로 폴더는 남기지만, 누가 폴더를 지우거나
#   새 환경에서 처음 띄우는 경우까지 여기서 막아준다.
os.makedirs(SNAPSHOT_DIR, exist_ok=True)

app.mount("/snapshots", StaticFiles(directory=SNAPSHOT_DIR), name="snapshots")

# =========================================================
# [데이터베이스 셋업 구역]
# MongoDB Atlas와 통신하는 선을 연결하는 곳.
# =========================================================

# .env 파일에 적어둔 MONGO_URL 값을 읽어옴 (예: mongodb+srv://...).
# 값이 없으면 None이 반환되며, 이 경우 아래 client 생성 시 접속 실패로 이어짐.
MONGO_URL = os.getenv("MONGO_URL")

# 비동기 방식으로 MongoDB에 접속하는 클라이언트 객체 생성.
#
# ★ 타임아웃을 반드시 지정한다 (기본값은 30초).
#   MongoDB Atlas는 클라우드라 인터넷이 필요하다. 시연장 와이파이가 불안정하면
#   저장 시도가 기본 30초를 붙들고 있다가 실패한다. 그동안 그 작업이 살아 있어
#   이벤트가 계속 오면 대기 중인 작업이 수십 개로 불어난다.
#
#   5초로 정한 이유:
#     - mongodb+srv 는 접속할 때 DNS SRV 조회 + TLS 악수를 한다. 인터넷이
#       '느리지만 살아있는' 상태면 정상 저장에도 2~3초가 걸릴 수 있어,
#       3초로 자르면 성공할 수 있었던 저장을 실패로 만든다.
#     - 반대로 30초는 실패 판정이 너무 늦다.
#   실패해도 아래 재시도 큐가 받아주므로 값이 조금 어긋나도 손해가 작다.
client = AsyncIOMotorClient(
    MONGO_URL,
    serverSelectionTimeoutMS=5000,   # 쓸 수 있는 서버를 찾는 데 쓸 최대 시간
    connectTimeoutMS=5000,           # TCP/TLS 연결 수립 최대 시간
    socketTimeoutMS=10000,           # 연결된 뒤 응답을 기다리는 최대 시간
)

# 'shipyard_db'라는 이름의 데이터베이스를 선택 (없으면 첫 데이터 삽입 시 자동 생성됨).
db = client.shipyard_db

# 그 안의 'events' 컬렉션(=서류함)을 선택. 여기에 4종 이벤트 로그가 쌓임.
event_collection = db.events


# 위험 이벤트 4종 — 대시보드 재접속 시 되살릴 대상.
DANGER_TYPES = {SHIP_DEFECT, NO_HELMET, FALLEN_PERSON, FIRE}

# 이 서버 프로세스가 켜진 시각. 복원은 이 시각 이후에 들어온 것만 한다.
#
# ★ 왜 필요한가 (2026-08-27)
#   "살아있다"의 판정은 event_cleared 가 오는지에 전적으로 기대고 있는데,
#   그 신호가 한 번이라도 안 오면 그 이벤트는 **영원히 살아있는 것으로 남는다.**
#   실제로 이 제한을 넣기 전 DB 에는 복원 대상이 56건 있었고, 그중 46건이 하루
#   이상 된 것이었다. 옛 좌표의 불을 로봇이 다시 지나갈 일이 없으니
#   event_cleared 가 올 수 없고, 젯슨 쪽 clear 로직을 고쳐도 이미 쌓인 것은
#   그대로 남는다.
#
#   서버를 다시 켰다는 것은 "여기서부터 새로 본다"는 뜻이므로 그 시각을
#   바닥으로 깐다. 이전 세션의 찌꺼기가 올라올 여지가 아예 없어진다.
#
#   이것은 clear 가 깨졌을 때 화면이 무너지지 않게 막아주는 안전장치이지,
#   clear 를 대신하는 장치가 아니다. clear 가 안 되면 대시보드보다 로봇이 더
#   문제다 — 같은 자리 불로 계속 멈추기 때문이다.
#
#   ⚠️ 반드시 KST 로 잡는다 (2026-08-28 버그 수정).
#     아래 비교는 ISO 문자열의 사전순 비교인데, 문자열 비교는 시간대 표기를
#     읽지 않는다. 이 값을 UTC 로 만들면 "22:23+09:00" >= "14:23+00:00" 이
#     참이 되어 통과한다 — 실제로는 22:23 KST = 13:23 UTC 로 서버가 켜지기
#     전인데도 그렇다. 즉 바닥이 9시간 뒤로 밀려, 서버 켜기 직전 9시간의
#     이벤트가 전부 되살아났다. 대시보드에 유령 핑이 뜬 원인이 이것이다.
#
#     DB 의 timestamp 는 datetime.now(KST) 로 쓰므로 바닥도 같은 시간대여야
#     문자열 비교가 곧 시간 비교가 된다.
# 이 시각보다 오래된 이벤트는 복원하지 않는다.
#
# 이름이 "서버 시작 시각" 이 아닌 이유 — 서버가 켜질 때뿐 아니라
# 프론트의 [초기화](reset_events)로도 지금 시각으로 밀린다. 즉 "복원의 바닥"
# 이지 "프로세스가 뜬 시각" 이 아니다.
RESTORE_FLOOR_AT = datetime.now(KST)

# 젯슨이 "이건 아직 살아있다"고 알려준 이벤트 id 들.
#
# ★ 왜 필요한가 (2026-08-29)
#   젯슨은 서버에 (재)연결할 때 지금 살아있는 위험을 replay:true 로 다시 보낸다.
#   그 메시지는 **DB에 저장하지 않는다** — 이미 있는 이벤트의 중복 기록이고,
#   실제로 그 시각에 감지된 것도 아니라 감사 기록을 흐린다.
#
#   그런데 저장을 안 하면 구멍이 하나 생긴다. 서버를 재시작한 뒤라면 그 이벤트의
#   원래 DB 기록은 RESTORE_FLOOR_AT 아래에 있어 복원 대상이 아니다. 그래서
#   젯슨 재통보로 핑은 떴는데 **브라우저를 새로고침하면 사라진다.**
#
#   그래서 저장하는 대신 "젯슨이 살아있다고 한 것" 을 여기 기억해두고, 복원할 때
#   시각 바닥과 무관하게 함께 꺼내온다. DB 는 그대로 두고 판정만 넓히는 것이다.
#
#   젯슨이 이 판정의 주인이다 — 로봇이 그 자리를 다시 보고 확인하므로 며칠 전
#   기록보다 훨씬 믿을 만하다. 젯슨 연결이 새로 열리면 통째로 비우고 다시 채운다.
#
#   ⚠️ 다만 이것만 믿으면 안 된다 (2026-08-29).
#     이 목록은 젯슨이 재연결할 때마다 비워지고 그 연결의 재통보로 다시 찬다.
#     젯슨 소켓이 한 번 끊겼다 붙는 사이에 대시보드를 새로고침하면 목록이 비어
#     있어 핑이 통째로 사라진다. 그래서 _backfill_replay_event 가 이번 세션의
#     DB 기록도 한 건 남긴다 — 복원이 시각만으로도 성립하게 하는 안전장치다.
jetson_live_event_ids: set = set()

# 지금 백필이 돌고 있는 event_id.
#
# ★ 왜 필요한가 (2026-08-29)
#   _backfill_replay_event 는 "있나 확인 → 없으면 넣기" 인데 그 사이가 비어 있다.
#   재통보는 **묶음으로** 온다(젯슨이 복원한 것을 재연결 때 한꺼번에 올린다).
#   젯슨이 짧은 간격으로 두 번 붙으면 같은 event_id 의 백필 두 개가 동시에 돌아
#   둘 다 "없음" 을 보고 각각 넣는다 —— 중복 기록이 생긴다.
#
#   여기 넣어두고 끝나면 뺀다. 이미 돌고 있으면 두 번째는 그냥 넘어간다.
_backfill_inflight: set = set()


async def _backfill_replay_event(doc: dict):
    """재통보로만 알게 된 이벤트를 처음 한 번 기록한다.

    이미 같은 event_id 의 위험 기록이 있으면 아무것도 하지 않는다 —
    중복 저장은 감사 기록을 흐리고, 복원은 어차피 기존 문서로 된다.

    저장할 때 replay 플래그를 지우지 않는다. "이 기록은 재통보로 알게 된
    것" 이라는 사실을 남겨야 나중에 감사할 때 감지 시각을 오해하지 않는다.
    """
    eid = doc.get("event_id")
    if eid in _backfill_inflight:
        return              # 같은 이벤트를 이미 넣는 중이다
    _backfill_inflight.add(eid)
    try:
        # ★ "아예 없을 때" 가 아니라 "이번 세션 기록이 없을 때" 저장한다
        #   (2026-08-29 수정).
        #
        #   원래는 같은 event_id 기록이 하나라도 있으면 건너뛰었다. 그러면 그
        #   이벤트는 복원될 때 **메모리의 jetson_live_event_ids 에만** 기대게 되는데,
        #   그 목록은 젯슨이 재연결할 때마다 비워진다. 젯슨 소켓이 한 번만 끊겼다
        #   붙어도(재연결 루프가 있다) 목록이 날아가고, 그 뒤 대시보드를
        #   새로고침하면 **살아있는 위험 핑이 통째로 사라졌다.**
        #
        #   이번 세션(RESTORE_FLOOR_AT 이후) 기록을 한 번 남겨두면 복원이
        #   시각만으로 성립해서, 메모리 상태가 어떻든 흔들리지 않는다.
        #   세션당 event_id 하나에 최대 한 건이라 무한히 쌓이지도 않는다.
        exists = await event_collection.find_one(
            {"event_id": eid,
             "event_type": {"$in": list(DANGER_TYPES)},
             "timestamp": {"$gte": RESTORE_FLOOR_AT.isoformat()}},
            {"_id": 1},
        )
        if exists:
            return
        doc["timestamp"] = datetime.now(KST).isoformat()
        await event_collection.insert_one(doc)
        print(f"💾 [재통보] 이번 세션 기록으로 남김: {eid}")
    except Exception as e:
        print(f"⚠️ [재통보] 기록 실패({type(e).__name__}): {eid}")
    finally:
        _backfill_inflight.discard(eid)


def schedule_replay_backfill(doc: dict):
    """DB 를 기다리지 않는다 — 젯슨 수신 루프가 멈추면 안 된다."""
    task = asyncio.create_task(_backfill_replay_event(doc))
    _save_tasks.add(task)
    task.add_done_callback(_save_tasks.discard)


async def broadcast_jetson_status():
    """로봇 연결 상태를 프론트 전체에 알린다. DB에 저장하지 않는다."""
    await manager.broadcast({
        "event_type": JETSON_STATUS,
        "connected": jetson_connection is not None,
    })


async def _active_danger_events(limit: int = 300):
    """지금도 살아있는 위험 이벤트를 오래된 순으로 돌려준다.

    '살아있다' = 등록된 뒤 같은 event_id 로 event_cleared 가 오지 않았다는 뜻.
    젯슨의 change_point_detector 가 "로봇이 그 자리를 다시 지나갔는데 안 보인다"
    를 확인해야만 event_cleared 를 보내므로, 이 판정은 젯슨의 판단을 그대로
    따르는 것이다. 서버가 따로 시간 만료 같은 규칙을 두지 않는 이유다.

    최신 limit 건만 훑는다. 그보다 오래된 것이 아직 살아있을 가능성은 낮고,
    전체 스캔은 접속할 때마다 도는 경로라 비용을 묶어두는 편이 안전하다.
    """
    # RESTORE_FLOOR_AT 도 KST 라 timestamp 와 표기가 같다 → 사전순 = 시간순.
    cutoff = RESTORE_FLOOR_AT.isoformat()
    docs = await event_collection.find({
        "event_type": {"$in": list(DANGER_TYPES) + [EVENT_CLEARED]},
        "$or": [
            # timestamp 는 서버가 저장할 때 붙이는 ISO 문자열이라 사전순 비교가
            # 곧 시간순 비교다. 필드가 없는 구버전 문서는 여기서 함께 걸러진다.
            {"timestamp": {"$gte": cutoff}},
            # 서버가 켜지기 전 기록이어도, 젯슨이 아직 살아있다고 한 것은 꺼낸다.
            {"event_id": {"$in": list(jetson_live_event_ids)}},
        ],
    }).sort("_id", -1).to_list(length=limit)
    docs.reverse()                      # 오래된 것부터 훑어야 등록/해제 순서가 맞다

    alive = {}
    for d in docs:
        eid = d.get("event_id")
        if not eid:
            continue                    # event_id 없는 구버전 문서는 짝을 못 지어 건너뛴다
        if d["event_type"] == EVENT_CLEARED:
            alive.pop(eid, None)
        else:
            alive[eid] = d
    return list(alive.values())


# =========================================================
# [DB 저장 구역 — 알림 경로에서 DB를 완전히 분리한다]
# =========================================================
#
# ★ 왜 이렇게 하는가
#
#   원래는 젯슨 메시지를 받는 루프 안에서 곧바로 insert_one 을 await 했다.
#   그러면 인터넷이 끊겼을 때 이런 일이 벌어진다:
#
#       젯슨: "화재 신고"  ->  서버가 받음
#       서버: 몽고에 저장 시도  ->  인터넷 끊김  ->  타임아웃까지 대기
#                                  그동안 다음 메시지를 못 받는다
#       타임아웃 후: 예외 발생  ->  except WebSocketDisconnect 로는 안 잡힘
#                                  -> 핸들러가 끝나며 웹소켓이 닫힌다
#       젯슨: 재접속  ->  다음 이벤트에서 똑같이 반복
#
#   더 나쁜 것은 broadcast() 가 insert_one **다음 줄**이라, DB가 실패하면
#   프론트 실시간 알림까지 안 나갔다는 점이다. DB는 통계용 과거 기록인데
#   그것 때문에 안전 알림이 막히는 것은 우선순위가 뒤집힌 것이다.
#
#   그래서 저장을 **백그라운드 작업**으로 떼어냈다. 메인 루프는 저장을
#   시켜놓고 즉시 broadcast() 로 넘어간다. 인터넷이 아예 죽어도 프론트
#   알림 지연은 0이다.
#
#   저장에 실패한 것은 버리지 않고 재시도 큐에 넣어, 인터넷이 돌아오면
#   자동으로 밀어 넣는다.
#
# ※ 프론트/젯슨이 접속하는 주소·형식은 전혀 바뀌지 않는다. 서버 안에서
#   일하는 순서만 달라진 것이다.

# 저장 실패분을 담아두는 대기줄. 인터넷이 오래 끊겨도 메모리가 계속 불어나지
# 않도록 상한을 둔다. maxlen 을 넘기면 **가장 오래된 것부터** 자동으로 밀려난다.
# (최신 이벤트가 더 중요하므로 오래된 쪽을 버리는 것이 맞다)
FAILED_SAVE_MAX = 200
failed_saves: deque = deque(maxlen=FAILED_SAVE_MAX)

# 재시도 주기(초). 너무 짧으면 인터넷이 끊긴 동안 로그가 폭주하고,
# 너무 길면 복구가 늦다.
RETRY_INTERVAL_S = 30

# asyncio.create_task 로 만든 작업은 참조를 붙들지 않으면 실행 도중
# 가비지 컬렉션될 수 있다(파이썬 공식 문서 경고). 그래서 집합에 담아두고
# 끝나면 스스로 빠지게 한다.
_save_tasks: set = set()


async def save_event_background(doc: dict):
    """이벤트 하나를 몽고에 저장한다. 실패해도 예외를 밖으로 내보내지 않는다.

    이 함수는 백그라운드 작업으로 실행되므로, 여기서 예외가 새어나가면
    잡아줄 곳이 없다. 반드시 안에서 처리한다.
    """
    try:
        await event_collection.insert_one(doc)
        print(f"💾 몽고DB에 이벤트 저장 완료: {doc.get('event_type')}")
    except Exception as e:
        failed_saves.append(doc)
        print(f"⚠️ [DB] 저장 실패({type(e).__name__}) — 재시도 대기줄에 넣음 "
              f"({len(failed_saves)}/{FAILED_SAVE_MAX}): {doc.get('event_type')}")


def schedule_save(doc: dict):
    """저장을 백그라운드로 던지고 즉시 돌아온다.

    호출한 쪽(젯슨 메시지 루프)은 DB를 기다리지 않는다.
    """
    task = asyncio.create_task(save_event_background(doc))
    _save_tasks.add(task)
    task.add_done_callback(_save_tasks.discard)


async def retry_failed_saves():
    """인터넷이 돌아왔는지 주기적으로 확인하며 밀린 저장을 처리한다.

    한 번에 전부 시도하지 않고 대기줄을 앞에서부터 비워나간다. 도중에 다시
    실패하면 그 항목을 되돌려 넣고 멈춘다 — 아직 인터넷이 안 됐다는 뜻이므로
    나머지를 시도해봐야 시간만 버린다.
    """
    while True:
        await asyncio.sleep(RETRY_INTERVAL_S)
        if not failed_saves:
            continue

        pending = len(failed_saves)
        print(f"🔁 [DB] 밀린 저장 {pending}건 재시도")
        saved = 0
        while failed_saves:
            doc = failed_saves.popleft()
            try:
                await event_collection.insert_one(doc)
                saved += 1
            except Exception as e:
                # 아직 안 된다 — 꺼낸 것을 앞으로 되돌리고 다음 주기를 기다린다
                failed_saves.appendleft(doc)
                print(f"   아직 실패({type(e).__name__}). {saved}건 저장, "
                      f"{len(failed_saves)}건 남음")
                break
        if saved and not failed_saves:
            print(f"   ✅ 밀린 저장 {saved}건 모두 완료")


# =========================================================
# [이벤트 스냅샷 구역 — 감지 순간의 사진]
# =========================================================
#
# 젯슨은 위험 이벤트를 보낸 **직후**, 같은 event_id 로 그 순간의 crop 사진을
# event_snapshot 메시지에 base64 로 실어 보낸다.
# (설계 근거: docs/이벤트_스냅샷_및_위치표기_요청.md)
#
# ★ base64 를 DB 에 넣지도, 프론트로 흘리지도 않는다.
#
#   - DB 에 넣으면: 문서가 통째로 커진다. 게다가 재접속 복원은 문서를 그대로
#     다시 보내는 구조라, 대시보드를 새로고침할 때마다 살아있는 이벤트의 사진이
#     **전부 다시 흐른다.** base64 는 원본 바이트보다 33% 크기까지 하다.
#   - 프론트로 릴레이하면: 같은 이유. URL 이면 브라우저가 캐시해 두 번째부터는
#     안 받지만, 메시지에 박힌 base64 는 매번 새로 받는다.
#
#   그래서 서버는 사진을 **파일로 떨어뜨리고 URL 만** 남긴다. 실시간 표시는
#   URL 만 담은 가벼운 메시지로, 재접속 복원은 이벤트 문서에 붙인 image_url 로.
#
# ⚠️ 사진에는 작업자 얼굴이 찍힐 수 있다. .gitignore 가 backend/snapshots/* 를
#    막고 있으니 절대 풀지 말 것.

# DB 갱신 재시도 간격(초). 왜 재시도가 필요한지는 _attach_snapshot_url 참조.
SNAPSHOT_ATTACH_RETRY_S = (0.5, 1.5, 3.0)


def _snapshot_name(event_id: str) -> str:
    """event_id 를 파일 이름으로 쓸 수 있는 형태로 바꾼다.

    event_id 는 'fire@0.63,-0.23' 처럼 @ 와 쉼표, 마이너스가 섞여 있어 그대로
    파일 이름에 쓰면 OS 와 URL 양쪽에서 말썽이 난다. 해시로 고정 길이 이름을
    만들면 그 문제가 사라지고, 같은 event_id 는 항상 같은 이름이 나오므로
    재전송이 와도 그냥 덮어쓰기가 된다(사진이 중복으로 쌓이지 않는다).
    """
    return hashlib.sha1(event_id.encode("utf-8")).hexdigest()[:16] + ".jpg"


def _write_snapshot_file(path: str, blob: bytes):
    """별도 스레드에서 돌 파일 쓰기 (아래 asyncio.to_thread 로 호출)."""
    with open(path, "wb") as f:
        f.write(blob)


async def _attach_snapshot_url(event_id: str, url: str):
    """짝이 되는 위험 이벤트 문서에 사진 URL 을 붙인다.

    ★ 새 문서를 만들지 않는다. 이미 저장된 그 이벤트에 필드 하나를 더하는 것이다.
      새로 만들면 복원 때 같은 위험이 두 번 뜬다.

    ★ 경쟁 상태를 견뎌야 한다.
      위험 이벤트 저장은 schedule_save 가 백그라운드로 돌리는데 젯슨은 그
      직후에 스냅샷을 보낸다. 그래서 이 갱신이 정작 저장보다 **먼저** 도착할
      수 있다. 짧게 몇 번 다시 시도하면 대부분 해결된다.

      끝내 못 붙어도 실시간 표시는 이미 끝난 상태고(브로드캐스트는 별개),
      재접속 복원에서만 그 한 건의 사진이 빠진다.
    """
    for delay in SNAPSHOT_ATTACH_RETRY_S:
        try:
            doc = await event_collection.find_one_and_update(
                {"event_id": event_id, "event_type": {"$in": list(DANGER_TYPES)}},
                {"$set": {"image_url": url}},
                # 같은 자리에서 치웠다가 다시 발생한 경우 문서가 여럿일 수 있다.
                # 최신 것에 붙여야 지금 살아있는 이벤트의 사진이 된다.
                sort=[("_id", -1)],
            )
            if doc is not None:
                return
        except Exception as e:
            print(f"⚠️ [스냅샷] DB 갱신 실패({type(e).__name__}): {event_id}")
            return
        await asyncio.sleep(delay)

    print(f"⚠️ [스냅샷] 짝이 되는 위험 이벤트를 못 찾음: {event_id} "
          f"(사진은 저장됨 · 실시간 표시 정상 · 재접속 복원에서만 빠짐)")


async def handle_event_snapshot(data: dict) -> Optional[dict]:
    """스냅샷을 파일로 저장하고, 프론트에 보낼 가벼운 메시지를 돌려준다.

    돌려주는 메시지에는 base64 가 없고 image_url 만 있다.
    저장하지 못하면 None — 그때는 아무것도 브로드캐스트하지 않는다.
    """
    event_id = data.get("event_id")
    image_b64 = data.get("image_b64")
    if not event_id or not image_b64:
        print(f"⚠️ [스냅샷] event_id 나 image_b64 가 없어 무시: {list(data)}")
        return None

    try:
        blob = base64.b64decode(image_b64)
    except Exception as e:
        print(f"⚠️ [스냅샷] base64 해독 실패({type(e).__name__}): {event_id}")
        return None

    name = _snapshot_name(event_id)
    path = os.path.join(SNAPSHOT_DIR, name)
    try:
        # 수십 KB 라 금방 끝나지만 디스크 쓰기는 이벤트 루프를 멈춘다.
        # 별도 스레드로 넘겨 그동안 젯슨의 다음 메시지 수신이 밀리지 않게 한다.
        await asyncio.to_thread(_write_snapshot_file, path, blob)
    except Exception as e:
        print(f"⚠️ [스냅샷] 파일 저장 실패({type(e).__name__}): {path}")
        return None

    url = f"/snapshots/{name}"
    print(f"📸 [스냅샷] {data.get('cls') or '?'} @ {data.get('block_id') or '?'} "
          f"→ {url} ({len(blob) / 1024:.0f}KB)")

    # DB 갱신은 기다리지 않는다 — 사진은 이미 디스크에 있고, 실시간 표시는
    # 호출한 쪽이 곧바로 브로드캐스트한다. (schedule_save 와 같은 이유)
    task = asyncio.create_task(_attach_snapshot_url(event_id, url))
    _save_tasks.add(task)
    task.add_done_callback(_save_tasks.discard)

    return {
        "event_type": EVENT_SNAPSHOT,
        "block_id": data.get("block_id"),
        "event_id": event_id,
        "cls": data.get("cls"),
        "image_url": url,
    }


# =========================================================
# [웹소켓 연결 관리자 구역]
# 프론트엔드의 접속 상태를 기억하고 관리한다.
# =========================================================
class ConnectionManager:
    """현재 연결된 프론트엔드 대시보드들의 웹소켓 목록을 관리하는 클래스."""

    def __init__(self):
        # 현재 대시보드를 켜놓고 있는 모든 프론트엔드의 웹소켓 객체를 담는 리스트.
        # 타입 힌트 List[WebSocket]: "WebSocket 객체들의 리스트"라는 뜻.
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        """새로운 프론트엔드 접속을 수락하고 관리 목록에 추가한다."""
        # 웹소켓 핸드셰이크를 수락 (이걸 안 하면 연결이 성립되지 않음).
        await websocket.accept()
        # 수락된 연결을 리스트에 등록 → 이후 broadcast() 대상이 됨.
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        """프론트엔드가 창을 끄면 관리 목록에서 제거한다."""
        # 이미 제거된 연결을 또 지우려다 에러 나는 것을 방지하기 위해 존재 여부 확인.
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)

    def status_line(self) -> str:
        """현재 접속 상태를 한 줄로. 로그 끝에 항상 붙인다.

        왜 필요한가 — 연결/해제 로그만 찍으면 마지막 줄이 "끊어짐"일 때
        정말 아무도 없는 건지, 다른 창이 아직 붙어 있는 건지 알 수 없다.
        브라우저를 새로고침하면 새 연결이 먼저 붙고 옛 연결이 나중에 끊기는
        순서라(겹침), "연결됨 → 연결됨 → 끊어짐" 으로 보여 접속이 끊긴 것처럼
        읽힌다. 실제로는 1개가 살아 있다. 그래서 대수를 함께 찍는다.
        """
        n = len(self.active_connections)
        return f"현재 접속 {n}대" if n else "현재 접속 없음"

    async def broadcast(self, message: dict):
        """
        젯슨이 보낸 데이터를 현재 접속 중인 '모든' 프론트엔드로 전송한다.

        NOTE: 전송 중 연결 하나가 끊겨 있어도 나머지 연결에는 계속
              전송되도록 개별 예외 처리를 한다 (끊긴 연결 하나 때문에
              전체 브로드캐스트가 멈추는 것을 방지).
        """
        # 전송 도중 끊긴 것으로 판명된 연결을 모아뒀다가 나중에 한꺼번에 정리.
        dead_connections = []

        # 현재 등록된 모든 프론트엔드 연결에 순서대로 같은 메시지를 전송.
        for connection in self.active_connections:
            try:
                # message(dict)를 JSON으로 직렬화하여 해당 연결로 전송.
                await connection.send_json(message)
            except Exception:
                # 전송 실패(연결 끊김 등) 시, 리스트에서 즉시 지우지 않고
                # 순회 중 리스트를 변경하면 버그가 생기므로 별도 목록에 모아둠.
                dead_connections.append(connection)

        # 순회가 끝난 뒤, 끊긴 연결들을 관리 목록에서 안전하게 제거.
        for dead in dead_connections:
            self.disconnect(dead)

manager = ConnectionManager()   # /ws/frontend (이벤트 JSON)

# 현재 접속 중인 젯슨의 JSON 채널 웹소켓 (서버→젯슨 쪽지 전달용).
# 젯슨은 1대뿐이므로 목록이 아닌 단일 참조로 관리. 미접속 시 None.
jetson_connection: Optional[WebSocket] = None


# =========================================================
# 1. REST API 구역 (프론트엔드 ↔ 서버 ↔ MongoDB)
# 웹 브라우저 주소창이나 HTTP GET 요청으로 접근하는 단발성 창구들.
# =========================================================

@app.get("/api/init-data")
async def get_init_data():
    """프론트엔드 대시보드가 처음 켜질 때 필요한 3D 맵 기본 정보를 준다.

    각 블록에는 현재 조립 단계(level)를 함께 담아준다.
    block_level 이벤트는 단계가 '바뀔 때만' 오기 때문에, 변화 이후에
    새로 열린 대시보드는 그 메시지를 놓친다 → 최신 상태는 이 REST로 복원.
    """
    # 하드코딩된 블록 목록. 추후 DB나 설정 파일에서 읽도록 확장 예정.
    #
    # x, y 는 ship_pose 기록이 아직 없을 때만 쓰이는 자리표시값이다. 아래에서
    # MongoDB 의 최신 ship_pose 로 덮어쓴다(젯슨이 실측해서 보낸 map 좌표, 미터).
    #
    # ※ B2 는 2026-08-20 에 주석 처리했다.
    #   지금 현장에는 선박 블록이 **한 대(B1)뿐**이고, 프론트엔드도 B2 를
    #   그리지 않는다(프론트의 S1~S5 는 배 두 척이 아니라 B1 한 척의 다섯
    #   구획이다 — 선수/전방/중앙/후방/선미). 그래서 B2 는 측량값이 영영
    #   안 들어오고, 자리표시값 (50, 80) 이 그대로 응답에 실려 나갔다.
    #   실제 좌표가 미터 단위인데 저 값만 단위가 달라 혼동을 준다.
    #   ▶ 블록이 늘어나면(공정이 여러 대로 확장되면) 아래 줄의 주석을 풀고
    #     id 를 추가하면 된다. 젯슨이 그 id 로 ship_pose / block_level 을
    #     보내기 시작하면 좌표와 단계가 자동으로 채워진다.
    blocks = [
        {"id": "B1", "x": 10, "y": 20},
        # {"id": "B2", "x": 50, "y": 80},
    ]

    # 블록마다 DB에 저장된 '가장 최근' block_level 이벤트를 찾아 현재 단계를 채움.
    # 아직 기록이 없는 블록(한 번도 감지 안 됨)은 초기 단계인 1로 간주.
    for block in blocks:
        latest = await event_collection.find_one(
            {"event_type": BLOCK_LEVEL, "block_id": block["id"]},
            sort=[("_id", -1)],  # _id 내림차순 정렬 = 가장 최근 문서 1개
        )
        # .get() 사용: level 필드가 빠진 비정상 문서가 섞여 있어도
        # KeyError로 init-data 전체가 500 나지 않도록 기본값 1로 방어.
        block["level"] = latest.get("level", 1) if latest else 1

        # 가장 최근 ship_pose(배 위치 측량) 결과로 블록 좌표·방향을 덮어씀.
        # 측량 기록이 없으면 위의 하드코딩 좌표 + yaw 0.0을 그대로 사용.
        latest_pose = await event_collection.find_one(
            {"event_type": SHIP_POSE, "block_id": block["id"]},
            sort=[("_id", -1)],
        )
        # map_xy가 [x, y] 형태로 온전할 때만 덮어씀 (불완전한 문서 방어).
        map_xy = latest_pose.get("map_xy") if latest_pose else None
        if isinstance(map_xy, (list, tuple)) and len(map_xy) == 2:
            block["x"], block["y"] = map_xy
            block["yaw"] = latest_pose.get("yaw", 0.0)
        else:
            block["yaw"] = 0.0

    return {
        "shipyard_map": "basic_3d_map_v1",
        "blocks": blocks,
        "cctv_count": 5,
    }


@app.get("/api/history")
async def get_history():
    """프론트엔드 통계 페이지가 켜질 때, DB에서 과거 이벤트 기록을 꺼내서 준다."""
    # event_collection에서 _id 기준 내림차순(최신순)으로 정렬 후 상위 50개만 조회.
    # to_list(length=50): 비동기 커서 결과를 리스트로 변환 (최대 50개 제한).
    logs = await event_collection.find().sort("_id", -1).to_list(length=50)

    # MongoDB가 자동 부여하는 _id는 ObjectId라는 특수 타입이라
    # JSON으로 그대로 보내면 직렬화 에러가 남 → 문자열로 변환.
    for log in logs:
        log["_id"] = str(log["_id"])

    return {
        "total_events": len(logs),  # 조회된 이벤트 총 개수
        "logs": logs,                # 실제 이벤트 데이터 리스트
    }


# =========================================================
# 2. WebSocket API 구역 (RC카 ↔ 서버 ↔ 프론트엔드)
# 실시간 양방향 통신을 위한 전용 채널들.
# =========================================================

@app.websocket("/ws/frontend")
async def websocket_frontend(websocket: WebSocket):
    """프론트엔드가 실시간 알림을 받기 위해 연결하는 웹소켓 채널."""
    global jetson_connection

    # ConnectionManager에 등록 (accept + 목록 추가가 여기서 함께 처리됨).
    await manager.connect(websocket)
    print(f"🖥️ [프론트엔드] 대시보드 연결됨 — {manager.status_line()}")

    # 지금 로봇이 붙어 있는지 먼저 알려준다.
    # 로봇이 이미 꺼진 상태에서 대시보드를 여는 경우, 이게 없으면 관제사는
    # 화면이 멀쩡해 보여서 실시간 정보가 멈춘 줄 모른다.
    try:
        await websocket.send_json({
            "event_type": JETSON_STATUS,
            "connected": jetson_connection is not None,
        })
    except Exception as e:
        print(f"⚠️ [프론트엔드] 로봇 상태 전송 실패({type(e).__name__})")

    # ★ 재접속 복원 (2026-08-27).
    #   프론트는 WebSocket 으로 받은 것만 그리므로, 새로고침하면 화면의 위험
    #   핑이 전부 사라진다. 불이 그 자리에 그대로 있어도 안 뜬다.
    #
    #   젯슨이 다시 보내게 만들면 안 된다. change_point_detector 가 이미 보고한
    #   이벤트를 재발행하지 않는 것은 같은 자리 불로 로봇이 반복 정지하지 않게
    #   하는 핵심 로직이다. 복원은 DB 를 들고 있는 서버가 해야 한다.
    #
    #   broadcast 가 아니라 **방금 접속한 소켓에만** 보낸다. broadcast 하면
    #   이미 보고 있던 다른 대시보드에 같은 핑이 하나 더 생긴다.
    try:
        # 배 위치를 **먼저** 보낸다. 프론트가 map_xy -> 구획 좌표로 바꿀 때
        # ship_pose 가 필요한데, 없으면 전부 첫 구획으로 떨어져 핑이 엉뚱한
        # 자리에 찍힌다 (mapXYToPingWorld).
        pose = await event_collection.find_one(
            {"event_type": SHIP_POSE}, sort=[("_id", -1)])
        if pose:
            pose.pop("_id", None)       # ObjectId 는 send_json 에서 직렬화 안 된다
            await websocket.send_json(pose)

        restored = await _active_danger_events()
        for doc in restored:
            doc.pop("_id", None)
            doc["replay"] = True        # 프론트가 팝업을 안 띄우게 하는 표시
            await websocket.send_json(doc)
        if restored:
            print(f"🖥️ [프론트엔드] 살아있는 위험 이벤트 {len(restored)}건 복원 전송")
    except Exception as e:
        # 복원이 실패해도 연결은 살린다. 핑이 안 뜨는 것은 불편이지만,
        # 여기서 예외가 새어나가면 대시보드가 아예 붙지 못한다.
        print(f"⚠️ [프론트엔드] 재접속 복원 실패(무시하고 계속): {e}")

    try:
        # 연결이 살아있는 동안 무한 대기하며 메시지를 수신.
        # 프론트→서버 방향 메시지: webrtc_signal(시그널링), event_ack(이벤트 확인).
        while True:
            raw = await websocket.receive_text()

            # JSON이 아니면 연결을 끊지 않고 해당 메시지만 무시 (개발 중 실수 대비).
            try:
                data = json.loads(raw)
            except json.JSONDecodeError:
                print(f"⚠️ [프론트엔드] JSON이 아닌 메시지 무시: {raw[:100]}")
                continue

            event_type = data.get("event_type")

            # 프론트→젯슨 전달 대상: webrtc_signal(시그널링), event_ack(이벤트 확인),
            # reset_events(기억 초기화).
            # 서버는 내용을 판단하지 않고 젯슨에게 그대로 배달만 한다 —
            # webrtc_signal 의 payload 는 WebRTC 라이브러리가 만든 것이고
            # event_ack 는 젯슨의 Nav2 가 해석할 것이라, 서버가 검사할 게 없다.
            # 🧹 이벤트 기억 초기화 — 젯슨으로 보내기 전에 **서버도 같이** 비운다.
            #   젯슨만 비우면 서버는 계속 옛 이벤트를 복원해서, 새로고침하면
            #   방금 지운 핑이 되살아난다.
            if event_type == RESET_EVENTS:
                global RESTORE_FLOOR_AT
                RESTORE_FLOOR_AT = datetime.now(KST)
                cleared = len(jetson_live_event_ids)
                jetson_live_event_ids.clear()
                print(f"🧹 [초기화] 복원 기준선을 지금으로 밀었다 "
                      f"({RESTORE_FLOOR_AT.isoformat()[11:19]}) · 생존 목록 {cleared}건 비움. "
                      f"DB 기록은 그대로 둔다(증빙).")

            if event_type in JETSON_BOUND_TYPES:
                if jetson_connection is None:
                    print(f"⚠️ [중계] {event_type} 전달 실패: 젯슨 미접속 상태")
                    continue
                try:
                    await jetson_connection.send_json(data)
                    print(f"📮 [중계] {event_type} → 젯슨 전달 완료")
                except Exception:
                    # 전달 도중 젯슨 연결이 끊긴 경우: 끊긴 연결 참조를 계속
                    # 들고 있으면 이후 요청도 계속 실패하므로 즉시 비워서
                    # 다음 요청부터 '미접속'으로 정확히 처리되게 한다.
                    jetson_connection = None
                    print(f"⚠️ [중계] {event_type} 전달 중 젯슨 연결 끊김 → 참조 해제")
            else:
                print(f"프론트엔드에서 온 메시지: {data}")

    except WebSocketDisconnect:
        # 브라우저 창을 닫는 등으로 연결이 끊기면 이 예외가 발생.
        manager.disconnect(websocket)
        # 새로고침이면 여기서 0 이 아니다 (새 연결이 먼저 붙어 있으므로).
        # 대수를 함께 찍어야 "끊겼는데 왜 아직 되지?" 로 헷갈리지 않는다.
        if manager.active_connections:
            print(f"🖥️ [프론트엔드] 창 하나 닫힘 (새로고침일 수 있음) — "
                  f"{manager.status_line()}")
        else:
            print(f"🖥️ [프론트엔드] 연결 끊어짐 — {manager.status_line()}")


@app.websocket("/ws/jetson")
async def websocket_jetson(websocket: WebSocket):
    """RC카(젯슨)가 실시간 센서 및 AI 감지 데이터를 보내기 위해 연결하는 채널."""
    global jetson_connection

    # 젯슨 쪽은 ConnectionManager에 등록하지 않고 단순 accept만 함
    # (젯슨은 1대뿐이라 브로드캐스트 대상 목록에 넣을 필요가 없음).
    await websocket.accept()

    # 서버→젯슨 방향(webrtc_signal·event_ack 전달)에 쓸 수 있도록 연결을 기억해 둠.
    # 재접속 등으로 새 연결이 오면 마지막 연결이 이전 것을 덮어씀.
    jetson_connection = websocket
    # 새 연결은 곧바로 "살아있는 위험" 묶음을 재통보한다. 옛 목록을 비워두고
    # 그것으로 다시 채워야 젯슨이 이미 잊은 이벤트가 남지 않는다.
    jetson_live_event_ids.clear()
    print("🚗 [젯슨 RC카] 연결됨 — 현재 접속 중")
    await broadcast_jetson_status()

    try:
        while True:
            # 젯슨이 보내는 JSON 메시지를 dict 형태로 수신.
            # 평상시 위치 핑(ping)일 수도 있고, 4종 이벤트 중 하나일 수도 있음.
            data = await websocket.receive_json()

            # 수신한 메시지의 event_type 값을 확인.
            event_type = data.get("event_type")

            # 📸 [스냅샷] 감지 순간의 사진. 다른 메시지와 처리 방식이 아예 다르다.
            #   DB 에 통째로 저장하지 않고 파일로 떨군 뒤, base64 를 뺀 가벼운
            #   메시지만 프론트로 보낸다. 자세한 이유는 [이벤트 스냅샷 구역] 참조.
            #   continue 로 아래 저장/브로드캐스트 경로를 타지 않게 한다 —
            #   그냥 흘려보내면 base64 가 프론트까지 그대로 간다.
            if event_type == EVENT_SNAPSHOT:
                light = await handle_event_snapshot(data)
                if light is not None:
                    await manager.broadcast(light)
                continue

            # 🚨 [DB 저장 로직] 팀이 합의한 저장 대상(위험 이벤트 4종 +
            # BLOCK_LEVEL 단계 변화 + SHIP_POSE 배 위치)만 DB에 영구 저장한다.
            # (평상시 위치 핑 등 그 외 메시지는 저장하지 않고 브로드캐스트만 함)
            # 📣 젯슨 재연결 시 "아직 살아있다" 재통보 (replay:true).
            #   이미 아는 이벤트면 저장하지 않는다 — 중복 기록이 되고, 실제로
            #   그 시각에 감지된 것도 아니라 감사 기록을 흐린다. 대신 생존
            #   목록에 넣어 복원 판정이 시각 바닥을 넘어 꺼내오게 한다.
            if data.get("replay") and event_type in DANGER_TYPES:
                eid = data.get("event_id")
                if eid:
                    jetson_live_event_ids.add(eid)
                    # ★ 단, 처음 보는 이벤트면 저장해야 한다 (2026-08-29).
                    #   서버가 꺼져 있는 동안 처음 감지된 불은 이 재통보가
                    #   **유일한 전달 경로**다. 안 남기면 DB 에 기록이 아예 없어,
                    #   생존 목록에 id 는 있는데 꺼내올 문서가 없다 —— 핑은 떴다가
                    #   새로고침하면 사라진다.
                    schedule_replay_backfill(data.copy())
                await manager.broadcast(data)
                continue

            if event_type in LOGGED_EVENT_TYPES:
                # 서버 수신 시각을 timestamp 필드로 추가 (감사/증빙 자료 용도).
                # 한국 표준시 + 오프셋 명시(+09:00 포함 ISO 8601)로 기록 —
                # DB를 눈으로 볼 때 한국 시간 그대로 읽히고, 오프셋이 있어
                # 프론트 JS의 new Date()도 정확히 해석함.
                data["timestamp"] = datetime.now(KST).isoformat()

                # ★ 저장을 기다리지 않는다 (2026-08-12).
                #   schedule_save 는 백그라운드 작업만 만들고 즉시 돌아온다.
                #   인터넷이 끊겨 저장이 오래 걸리거나 실패해도 이 루프는
                #   멈추지 않고, 아래 broadcast 가 지연 없이 실행된다.
                #   실패분은 재시도 대기줄에 들어가 인터넷 복구 시 자동 저장된다.
                #
                #   data.copy() 로 복사본을 넘기는 이유: 원본 dict 는 곧이어
                #   broadcast() 에도 쓰이는데, 저장이 백그라운드로 도는 동안
                #   원본이 바뀌면 엉뚱한 값이 저장될 수 있다.
                schedule_save(data.copy())

            # 치워졌다고 하면 생존 목록에서도 뺀다 — 안 그러면 시각 바닥을
            # 넘어 계속 꺼내오는 유령이 된다.
            if event_type == EVENT_CLEARED and data.get("event_id"):
                jetson_live_event_ids.discard(data["event_id"])

            # DB 저장 여부와 관계없이, 프론트엔드에는 지연 없이 즉시 브로드캐스트.
            await manager.broadcast(data)

    except WebSocketDisconnect:
        # 젯슨 전원이 꺼지거나 통신이 끊기면 발생.
        # 이 연결이 현재 기억된 연결일 때만 해제 (재접속 직후 옛 연결이
        # 끊기면서 새 연결 참조를 지워버리는 것을 방지).
        # 프론트와 같은 이유로 대수 대신 "지금 붙어 있나"를 함께 찍는다.
        # 재접속이면 새 연결이 이미 jetson_connection 을 차지한 뒤라
        # 옛 연결이 끊기는 이 시점에도 접속은 살아 있다.
        if jetson_connection is websocket:
            jetson_connection = None
            print("🚗 [젯슨 RC카] 연결 끊어짐 — 현재 접속 없음")
            # 재접속이면 새 연결이 이미 알렸으므로 여기서는 보내지 않는다.
            # (그래야 "끊김 → 연결됨" 이 순서가 뒤집혀 도착하지 않는다)
            await broadcast_jetson_status()
        else:
            print("🚗 [젯슨 RC카] 옛 연결 정리됨 (재접속) — 현재 접속 중")

