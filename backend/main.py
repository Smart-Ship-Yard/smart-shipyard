"""
main.py — 스마트 조선소 FastAPI 백엔드 서버

프로젝트   : 스마트 조선소 선박 건조 공정 트래킹 및 디지털 트윈 관제 시스템
모듈 설명  : 젯슨(UGV) ↔ 서버 ↔ 프론트엔드 간 실시간 이벤트 중계 서버.
            - WebSocket으로 젯슨의 실시간 센서/AI 감지 이벤트를 수신하여
              프론트엔드(React+Three.js 대시보드)로 즉시 브로드캐스트
            - 위험 이벤트 4종 + block_level + ship_pose는 MongoDB에 영구 로그 저장
            - WebRTC 시그널링(webrtc_signal) 쪽지를 프론트↔젯슨 양방향 중계
              (영상은 WebRTC P2P 직결로 변경 — 2026-07-10 팀 결정)
            - 실시간 영상(JPEG 바이너리) 중계 채널은 P2P 실패 시 폴백용으로 유지
            - 프론트의 stream_boost(영상 화질 전환) 명령을 젯슨으로 전달
            - REST API로 대시보드 초기 로딩 데이터 및 과거 이벤트 이력 제공

웹소켓 채널 4개 (통신 스펙 v1.2 = docs/interface.md 참조):
    /ws/jetson           젯슨 JSON 채널 (이벤트 수신 + stream_boost 송신)
    /ws/frontend         프론트 JSON 채널 (이벤트 브로드캐스트 + stream_boost 수신)
    /ws/jetson-stream    젯슨 영상 채널 (JPEG 바이너리 수신)
    /ws/frontend-stream  프론트 영상 채널 (JPEG 바이너리 브로드캐스트)

작성자     : 이정기 (Backend & Streaming Engineer)
작성일     : 2026-07-06
최근 수정일 : 2026-07-10

의존성     : Python 3.10+, FastAPI 0.138.2, motor 3.7.1 (requirements.txt 참조)
실행 방법  : uvicorn main:app --reload
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

import json
import os
# python-dotenv 패키지: .env 파일에 적힌 키=값 쌍을 읽어서
# os.environ(환경변수)에 등록해주는 역할. 아래 load_dotenv() 호출과 짝을 이룸.
from dotenv import load_dotenv

# FastAPI 핵심 클래스와, 웹소켓 연결 객체 / 연결 끊김 예외를 가져옴
from fastapi import FastAPI, WebSocket, WebSocketDisconnect

# 다른 도메인(프론트엔드)에서의 API 요청을 허용해주는 미들웨어
from fastapi.middleware.cors import CORSMiddleware

# MongoDB를 비동기(async)로 다루기 위한 motor 라이브러리의 클라이언트
from motor.motor_asyncio import AsyncIOMotorClient

# 타입 힌트용 List/Optional (연결 목록, 젯슨 단일 연결 참조 타입 표기에 사용)
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

# 프론트→서버→젯슨 방향 명령 (DB 저장 대상 아님).
# 프론트가 영상 팝업을 열/닫을 때 젯슨의 영상 화질을 전환시키는 명령.
STREAM_BOOST = "stream_boost"    # action: "start"(부스트) / "stop"(원복)

# WebRTC 시그널링 쪽지 (영상 P2P 직결용, 양방향, DB 저장 대상 아님).
# payload 안의 내용(SDP/ICE)은 WebRTC 라이브러리가 자동 생성한 것 —
# 서버는 열어보지 않고 반대편에 그대로 배달만 한다.
# (예: {"event_type": "webrtc_signal", "payload": {...}})
WEBRTC_SIGNAL = "webrtc_signal"

# 프론트→서버→젯슨 방향으로 '그대로 전달'하는 메시지 종류 모음.
# (젯슨→프론트 방향은 기존 브로드캐스트가 모든 메시지를 전달하므로 목록 불필요)
JETSON_BOUND_TYPES = {STREAM_BOOST, WEBRTC_SIGNAL}

# 이벤트 timestamp 기록용 한국 표준시.
# 시간대 정보 없는(naive) 시각은 환경마다 해석이 달라지므로 +09:00을 명시한다.
KST = timezone(timedelta(hours=9))

# DB 저장 대상 이벤트 목록 — 위험 이벤트 4종 + 공정 단계 변화 + 배 위치 측량.
# block_level/ship_pose는 '바뀔 때만' 오는 희소 이벤트라 저장량 부담이 없고,
# 최신 값을 init-data 상태 복원에 쓰므로 저장 대상에 포함.
LOGGED_EVENT_TYPES = {SHIP_DEFECT, NO_HELMET, FALLEN_PERSON, FIRE, BLOCK_LEVEL, SHIP_POSE}

# FastAPI 서버 객체 생성 (우리의 백엔드 서버 본체).
# title/description/version은 자동 생성되는 API 문서(/docs)에 표시됨.
app = FastAPI(
    title="Smart Shipyard Digital Twin Backend",
    description="젯슨 UGV ↔ 서버 ↔ 프론트엔드 실시간 이벤트 중계 API",
    version="0.1.0",
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
# [데이터베이스 셋업 구역]
# MongoDB Atlas와 통신하는 선을 연결하는 곳.
# =========================================================

# .env 파일에 적어둔 MONGO_URL 값을 읽어옴 (예: mongodb+srv://...).
# 값이 없으면 None이 반환되며, 이 경우 아래 client 생성 시 접속 실패로 이어짐.
MONGO_URL = os.getenv("MONGO_URL")

# 비동기 방식으로 MongoDB에 접속하는 클라이언트 객체 생성.
# 비동기이기 때문에 DB에 쓰는 동안에도 웹소켓 통신이 멈추지 않음.
client = AsyncIOMotorClient(MONGO_URL)

# 'shipyard_db'라는 이름의 데이터베이스를 선택 (없으면 첫 데이터 삽입 시 자동 생성됨).
db = client.shipyard_db

# 그 안의 'events' 컬렉션(=서류함)을 선택. 여기에 4종 이벤트 로그가 쌓임.
event_collection = db.events


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

    async def broadcast_bytes(self, data: bytes):
        """
        바이너리 데이터(영상 JPEG 프레임)를 접속 중인 모든 연결로 전송한다.
        broadcast()와 동일한 패턴이며 전송 방식만 send_bytes로 다름.
        """
        dead_connections = []
        for connection in self.active_connections:
            try:
                await connection.send_bytes(data)
            except Exception:
                dead_connections.append(connection)
        for dead in dead_connections:
            self.disconnect(dead)


# 이벤트(JSON) 채널과 영상(바이너리) 채널의 프론트엔드 연결을 각각 따로 관리.
# 채널을 분리하는 이유: 영상 프레임이 몰릴 때 이벤트 JSON 전달이 밀리지 않게 하기 위함.
manager = ConnectionManager()         # /ws/frontend        (이벤트 JSON)
stream_manager = ConnectionManager()  # /ws/frontend-stream (영상 바이너리)

# 현재 접속 중인 젯슨의 JSON 채널 웹소켓 (서버→젯슨 stream_boost 전달용).
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
    # 지금은 하드코딩된 예시 데이터. 추후 선박 블록 좌표는 DB나 설정 파일에서
    # 읽어오도록 확장 예정 (블록 개수/좌표가 늘어날 것이므로).
    blocks = [{"id": "B1", "x": 10, "y": 20}, {"id": "B2", "x": 50, "y": 80}]

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
    print("🖥️ [프론트엔드] 대시보드 웹소켓 연결됨!")

    try:
        # 연결이 살아있는 동안 무한 대기하며 메시지를 수신.
        # 프론트→서버 방향 메시지: stream_boost(화질 명령), webrtc_signal(시그널링).
        while True:
            raw = await websocket.receive_text()

            # JSON이 아니면 연결을 끊지 않고 해당 메시지만 무시 (개발 중 실수 대비).
            try:
                data = json.loads(raw)
            except json.JSONDecodeError:
                print(f"⚠️ [프론트엔드] JSON이 아닌 메시지 무시: {raw[:100]}")
                continue

            event_type = data.get("event_type")

            # 프론트→젯슨 전달 대상: stream_boost(화질 명령), webrtc_signal(시그널링).
            # 서버는 내용을 판단하지 않고 젯슨에게 그대로 배달만 한다.
            if event_type in JETSON_BOUND_TYPES:
                # stream_boost만 action 값을 검증. webrtc_signal의 payload는
                # WebRTC 라이브러리가 만든 것이라 서버가 검사할 필요가 없다.
                if event_type == STREAM_BOOST and data.get("action") not in ("start", "stop"):
                    print(f"⚠️ [프론트엔드] stream_boost의 action 값이 이상함: {data}")
                    continue

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
        print("🖥️ [프론트엔드] 연결 끊어짐.")


@app.websocket("/ws/jetson")
async def websocket_jetson(websocket: WebSocket):
    """RC카(젯슨)가 실시간 센서 및 AI 감지 데이터를 보내기 위해 연결하는 채널."""
    global jetson_connection

    # 젯슨 쪽은 ConnectionManager에 등록하지 않고 단순 accept만 함
    # (젯슨은 1대뿐이라 브로드캐스트 대상 목록에 넣을 필요가 없음).
    await websocket.accept()

    # 서버→젯슨 방향(stream_boost 전달)에 쓸 수 있도록 연결을 기억해 둠.
    # 재접속 등으로 새 연결이 오면 마지막 연결이 이전 것을 덮어씀.
    jetson_connection = websocket
    print("🚗 [젯슨 RC카] 웹소켓 연결됨!")

    try:
        while True:
            # 젯슨이 보내는 JSON 메시지를 dict 형태로 수신.
            # 평상시 위치 핑(ping)일 수도 있고, 4종 이벤트 중 하나일 수도 있음.
            data = await websocket.receive_json()

            # 수신한 메시지의 event_type 값을 확인.
            event_type = data.get("event_type")

            # 🚨 [DB 저장 로직] 팀이 합의한 저장 대상(위험 이벤트 4종 +
            # BLOCK_LEVEL 단계 변화 + SHIP_POSE 배 위치)만 DB에 영구 저장한다.
            # (평상시 위치 핑 등 그 외 메시지는 저장하지 않고 브로드캐스트만 함)
            if event_type in LOGGED_EVENT_TYPES:
                # 서버 수신 시각을 timestamp 필드로 추가 (감사/증빙 자료 용도).
                # 한국 표준시 + 오프셋 명시(+09:00 포함 ISO 8601)로 기록 —
                # DB를 눈으로 볼 때 한국 시간 그대로 읽히고, 오프셋이 있어
                # 프론트 JS의 new Date()도 정확히 해석함.
                data["timestamp"] = datetime.now(KST).isoformat()

                # data.copy()로 복사본을 저장 — 원본 dict는 곧이어 그대로
                # broadcast()에도 쓰이므로, insert_one이 원본을 변형하지
                # 않도록 방어적으로 복사해서 넘김.
                await event_collection.insert_one(data.copy())
                print(f"💾 몽고DB에 이벤트 저장 완료: {event_type}")

            # DB 저장 여부와 관계없이, 프론트엔드에는 지연 없이 즉시 브로드캐스트.
            await manager.broadcast(data)

    except WebSocketDisconnect:
        # 젯슨 전원이 꺼지거나 통신이 끊기면 발생.
        # 이 연결이 현재 기억된 연결일 때만 해제 (재접속 직후 옛 연결이
        # 끊기면서 새 연결 참조를 지워버리는 것을 방지).
        if jetson_connection is websocket:
            jetson_connection = None
        print("🚗 [젯슨 RC카] 연결 끊어짐.")


# =========================================================
# 3. 영상 스트림 중계 구역 (젯슨 → 서버 → 프론트엔드, 바이너리)
# 젯슨이 보내는 JPEG 프레임을 디코딩 없이 그대로 프론트로 흘려보낸다.
# 이벤트 JSON 채널과 분리해서 영상 때문에 이벤트 전달이 밀리지 않게 함.
# =========================================================

@app.websocket("/ws/jetson-stream")
async def websocket_jetson_stream(websocket: WebSocket):
    """젯슨이 실시간 영상 JPEG 프레임(바이너리)을 보내는 전용 채널."""
    await websocket.accept()
    print("🎥 [젯슨 영상] 스트림 채널 연결됨!")

    try:
        while True:
            # JPEG 한 장 = 바이너리 메시지 한 개.
            frame = await websocket.receive_bytes()

            # 서버는 프레임을 열어보지 않고(디코딩 없음) 바이트 그대로 중계.
            await stream_manager.broadcast_bytes(frame)

    except WebSocketDisconnect:
        print("🎥 [젯슨 영상] 스트림 채널 끊어짐.")


@app.websocket("/ws/frontend-stream")
async def websocket_frontend_stream(websocket: WebSocket):
    """프론트엔드가 실시간 영상을 받기 위해 연결하는 채널 (수신 전용)."""
    await stream_manager.connect(websocket)
    print("🖥️ [프론트엔드 영상] 스트림 시청 시작!")

    try:
        # 이 채널은 서버→프론트 단방향. 아래 수신 대기는 데이터를 쓰기 위함이
        # 아니라 연결 유지와 끊김(WebSocketDisconnect) 감지를 위한 것.
        while True:
            await websocket.receive_text()

    except WebSocketDisconnect:
        stream_manager.disconnect(websocket)
        print("🖥️ [프론트엔드 영상] 스트림 시청 종료.")