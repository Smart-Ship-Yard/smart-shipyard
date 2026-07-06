"""
main.py — 스마트 조선소 FastAPI 백엔드 서버

프로젝트   : 스마트 조선소 선박 건조 공정 트래킹 및 디지털 트윈 관제 시스템
모듈 설명  : 젯슨(UGV) ↔ 서버 ↔ 프론트엔드 간 실시간 이벤트 중계 서버.
            - WebSocket으로 젯슨의 실시간 센서/AI 감지 이벤트를 수신하여
              프론트엔드(React+Three.js 대시보드)로 즉시 브로드캐스트
            - 아래 4종 위험 이벤트는 MongoDB에 영구 로그 저장
            - REST API로 대시보드 초기 로딩 데이터 및 과거 이벤트 이력 제공

작성자     : 이정기 (Backend & Streaming Engineer)
작성일     : 2026-07-06
최근 수정일 : 2026-07-06

의존성     : Python 3.10+, FastAPI 0.138.2, motor 3.7.1 (requirements.txt 참조)
실행 방법  : uvicorn main:app --reload
환경 변수  : .env 파일에 MONGO_URL 필요 (.env.example 참조, CONTRIBUTING.md 참고)

탐지 이벤트 4종 (팀 합의):
    - ship_defect       : 선박(블록) 결함 — 세부 판정 기준 추후 협의 예정
    - helmet_off        : 안전모 미착용
    - worker_collapsed  : 작업자 쓰러짐
    - fire              : 화재

TODO: ship_defect의 구체적 판정 클래스/기준은 AI팀과 추후 확정 필요.
      확정 전까지는 문자열 "ship_defect"를 그대로 event_type 값으로 사용.
"""

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

# 타입 힌트용 List (ConnectionManager의 연결 목록 타입 표기에 사용)
from typing import List

# 이벤트 저장 시각을 기록하기 위한 datetime
from datetime import datetime

# .env 파일(금고)에 적힌 값들을 실제로 읽어들여 환경변수로 등록.
# 이 줄이 없으면 아래 os.getenv("MONGO_URL")이 None을 반환함.
load_dotenv()

# =========================================================
# [이벤트 타입 상수 정의 구역]
# 팀이 합의한 4종 이벤트만 DB에 영구 저장 대상으로 취급한다.
# 여기 한 곳만 고치면 아래 로직 전체에 반영되도록 상수로 분리함.
# =========================================================
SHIP_DEFECT = "ship_defect"            # 선박(블록) 결함 — 판정 기준 추후 협의
HELMET_OFF = "helmet_off"              # 안전모 미착용
WORKER_COLLAPSED = "worker_collapsed"  # 작업자 쓰러짐
FIRE = "fire"                          # 화재

# DB 저장 대상 이벤트 목록 (이 4개 중 하나에 해당하는 event_type만 로그 저장)
LOGGED_EVENT_TYPES = {SHIP_DEFECT, HELMET_OFF, WORKER_COLLAPSED, FIRE}

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


# 앱 전체에서 공유해서 쓸 ConnectionManager 인스턴스를 하나만 생성 (싱글턴처럼 사용).
manager = ConnectionManager()


# =========================================================
# 1. REST API 구역 (프론트엔드 ↔ 서버 ↔ MongoDB)
# 웹 브라우저 주소창이나 HTTP GET 요청으로 접근하는 단발성 창구들.
# =========================================================

@app.get("/api/init-data")
async def get_init_data():
    """프론트엔드 대시보드가 처음 켜질 때 필요한 3D 맵 기본 정보를 준다."""
    # 지금은 하드코딩된 예시 데이터. 추후 선박 블록 좌표는 DB나 설정 파일에서
    # 읽어오도록 확장 예정 (블록 개수/좌표가 늘어날 것이므로).
    return {
        "shipyard_map": "basic_3d_map_v1",
        "blocks": [{"id": "B1", "x": 10, "y": 20}, {"id": "B2", "x": 50, "y": 80}],
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
    # ConnectionManager에 등록 (accept + 목록 추가가 여기서 함께 처리됨).
    await manager.connect(websocket)
    print("🖥️ [프론트엔드] 대시보드 웹소켓 연결됨!")

    try:
        # 연결이 살아있는 동안 무한 대기하며 메시지를 수신.
        # 프론트→서버 방향 메시지는 지금은 로그만 출력 (추후 필요 시 처리 로직 추가).
        while True:
            data = await websocket.receive_text()
            print(f"프론트엔드에서 온 메시지: {data}")

    except WebSocketDisconnect:
        # 브라우저 창을 닫는 등으로 연결이 끊기면 이 예외가 발생.
        manager.disconnect(websocket)
        print("🖥️ [프론트엔드] 연결 끊어짐.")


@app.websocket("/ws/jetson")
async def websocket_jetson(websocket: WebSocket):
    """RC카(젯슨)가 실시간 센서 및 AI 감지 데이터를 보내기 위해 연결하는 채널."""
    # 젯슨 쪽은 ConnectionManager에 등록하지 않고 단순 accept만 함
    # (젯슨은 1대뿐이라 브로드캐스트 대상 목록에 넣을 필요가 없음).
    await websocket.accept()
    print("🚗 [젯슨 RC카] 웹소켓 연결됨!")

    try:
        while True:
            # 젯슨이 보내는 JSON 메시지를 dict 형태로 수신.
            # 평상시 위치 핑(ping)일 수도 있고, 4종 이벤트 중 하나일 수도 있음.
            data = await websocket.receive_json()

            # 수신한 메시지의 event_type 값을 확인.
            event_type = data.get("event_type")

            # 🚨 [DB 저장 로직] 팀이 합의한 4종 이벤트(SHIP_DEFECT, HELMET_OFF,
            # WORKER_COLLAPSED, FIRE)에 해당할 때만 DB에 영구 저장한다.
            # (평상시 위치 핑 등 그 외 메시지는 저장하지 않고 브로드캐스트만 함)
            if event_type in LOGGED_EVENT_TYPES:
                # 서버 수신 시각을 timestamp 필드로 추가 (감사/증빙 자료 용도).
                data["timestamp"] = datetime.now().isoformat()

                # data.copy()로 복사본을 저장 — 원본 dict는 곧이어 그대로
                # broadcast()에도 쓰이므로, insert_one이 원본을 변형하지
                # 않도록 방어적으로 복사해서 넘김.
                await event_collection.insert_one(data.copy())
                print(f"💾 몽고DB에 이벤트 저장 완료: {event_type}")

            # DB 저장 여부와 관계없이, 프론트엔드에는 지연 없이 즉시 브로드캐스트.
            await manager.broadcast(data)

    except WebSocketDisconnect:
        # 젯슨 전원이 꺼지거나 통신이 끊기면 발생.
        print("🚗 [젯슨 RC카] 연결 끊어짐.")