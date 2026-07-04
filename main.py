import os
from dotenv import load_dotenv # 이 줄 추가
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from motor.motor_asyncio import AsyncIOMotorClient
from typing import List
from datetime import datetime 

# .env 파일(금고)을 불러옵니다.
load_dotenv()

# FastAPI 서버 객체 생성 (우리의 백엔드 서버 본체입니다)
app = FastAPI()

# =========================================================
# [데이터베이스 셋업 구역]
# MongoDB Atlas와 통신하는 선을 연결하는 곳입니다.
# =========================================================

# 1. MongoDB 연결 주소 가져오기
# os.getenv("키값")을 사용하면 .env 파일에 적어둔 주소를 가져옵니다.
MONGO_URL = os.getenv("MONGO_URL")

# 2. 비동기(Async) 방식으로 DB에 접속하는 클라이언트 객체 생성
# (비동기 방식이라 DB 저장 중에도 젯슨의 실시간 통신이 끊기지 않습니다)
client = AsyncIOMotorClient(MONGO_URL)

# 3. 데이터베이스와 컬렉션(서류함) 지정
db = client.shipyard_db         # 'shipyard_db'라는 이름의 데이터베이스 사용
event_collection = db.events    # 그 안에 'events'라는 사고 기록 전용 서류함 사용


# =========================================================
# [웹소켓 연결 관리자 구역] 
# 프론트엔드의 접속 상태를 기억하고 관리합니다.
# =========================================================
class ConnectionManager:
    def __init__(self):
        # 현재 대시보드를 켜놓고 있는 프론트엔드의 웹소켓(무전기)들을 담아두는 리스트
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        # 새로운 프론트엔드가 접속하면 무전기 연결을 수락하고 리스트에 추가합니다.
        await websocket.accept() 
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        # 프론트엔드가 창을 끄면 리스트에서 제거합니다.
        self.active_connections.remove(websocket)

    async def broadcast(self, message: dict):
        # RC카가 젯슨에서 보낸 데이터를, 현재 접속 중인 '모든' 프론트엔드 화면에 동시에 쏴줍니다.
        for connection in self.active_connections:
            await connection.send_json(message)

manager = ConnectionManager()


# =========================================================
# 1. REST API 구역 (프론트엔드 ↔ 서버 ↔ MongoDB)
# 웹 브라우저 주소창이나 HTTP GET 요청으로 접근하는 단발성 창구들입니다.
# =========================================================

@app.get("/api/init-data")
async def get_init_data():
    """프론트엔드 대시보드가 처음 켜질 때 필요한 3D 맵 기본 정보를 줍니다."""
    return {
        "shipyard_map": "basic_3d_map_v1",
        "blocks": [{"id": "B1", "x": 10, "y": 20}, {"id": "B2", "x": 50, "y": 80}],
        "cctv_count": 5
    }

@app.get("/api/history")
async def get_history():
    """프론트엔드가 대시보드 통계 페이지를 켤 때, DB에서 과거 사고 기록을 꺼내서 줍니다."""
    # DB 서류함(event_collection)에서 최근 데이터 50개를 찾아 리스트 형태로 가져옵니다.
    logs = await event_collection.find().sort("_id", -1).to_list(length=50)
    
    # MongoDB가 자동으로 부여하는 고유 ID(_id)는 특수 객체 형태라서, 
    # 프론트엔드로 보낼 때 에러가 나지 않도록 문자열(string)로 변환해 줍니다.
    for log in logs:
        log["_id"] = str(log["_id"])
        
    return {
        "total_events": len(logs), # 가져온 사고 기록의 총 개수
        "logs": logs               # 실제 사고 기록 데이터 리스트
    }


# =========================================================
# 2. WebSocket API 구역 (RC카 ↔ 서버 ↔ 프론트엔드)
# 실시간 0.1초 단위 양방향 통신을 위한 전용 무전기 채널들입니다.
# =========================================================

@app.websocket("/ws/frontend")
async def websocket_frontend(websocket: WebSocket):
    """프론트엔드가 실시간 알림을 받기 위해 연결하는 웹소켓 채널"""
    await manager.connect(websocket)
    print("🖥️ [프론트엔드] 대시보드 웹소켓 연결됨!")
    
    try:
        # 연결이 유지되는 동안 무한 루프를 돌며 대기합니다.
        while True:
            # 프론트엔드에서 버튼 클릭 등 서버로 보낼 메시지가 있다면 여기서 받습니다.
            data = await websocket.receive_text()
            print(f"프론트엔드에서 온 메시지: {data}")
            
    except WebSocketDisconnect:
        # 프론트엔드가 브라우저 창을 닫으면 연결 해제 처리를 합니다.
        manager.disconnect(websocket)
        print("🖥️ [프론트엔드] 연결 끊어짐.")


@app.websocket("/ws/jetson")
async def websocket_jetson(websocket: WebSocket):
    """RC카가 실시간 센서 및 비전 데이터를 보내기 위해 연결하는 채널"""
    await websocket.accept()
    print("🚗 [젯슨 RC카] 웹소켓 연결됨!")
    
    try:
        while True:
            # 1. RC카가 보내는 JSON 데이터를 받습니다.
            data = await websocket.receive_json()
            
            # 2. 🚨 [DB 저장 로직] 받은 데이터가 긴급 사고(낙상, 화재)라면?
            if data.get("event_type") in ["fallen_person", "fire"]:
                # 서버의 현재 시간을 'timestamp'라는 이름으로 데이터에 추가합니다.
                data["timestamp"] = datetime.now().isoformat()
                
                # DB 서류함에 해당 데이터를 복사해서 영구 저장합니다.
                await event_collection.insert_one(data.copy())
                print(f"💾 몽고DB에 사고 기록 저장 완료: {data['event_type']}")

            # 3. DB 저장과 별개로, 프론트엔드에게 지연 없이 데이터를 즉시 브로드캐스트합니다.
            await manager.broadcast(data)
            
    except WebSocketDisconnect:
        # RC카 전원이 꺼지거나 통신이 끊기면 발생합니다.
        print("🚗 [젯슨 RC카] 연결 끊어짐.")