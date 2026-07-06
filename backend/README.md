# Backend — 실시간 이벤트 중계 서버

스마트 조선소 디지털 트윈 시스템의 백엔드 서버입니다.
젯슨(UGV)이 보내는 실시간 위치/이벤트 데이터를 받아 프론트엔드 대시보드에 중계하고,
위험 이벤트를 MongoDB에 기록합니다.

담당: 이정기 (Backend & Streaming Engineer)

## 기술 스택

| 구분 | 기술 |
|---|---|
| 프레임워크 | FastAPI (Python 3.10+) |
| 실시간 통신 | WebSocket |
| DB | MongoDB Atlas (motor 비동기 드라이버) |
| 실행 서버 | Uvicorn |

## 처리하는 이벤트 4종

| event_type | 의미 |
|---|---|
| `ship_defect` | 선박(블록) 결함 — 세부 기준 협의 중 |
| `helmet_off` | 안전모 미착용 |
| `worker_collapsed` | 작업자 쓰러짐 |
| `fire` | 화재 |

위 4종만 MongoDB에 영구 저장되며, 그 외 메시지(위치 핑 등)는 저장 없이 프론트엔드로 중계만 됩니다.

## 실행 방법

```bash
cd backend

# 1. 가상환경 생성 및 활성화 (최초 1회)
python -m venv venv
source venv/bin/activate      

# 2. 패키지 설치
pip install -r requirements.txt

# 3. 환경 변수 설정 (최초 1회)
cp .env.example .env
# .env 파일을 열어 MONGO_URL에 실제 접속 주소 입력

# 4. 서버 실행
uvicorn main:app --reload
```

서버가 뜨면 http://127.0.0.1:8000/docs 에서 API 문서를 확인할 수 있습니다.

## API 요약

| 종류 | 경로 | 설명 |
|---|---|---|
| WebSocket | `/ws/jetson` | 젯슨(UGV)이 데이터를 보내는 채널 |
| WebSocket | `/ws/frontend` | 대시보드가 실시간 알림을 받는 채널 |
| GET | `/api/init-data` | 대시보드 초기 3D 맵 정보 |
| GET | `/api/history` | 과거 이벤트 로그 조회 (최근 50건) |

## 젯슨 → 서버 메시지 형식 (협의 중)

```json
{
  "event_type": "fire",
  "confidence": 0.92,
  "uwb_xy": [1.2, 3.4],
  "depth_xyz": [1.1, 2.2, 0.8]
}
```

> 정식 스키마는 AI팀·프론트팀과 합의 후 이 문서에 확정본을 반영할 예정입니다.

## 주의사항

- `.env`는 절대 커밋하지 마세요 (`.gitignore`로 차단되어 있음).
- 패키지를 새로 설치했다면 `pip freeze > requirements.txt`로 버전 목록을 갱신하고 함께 커밋하세요.
- 팀 공통 개발 규칙(브랜치 전략, 커밋 메시지)은 저장소 루트의 `CONTRIBUTING.md`를 참고하세요.
