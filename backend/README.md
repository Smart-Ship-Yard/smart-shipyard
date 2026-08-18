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
cd ~/smart-shipyard/backend

# 1. 가상환경 생성 및 활성화 (최초 1회)
python -m venv venv
source venv/bin/activate      

# 2. 패키지 설치
pip install -r requirements.txt

# 3. 환경 변수 설정 (최초 1회)
cp .env.example .env
# .env 파일을 열어 MONGO_URL에 실제 접속 주소 입력

# 4. 서버 실행 (팀 연동용 — 0.0.0.0 이 핵심)
venv/bin/uvicorn main:app --host 0.0.0.0 --port 8000
```

**`--host 0.0.0.0` 을 반드시 붙입니다.** 생략하면 `127.0.0.1` 에만 묶여
**이 노트북 안에서만** 접속됩니다. 젯슨·프론트엔드는 같은 공유기의 다른
기기라서 연결이 안 됩니다.

> `--reload` 는 파일을 저장할 때마다 서버를 재시작하는 옵션이라 혼자
> 코드를 고칠 때만 씁니다. 젯슨이 붙어 있는 상태에서 켜두면 연동 중에
> 연결이 끊겼다 붙었다 합니다.

서버가 뜨면 http://127.0.0.1:8000/docs 에서 API 문서를 확인할 수 있습니다.
팀에 공유하는 주소는 `ws://<이 노트북 LAN IP>:8000/ws/...` 입니다
(`hostname -I` 로 확인).

## API 요약

| 종류 | 경로 | 설명 |
|---|---|---|
| WebSocket | `/ws/jetson` | 젯슨(UGV)이 데이터를 보내는 채널 |
| WebSocket | `/ws/frontend` | 대시보드가 실시간 알림을 받는 채널 |
| GET | `/api/init-data` | 대시보드 초기 3D 맵 정보 |
| GET | `/api/history` | 과거 이벤트 로그 조회 (최근 50건) |

## 젯슨 → 서버 메시지 형식 (v1.5 확정)

> **정본은 [`docs/interface.md`](../docs/interface.md) 입니다.** 아래는 발췌이며,
> 키 이름·타입이 이 문서와 다르면 `docs/interface.md` 쪽이 맞습니다.

위험 이벤트 (② — DB 저장 대상):

```json
{
  "event_type": "fire",
  "confidence": 0.92,
  "depth_xyz": [1.1, 2.2, 0.8],
  "ekf_global": [3.2, 7.8]
}
```

위치 핑 (① — DB 저장 안 함, 0.5초 주기):

```json
{"event_type": "position", "ekf_global": [3.2, 7.8]}
```

- `event_type` — 위험 이벤트 허용 값 4개: `fallen_person` · `fire` · `no_helmet` · `ship_defect`
- `confidence` — YOLO conf 그대로 (0.5~1.0)
- `depth_xyz` — **카메라 기준** 객체 3D 좌표 `[X, Y, Z]`, 미터
- `ekf_global` — **map 기준** 차체 절대좌표 `[x, y]`, 미터. ekf_global(EKF 융합) 출력값이다.
  UWB 원시좌표(`uwb_frame`)가 아니라 EKF가 map으로 변환·융합한 뒤의 값이므로,
  서버·프론트는 추가 좌표변환 없이 그대로 쓰면 됩니다.
- `timestamp` — 서버가 붙이므로 젯슨은 보내지 않습니다.

나머지 메시지(③ block_level, ④ ship_pose, ⑥ stream_boost, ⑦ webrtc_signal,
⑧ event_ack)와 전체 필드 표는 `docs/interface.md`를 참고하세요.

> ⚠️ 서버는 모르는 `event_type`을 에러 없이 조용히 무시합니다. 철자를 반드시 위 표에서
> 복사해 쓰세요 — 오타가 나면 아무 일도 일어나지 않아 디버깅이 어렵습니다.

## 주의사항

- `.env`는 절대 커밋하지 마세요 (`.gitignore`로 차단되어 있음).
- 패키지를 새로 설치했다면 `pip freeze > requirements.txt`로 버전 목록을 갱신하고 함께 커밋하세요.
- 팀 공통 개발 규칙(브랜치 전략, 커밋 메시지)은 저장소 루트의 `CONTRIBUTING.md`를 참고하세요.
