# Frontend — 3D 디지털 트윈 관제 대시보드

스마트 조선소 디지털 트윈 시스템의 웹 관제 대시보드입니다.
브라우저에서 3D 가상 조선소(선박 블록)를 렌더링하고, 서버가 중계하는 AI 감지
이벤트를 받아 Red Alert Ping 점멸·공정률 색상·이벤트 로그로 시각화합니다.

담당: 고명재 (Frontend & 3D Engineer)

## 기술 스택

| 구분 | 기술 |
|---|---|
| UI 프레임워크 | React 19 |
| 3D 렌더링 | Three.js (WebGL) |
| 빌드 도구 | Vite 8 |
| 린터 | Oxlint |

## 폴더 구조

```
frontend/
├── index.html                      ← Vite 진입 HTML
├── package.json                    ← 의존성·실행 스크립트 정의
├── package-lock.json               ← 의존성 버전 고정 (커밋 대상, 지우지 말 것)
├── vite.config.js                  ← Vite 설정
├── .oxlintrc.json                  ← 린터 설정
├── public/                         ← 정적 파일 (파비콘 등)
└── src/
    ├── main.jsx                    ← React 앱 부트스트랩
    ├── App.jsx                     ← 최상위 컴포넌트 (대시보드 렌더링만 담당)
    ├── ShipyardTwinDashboard.jsx   ← ★ 핵심. 대시보드 전체 로직 (약 1,200줄)
    ├── App.css / index.css         ← 스타일시트
    └── assets/                     ← 이미지 리소스
```

## 구현된 기능 (ShipyardTwinDashboard.jsx 내부 구성 순서)

1. **도메인 상수** — 탐지 클래스별 라벨/심각도, 선박 5구획(S1~S5) 정의
2. **좌표 매핑 레이어** — 서버 좌표(구획 id + 로컬 오프셋) → Three.js 3D 좌표 변환
3. **Three.js 씬 매니저** — 선박 블록 렌더링, Red Alert Ping 점멸, 블록 클릭 픽킹
4. **CCTV 영상 렌더** — 상시 라이브 패널 + Click & View 팝업 (현재는 Canvas 모의 영상)
5. **메인 대시보드 UI** — 실시간 위험 요약 / 구획별 공정률 / 이벤트 로그 패널

## 실행 방법

Node.js가 필요합니다 (설치 방법은 루트 `CONTRIBUTING.md` 7번 참고).

```bash
cd frontend

# 1. 패키지 설치 (최초 1회, node_modules 폴더가 생성됨)
npm install

# 2. 개발 서버 실행
npm run dev
```

터미널에 표시되는 주소(기본 http://localhost:5173)를 브라우저로 열면 대시보드가 뜹니다.
현재는 백엔드 없이도 **모의 이벤트(2.2초 간격)** 로 동작하도록 되어 있습니다.

## 백엔드 연동 상태 (중요)

- 현재 데이터는 실제 서버가 아니라 `connectEventSource()`(약 114행)의 **mock 피드**입니다.
- 실제 연동 시 이 함수 내부만 `new WebSocket("ws://<서버주소>:8000/ws/frontend")` 로
  교체하면 됩니다 (주석으로 교체 지점 표시되어 있음).
- **주의: 프론트가 가정한 이벤트 스키마(`cls`, `blockId`, `local`, `conf`)와 서버가
  실제로 중계하는 스키마가 아직 다릅니다.** 정본은 [`docs/interface.md`](../docs/interface.md)
  (v1.5)이며, 위험 이벤트는 `event_type` · `confidence` · `depth_xyz` · `ekf_global` 4개 키로 옵니다.

  | 프론트 (`ShipyardTwinDashboard.jsx`) | 서버 (`docs/interface.md` ②) | 상태 |
  |---|---|---|
  | `cls` | `event_type` | 이름만 다름, 값은 동일 (`fallen_person`·`fire`·`no_helmet`·`ship_defect`) |
  | `conf` | `confidence` | 이름만 다름, 범위 동일 (0.5~1.0) |
  | `blockId` | — | **대응 키 없음.** 위험 이벤트에는 `block_id`가 없고 ③ `block_level` / ④ `ship_pose`에만 있음 |
  | `local {x,y,z}` | — | **대응 키 없음.** `local`은 블록 내부 0~1 정규화 좌표(`serverToWorld()` 약 84행)인데, 서버는 `ekf_global`(map 미터)과 `depth_xyz`(카메라 미터)만 보냄 |

  > 예전 `backend/README.md`에 있던 `uwb_xy` 키는 v1.5 이전 초안의 잔재이며 폐기됐습니다.
  > 젯슨은 `ekf_global`만 보냅니다 (`websocket_client.py` 약 220·307행).
  > 연동 시 `uwb_xy`를 찾도록 짜면 `undefined`를 받게 되니 주의하세요.

  참고: 위 불일치는 **아직 실제 문제를 일으키지 않습니다.** 현재 대시보드는 mock 피드로만
  돌고 있어(`new WebSocket`은 약 118행에 주석 처리) 백엔드와 연결된 적이 없습니다.
  매핑·자율주행·백엔드 중계는 이 스키마를 쓰지 않으므로 영향받지 않습니다.
  이 표는 실제 연동을 시작할 때 할 일 목록으로 보시면 됩니다.

- **남은 작업:** `ekf_global`(map 좌표계 절대 미터) → 블록 판정 + 블록 내부 0~1 좌표로
  바꾸는 변환이 필요합니다. 배 중심·크기·yaw는 ④ `ship_pose`(`map_xy`, `yaw`)로 오므로,
  이 값과 `ekf_global`을 함께 써서 `blockId`와 `local`을 계산하는 방식이 자연스럽습니다.
  구현 시 `CLASS_META`·좌표 매핑·`connectEventSource()`를 함께 수정해야 합니다.

## 주의사항

- `node_modules/`와 `dist/`(빌드 산출물)는 커밋하지 마세요 (루트 `.gitignore`로 차단됨).
- 새 패키지를 설치하면 `package.json`·`package-lock.json`이 자동 갱신되므로 둘 다 함께 커밋하세요.
- 팀 공통 개발 규칙(브랜치 전략, 커밋 메시지)은 저장소 루트의 `CONTRIBUTING.md`를 참고하세요.
