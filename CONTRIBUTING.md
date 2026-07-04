# 팀 개발 환경 세팅 가이드 (초보자용)

## 1. 저장소 받아오기 (Clone)

```bash
git clone https://github.com/bestbada/Smart-Shipyard-Shipbuilding-Process-Tracking-and-Digital-Twin-Monitoring-System.git
cd Smart-Shipyard-Shipbuilding-Process-Tracking-and-Digital-Twin-Monitoring-System
```

## 2. 가상환경(venv) 만들기

```bash
python -m venv venv

# macOS / Linux
source venv/bin/activate

# Windows
venv\Scripts\activate
```

터미널 맨 앞에 `(venv)` 표시가 뜨면 성공입니다.

## 3. 패키지 설치

```bash
pip install -r requirements.txt
```

## 4. .env 파일 만들기

`.env`는 몽고DB 접속 정보 같은 비밀값이 담겨 있어서 깃허브에 올라가지 않습니다 (`.gitignore`로 차단됨). 대신 `.env.example`을 복사해서 각자 `.env`를 만들고 실제 값을 채워 넣으세요.

```bash
cp .env.example .env
```

`.env`는 절대 커밋하지 마세요.

## 5. 서버 실행

```bash
uvicorn main:app --reload
```

---

## Git 브랜치 전략

- `main` 브랜치는 항상 정상 동작하는 상태로 유지합니다. **직접 push 금지.**
- 작업 시작 전 최신 코드 받기: `git pull origin main`
- 기능 단위로 브랜치 생성: `git checkout -b feature/기능이름` (예: `feature/camera-api`)
- 작업 완료 후 GitHub에서 Pull Request(PR) 생성 → 팀원 리뷰 → 승인 후 머지

## 커밋 메시지 규칙 (Conventional Commits)

형식: `타입: 내용`

| 타입 | 의미 |
|---|---|
| `feat` | 새로운 기능 추가 |
| `fix` | 버그 수정 |
| `docs` | 문서 수정 (README 등) |
| `chore` | 설정, 빌드, 잡일성 변경 |
| `refactor` | 동작 변화 없는 코드 개선 |

예시:
```
feat: 몽고DB 연결 로직 추가
fix: 웹소켓 연결 끊김 오류 수정
chore: requirements.txt 추가
```

## Git 기초 운영 규칙 3줄

1. **`main`에 직접 push하지 않는다.** 항상 브랜치를 새로 파고 PR로 머지한다.
2. **작업 시작 전 `git pull origin main`부터 한다.** 안 그러면 남의 코드와 충돌(conflict)이 난다.
3. **커밋은 작게, 자주 한다.** 하루 작업을 한 커밋에 몰아넣지 말고 의미 단위로 쪼갠다.
