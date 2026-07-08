# 팀 개발 가이드 (CONTRIBUTING)

처음 오신 분은 위에서부터 순서대로 읽으면 됩니다.

---

## 1. 저장소 구조

저장소 루트(최상위)에는 아래 것들만 둡니다.

```
smart-shipyard/
├── README.md          ← 프로젝트 전체 소개
├── CONTRIBUTING.md    ← 이 문서 (팀 공통 규칙)
├── .gitignore         ← git이 무시할 파일 목록 (아래 4번 참고)
├── backend/           ← FastAPI 서버 (담당: 이정기)
├── frontend/          ← React 3D 대시보드 (담당: 고명재)
└── edge/              ← 젯슨(UGV)에서 실행되는 코드 전부
                          (AI 비전: 이주현 / 자율주행 ROS 2: 전원)
```

**각 파트 폴더 안의 구조는 담당자 재량입니다.** 대신 두 가지는 지켜주세요.

1. 자기 파트 폴더 안에 **README.md를 만들어** 그 파트의 설명, 폴더 구조(트리),
   세팅·실행 방법을 적어둡니다. (예시: `backend/README.md` 참고)
2. 이 문서(CONTRIBUTING) 아래에 있는 자기 파트 세팅 섹션을 채워 넣습니다.

**심볼릭 링크를 쓰는 경우**(예: ROS 2 드라이버 링크)에는 반드시
**저장소 내부의 상대 경로**로 만드세요. `ln -s /home/이름/drivers/rplidar_ros...` 같은 절대 경로 링크는 다른 사람이 clone하면 깨집니다. 'ln -s ../../drivers/rplidar_ros'처럼 저장소 내부 상대 경로로 만드세요.

---

## 2. Git / GitHub 기본 개념

- **clone**: GitHub 저장소를 내 컴퓨터로 통째로 복제하는 것.
  다운로드 + 원격 저장소(origin) 등록 + main 브랜치 연결이 한 번에 되므로,
  clone 이후 추가 설정은 필요 없습니다. **저장소가 없는 처음에 딱 한 번만** 합니다. 
- **push**: 내 컴퓨터에 쌓아둔 커밋(저장 기록)들을 GitHub 저장소에 올리는 것.
- **pull**: GitHub 저장소의 최신 내용을 내 컴퓨터로 내려받아 내 폴더에 합치는 것.
  git push 혹은 git pull은 **내 컴퓨터의 프로젝트 최상위 폴더에 들어가서** 실행합니다.
- clone은 최초 한 번, 이후에는 push(올리기)와 pull(받기)만 반복합니다.
- GitHub에서 머지가 일어나도 **내 컴퓨터에는 pull 하기 전까지 반영되지 않습니다.**
- pull 할 때 내 컴퓨터에만 있는 파일(git이 추적하지 않는 개인 작업물)은
  지워지지 않고 그대로 유지됩니다.

---

## 3. 최초 참여 절차 (딱 한 번)

```bash
# 홈 디렉터리(home)에서 git clone 실행
# (기존 개인 작업 폴더[예:edge] "안"에서 clone 금지 — 저장소 안에 저장소가 중첩됩니다)
git clone https://github.com/Smart-Ship-Yard/smart-shipyard.git
cd smart-shipyard
```

GitHub에 프로젝트 기본 구조(뼈대)를 올려두었으니, clone으로 전체 구조를 받은 뒤
**자기 기존 작업물을 자기 파트 폴더 안으로 복사**해 넣고, 아래 5번 작업 흐름에 따라
브랜치를 만들어 커밋 → push → PR 하세요.

주의: clone은 파일을 내려받을 뿐, 개발 환경을 자동으로 만들어주지 않습니다.
venv 생성이나 패키지 설치 같은 세팅은 각 파트 섹션(6번~)의 명령어를
**각자 직접 실행**해야 합니다.

---

## 4. .gitignore 안내

- **.gitignore란?** git이 추적·업로드하지 않을 파일/폴더 목록을 적어두는 파일입니다.
  비밀값(.env), 각자 컴퓨터에서 재생성 가능한 것들(venv, 빌드 산출물)을 올리지 않기 위해 씁니다.
- **어디에 있나?** 저장소 최상위에 있으며, 이름이 점(.)으로 시작하는 숨김 파일이라
  `ls`로는 안 보입니다. `ls -a`로 확인하고, 내용은 `cat .gitignore`로 봅니다.
- 현재 무시 목록에는 `venv/`, `__pycache__/`, `.env`, colcon 빌드 산출물
  (`build/`, `install/`, `log/`), `node_modules/` 등이 포함되어 있습니다.
  루트 .gitignore의 규칙은 하위 폴더 전체에 적용되므로 보통은 신경 쓸 일이 없습니다.
- **단, push 전에 스스로 확인하세요.** 자기 폴더에 올라가면 안 되는 파일
  (비밀키, 대용량 모델 가중치, 개인 캐시 등)이 새로 생겼다면
  push 전에 반드시 .gitignore에 그 이름을 적어두어야 합니다.
  `git status` 목록에 올라가면 안 되는 파일이 보인다면 그게 신호입니다.

---

## 5. 일일 작업 흐름 (매일 이 순서로)

1. 작업 시작 전 최신 코드 받기:
   ```bash
   git pull origin main
   ```
2. 기능 단위로 브랜치 생성:
   ```bash
   git checkout -b feature/기능이름    # 예: feature/camera-api
   ```
3. 작업 후 커밋. **그날 작업물은 그날 브랜치에 push까지 해둡니다.**
   ```bash
  git status   # 뭐가 바뀌었는지 확인
  # 주의!! 내가 건드린 파일만!! 아래처럼 콕 집어 add하기!!!
  git add backend/<내가_수정한_파일1> backend/<내가_수정한_파일2>
  # 최상위 폴더(smart-shipyard)에서 add하면 다른 작업자의 최신 파일이 변경될 수 있음!!
  # 추가로 남의 파일 건들지 말기!(git add .을 할 거면 자신의 폴더[예를 들어 backend폴더] 안에서 하기)
   git commit -m "feat: 작업 내용"
   git push -u origin feature/기능이름
   ```
4. GitHub에서 **PR(Pull Request) 생성** — push한 사람이 직접 만들고,
   팀 채팅에 PR 링크를 공유해 리뷰를 요청합니다.
5. **팀원 1인 이상의 승인(Approve)** 을 받습니다. 코멘트로 수정 요청이 오면
   반영해서 다시 push 합니다 (같은 브랜치에 push하면 PR에 자동 반영됨).
6. 승인 후 **PR 작성자 본인이 Merge 버튼**을 누릅니다.
7. 머지 직후 뜨는 **Delete branch 버튼으로 해당 브랜치를 삭제**합니다.
   커밋 기록은 main과 PR 페이지에 영구 보존되므로 잃는 것이 없습니다.
8. 깃허브에서 PR 머지 후 로컬 정리하는 전형적인 순서 :
# 1. main(또는 master) 브랜치로 이동
git checkout main

# 2. 원격 저장소의 최신 내용을 받아오기 (머지된 내용 포함)
git pull origin main

# 3. 다 쓴 로컬 feature 브랜치 삭제
git branch -d feature-branch-이름

# 4. (선택) 원격에 브랜치가 남아있다면 삭제
git push origin --delete feature-branch-이름

# 5. (선택) 로컬에 남아있는 원격 브랜치 참조 정리
git fetch --prune



- `main` 브랜치는 항상 정상 동작하는 상태로 유지합니다.
- **main에는 직접 push할 수 없습니다.** GitHub 저장소 설정(Ruleset)으로
  이미 차단되어 있어서, main으로 push를 시도하면 거부(rejected)됩니다.
  따라서 모든 작업은 반드시 브랜치를 만들어 그 브랜치에 push한 뒤,
  PR을 통해서만 main에 합칠 수 있습니다.
- 머지(Merge) 역시 팀원 1인 이상의 승인(Approve)이 있어야만 가능하도록
  설정되어 있습니다. 승인 전에는 머지 버튼이 잠겨 있습니다.
- 개인 단독으로 24시간 이상 막히면 즉시 팀에 공유합니다 (Ground Rule).

---

## 6. 백엔드 세팅 (backend/)

```bash
cd backend

# 가상환경 생성 (backend 폴더 안에 만든다)
python3 -m venv venv

# 활성화
source venv/bin/activate

# 패키지 설치
pip install -r requirements.txt

# 환경 변수 파일 생성 후, MONGO_URL 등 실제 값 채우기
cp .env.example .env

# 서버 실행
uvicorn main:app --reload
```

터미널 맨 앞에 `(venv)` 표시가 뜨면 가상환경 활성화 성공입니다.
`.env`는 비밀값이 담기므로 절대 커밋하지 마세요.

작업을 마치고 가상환경에서 빠져나올 때는:
```bash
deactivate
```
(터미널 앞의 `(venv)` 표시가 사라지면 성공. 서버 실행 중이라면
먼저 Ctrl+C로 서버를 종료한 뒤 실행하세요.)

새 패키지를 설치했다면 목록을 갱신해서 함께 커밋하세요:
```bash
pip freeze > requirements.txt
```

## 7. 프론트엔드 세팅 (frontend/)

프론트엔드는 Node.js 기반이라 venv/requirements.txt 대신
`package.json`(패키지 목록)과 `npm install`(패키지 설치)을 사용합니다.
백엔드의 venv ↔ requirements.txt 관계가 여기서는 node_modules ↔ package.json입니다.

### Node.js 설치 (최초 1회, 없는 컴퓨터만)

버전 확인부터 해보세요. 두 명령이 모두 버전을 출력하면 이미 설치된 것입니다.

```bash
node -v    # v20 이상 권장
npm -v
```

없다면 nvm(Node 버전 관리자)으로 설치하는 것을 권장합니다 (Ubuntu 기준):

```bash
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.1/install.sh | bash
# 터미널을 완전히 닫았다가 다시 연 뒤
nvm install 20
```

(참고: `sudo apt install nodejs`는 아주 오래된 버전이 깔릴 수 있어 권장하지 않습니다.)

### 실행

```bash
cd frontend

# 패키지 설치 (최초 1회 — node_modules 폴더가 생성됨, 커밋 금지)
npm install

# 개발 서버 실행
npm run dev
```

터미널에 뜨는 주소(기본 http://localhost:5173)를 브라우저로 열면 대시보드가 뜹니다.
현재는 백엔드 없이 모의 이벤트로 단독 동작합니다. 종료는 Ctrl+C.

새 패키지를 설치했다면(`npm install 패키지명`) `package.json`과 `package-lock.json`이
자동으로 갱신되므로 **둘 다 함께 커밋**하세요. (`pip freeze` 같은 별도 명령은 없습니다.)

상세 구조와 백엔드 연동 지점은 `frontend/README.md`를 참고하세요.

## 8. 엣지 — AI 비전 세팅 (edge/ 내 비전 폴더) — 담당자가 채울 예정

젯슨에서 실행되는 YOLOv8/Pose 추론 코드입니다. 젯슨 전용 라이브러리
(TensorRT 등)가 포함되므로 일반 PC에서의 실행 가능 범위도 함께 명시합니다.

## 9. 엣지 — 자율주행 세팅 (edge/ 내 ROS 2 워크스페이스) — 담당자가 채울 예정

ROS 2(Nav2/slam_toolbox/EKF) 통합 워크스페이스입니다. colcon으로 빌드하며,
빌드 산출물(build/, install/, log/)은 .gitignore로 무시됩니다.
ROS 2 배포판 버전과 빌드·실행 방법을 담당자가 채워 넣습니다.

---

## 10. 커밋 메시지 규칙 (Conventional Commits)

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

## 11. 기초 운영 규칙 3줄

1. **`main`에 직접 push하지 않는다.** 항상 브랜치를 새로 파고 PR로 머지한다.
2. **작업 시작 전 `git pull origin main`부터 한다.** 안 그러면 남의 코드와 충돌(conflict)이 난다.
3. **커밋은 작게, 자주 한다.** 하루 작업을 한 커밋에 몰아넣지 말고 의미 단위로 쪼갠다.
