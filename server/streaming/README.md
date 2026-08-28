# 중앙 미디어 서버 (mediamtx)

서버 노트북(**192.168.0.5**, 공유기에서 고정 IP)에서 도는 영상 중계 서버.
**백엔드(FastAPI)와는 완전히 별개 프로세스다.**

```
젯슨 ──H.264 1개, RTSP push──▶ mediamtx(여기) ──WebRTC──▶ 브라우저 N명
        rtsp://192.168.0.5:8554/ugv1        http://192.168.0.5:8889/ugv1
```

2026-08-28 에 mediamtx 를 젯슨에서 이리로 옮겼다. 프로토콜도 부품도 그대로고
**위치만 바뀌었다.** 옮긴 이유와 배경은 [`docs/interface.md`](../../docs/interface.md) ⑤ 참조.

| | 옮기기 전 | 지금 |
|---|---|---|
| 시청자 2명 → 젯슨 부하 | 스트림 2개를 젯슨이 뿌림 | 젯슨은 항상 1개 |
| 다른 망에서 접속 | 불가 | 서버만 열면 됨 |
| 젯슨 CPU | 0.43 코어 | 0.42 코어 |

이전 직후 실측 (2026-08-28): 젯슨→서버 **152 KB/s = 1.2 Mbps**, H.264 1트랙.
`[path ugv1] stream is available and online, 1 track (H264)` — 재인코딩 없이
그대로 통과한다.

### 시청자를 늘려도 젯슨이 안 움직인다 (2026-08-28 실측)

이번 이전의 목적이 이것이고, 실제로 확인했다.

| | 시청자 1명 | 시청자 3명 | |
|---|---|---|---|
| 젯슨 ffmpeg CPU | 44.0% | 44.2% | 변화 없음 |
| 젯슨 업링크 | 161 KB/s | 156 KB/s | 변화 없음 (측정 노이즈) |
| **젯슨 8554 연결** | **1개** | **1개** | ← 핵심 |
| 젯슨 8889 연결 | — | **0개** | 브라우저가 젯슨에 안 붙는다 |

**8554 연결이 1개 그대로**라는 것이 증거다. 시청자가 30명이 돼도 젯슨은 하나만
올려보낸다 — 팬아웃 부담이 완전히 서버로 넘어갔다.

옮기기 전이었다면 브라우저 3개가 각각 젯슨의 mediamtx 에 붙어 연결 3개,
업링크 3배가 됐을 것이다.

> 보고서용 문장 — 중앙 미디어 서버 구조로 전환한 뒤, 시청자를 1명에서 3명으로
> 늘려도 엣지(젯슨)의 업링크 연결 수와 CPU 사용량이 변하지 않음을 실측으로
> 확인했다 (RTSP 세션 1개 유지, ffmpeg 44.0% → 44.2%). 팬아웃 부담이 로봇에서
> 서버로 이전되어, 관제 인원이 늘어도 주행 성능에 영향을 주지 않는다.

---

## 설치 (이 노트북 = Ubuntu 22.04 / x86_64)

바이너리는 55MB 라 저장소에 넣지 않는다. 홈에 풀어둔다:

```bash
mkdir -p ~/mediamtx && cd ~/mediamtx
curl -sL -o mediamtx.tar.gz \
  https://github.com/bluenviron/mediamtx/releases/download/v1.20.1/mediamtx_v1.20.1_linux_amd64.tar.gz
tar xzf mediamtx.tar.gz && rm mediamtx.tar.gz
./mediamtx --version      # v1.20.1
```

설정은 이 폴더의 `mediamtx.yml` 을 쓴다 (git pull 하면 설정도 같이 따라온다).

> 젯슨의 `edge/streaming/mediamtx.yml` 은 38KB 짜리 배포판 원본인데 실제로 바꾼
> 값이 하나도 없었다. 그대로 복사하면 "우리가 무엇을 정했는지"가 주석 더미에
> 파묻히므로, 여기서는 정한 것만 남기고 나머지는 기본값에 맡겼다.
> 동작은 같다. 기본값 전체는 `~/mediamtx/mediamtx.yml` 에 있다.

## 방화벽 — 여기서 제일 많이 막힌다

```bash
sudo bash server/streaming/setup-firewall.sh
```

| 포트 | 방향 | 쓰임 |
|---|---|---|
| 8554/tcp | 젯슨 → 서버 | RTSP push |
| 8889/tcp | 브라우저 → 서버 | WebRTC 시그널링 + 재생 페이지 |
| **8189/udp** | 브라우저 ↔ 서버 | **WebRTC ICE 미디어** |

> ### ⚠️ 8189/udp 를 빠뜨리면 검은 화면이 된다
>
> 8889 만 열면 **재생 페이지는 정상적으로 뜨는데 영상이 영원히 안 나온다.**
> WebRTC 는 시그널링만 8889(HTTP)로 하고, 실제 영상은 ICE 로 뚫은 UDP 로 흐르기
> 때문이다. 에러 메시지도 없어서 젯슨 송출 문제로 착각하기 쉽다.
>
> 젯슨 쪽 인계 문서에는 8554/8889 두 개만 적혀 있었다. 세 개다.

`setup-firewall.sh` 는 인터넷 전체가 아니라 **같은 공유기 안(192.168.0.0/24)만**
허용한다. 지금은 인증이 없으므로 이 제한이 유일한 방어선이다 (아래 참조).

확인:

```bash
# 젯슨에서
nc -zv 192.168.0.5 8554      # succeeded 가 나와야 함
```

## 상시 실행 (systemd)

```bash
sudo cp server/streaming/mediamtx.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now mediamtx

systemctl status mediamtx
journalctl -u mediamtx -f        # 젯슨이 붙으면 여기 로그가 뜬다
```

수동 실행 (디버깅용):

```bash
~/mediamtx/mediamtx ~/smart-shipyard/server/streaming/mediamtx.yml
```

## 검증 순서

순서를 지켜야 어디서 막혔는지 가려낼 수 있다.

1. **서버에서 mediamtx 를 띄운다** → `journalctl -u mediamtx -f` 를 켜둔다
2. **방화벽을 연다** → 젯슨에서 `nc -zv 192.168.0.5 8554` 가 succeeded
3. **젯슨이 송출 목적지를 바꾸고 재시작** (젯슨 담당)
   ```bash
   sudo systemctl edit --full video-streamer   # Environment= 주석 풀고 서버 IP
   sudo systemctl disable --now mediamtx        # 젯슨 mediamtx 내림
   ```
   → 서버 로그에 `[RTSP] [session ...] created by 192.168.0.6` 가 떠야 한다
4. **브라우저로 `http://192.168.0.5:8889/ugv1` 을 직접 연다**
   → **여기서 영상이 나와야 한다.** 안 나오면 방화벽(특히 8189/udp)이나 젯슨 송출 문제다.
   대시보드를 열어보는 것은 그다음이다 — 한 번에 열면 원인이 둘로 갈린다
5. **대시보드**를 열어 영상 패널 확인
6. **탭 두세 개로 동시에** 열고 젯슨 CPU 를 본다
   → 시청자가 늘어도 젯슨 CPU 가 안 오르는 것이 이번 작업의 목적이다

## 녹화

`mediamtx.yml` 에서 켜져 있다. **보관 2시간.**

대시보드 상단의 **[🎬 녹화]** 버튼으로 켜고 끌 수 있다(재시작 불필요).
같은 화면에서 용량 확인·삭제와 지난 영상 보기도 된다.

| | |
|---|---|
| 저장 위치 | `~/mediamtx/recordings/ugv1/` (저장소 밖) |
| 형식 | fMP4, 1시간 단위 조각 |
| 용량 | **실측 8.7 MB/분 = 시간당 525 MB** (2026-08-29, 1분간 측정) |
| 2시간 상주 | 약 **1 GB** |

> ### 왜 2시간인가
>
> 지금 필요한 것은 "방금 무슨 일이 있었나" 를 되짚는 것이다. 2시간이면 1 GB 로
> 거의 부담이 없다. 더 길게 두려면 `recordDeleteAfter` 를 늘리되 `df -h` 로
> 여유를 먼저 본다 (48시간이면 약 25 GB).
>
> ⚠️ **`recordDeleteAfter` 없이 켜면 디스크가 찬다.** 둘은 항상 같이 간다.

용량 확인:

```bash
du -sh ~/mediamtx/recordings/
df -h /home
```

### 제어 API (127.0.0.1 전용)

`api: true` / `apiAddress: 127.0.0.1:9997`. 대시보드의 녹화 버튼이 백엔드를 거쳐
이걸 부른다.

> **밖으로 열지 말 것.** 이 API 는 설정을 바꿀 수 있다. 백엔드가 같은 노트북에
> 있으므로 루프백이면 충분하고, 방화벽에 포트를 열 필요도 없다.

### 그 순간 영상 꺼내기

`playback: true` 라 재생 API 가 열려 있다(9996/tcp). 이벤트의 `timestamp` 로
그때 영상을 자를 수 있다:

```
http://192.168.0.5:9996/get?path=ugv1&start=<ISO시각>&duration=15
```

**몽고에 녹화용 필드를 새로 두지 않는다.** 사진은 이벤트 1건 = 파일 1개라 URL 을
저장하지만, 녹화는 시간축으로 이어지므로 이벤트의 `timestamp` 로 조회하면 된다.

## 되돌리기

1. 프론트 `DIRECT_CAMERA_URL` 을 `http://192.168.0.6:8889/ugv1` 로 되돌린다
2. 젯슨에서 `Environment=RTSP_URL=...` 줄을 다시 주석 처리
3. 젯슨에서 `sudo systemctl enable --now mediamtx`

## 하지 말 것

- **`recordDeleteAfter` 없이 녹화를 켜지 말 것.** 디스크가 찬다. 둘은 항상 같이 간다.
- **mediamtx 에만 비밀번호를 걸지 말 것.** 백엔드에 인증이 없어서 프론트 소스에
  그 비밀번호가 그대로 박힌다. 백엔드 로그인이 생기면 그때
  `authMethod: http` + `authHTTPAddress` 로 백엔드에 물어보게 묶는다.
- **백엔드(FastAPI)로 영상을 중계하지 말 것.** MJPEG 이 되어 대역폭이 3~5배가
  되고 혼잡 제어를 잃는다.

## ⚠️ 인증이 없다 — 백엔드에는 생겼지만 영상은 아직

백엔드에 접근 암호가 생겼다(`DASHBOARD_PASSWORD`). 다만 **영상은 아직 열려 있다.**
지금은 같은 망의 누구나 보고 누구나 송출할 수 있고, 방화벽의 `192.168.0.0/24`
제한이 유일한 방어선이다.

### 붙이려면 정해야 할 것

mediamtx 는 `authMethod: http` + `authHTTPAddress` 로 외부에 인증을 위임할 수
있다. 백엔드에 `/api/mediamtx-auth` 자리는 잡아뒀다. 막힌 지점은 **토큰을 어떻게
실어 보내느냐** 다:

- 브라우저는 백엔드가 아니라 **mediamtx 에 직접** 붙는다(그것이 이 구조의 요점이다)
- `<iframe src="http://…:8889/ugv1">` 에는 헤더를 붙일 수 없다
- 쿼리스트링에 토큰을 넣으면 mediamtx 로그와 브라우저 이력에 남는다
- 쿠키를 쓰려면 mediamtx 와 백엔드가 같은 오리진이어야 해서 리버스 프록시가 필요하다

**리버스 프록시(nginx 등)로 둘을 한 오리진에 두는 것**이 정석이지만, 그러면
지금의 "브라우저가 mediamtx 에 직결" 구조가 바뀐다. 그 결정을 하기 전에는 켜지
않는다 — 반쯤 걸어두면 영상이 안 나오는 원인만 하나 더 늘어난다.

> mediamtx 에만 비밀번호를 거는 것은 여전히 의미가 없다. 프론트 소스에 그 비밀번호가
> 박히기 때문이다.

## 참고 — 로그에 뜨는 "RTP packets are too big"

```
[path ugv1] RTP packets are too big (1460 > 1440), remuxing them into smaller ones
```

**정상이다. 재인코딩이 아니다.** 젯슨의 ffmpeg 가 이더넷 MTU(1500) 기준으로
RTP 패킷을 만드는데, mediamtx 는 WebRTC 로 내보낼 때 더 작은 상한(1440)을 쓴다.
그래서 패킷을 쪼개기만 한다 — H.264 데이터 자체는 손대지 않는다.
바로 윗줄의 `stream is available and online, 1 track (H264)` 이 그 증거다.

"remuxing" 이라는 말 때문에 CPU 를 먹는 재인코딩으로 오해하기 쉽다.
없애고 싶으면 젯슨 ffmpeg 에 `-pkt_size 1200` 을 주면 되지만, 굳이 할 필요는 없다.

## 참고 — 포트가 백엔드와 겹쳐 보이는 것

`ss -lntup` 을 보면 mediamtx 가 **UDP 8000** 을 잡고 있다. 백엔드는 **TCP 8000**
이라 서로 다른 소켓이고 충돌하지 않는다. 놀라지 말 것.
