# 영상 송출 — video_streamer + mediamtx

젯슨의 카메라 영상을 프론트엔드로 보내는 두 조각. **둘 다 systemd 서비스**이고,
원본은 `/home/ship_yard/` 에 있다. 여기 있는 것은 그 사본이다 —
**젯슨이 죽으면 복구할 수 있어야 하므로 저장소에 둔다.**

## 구조

```
카메라 MJPEG
   ↓  YOLO 가 /camera/color/compressed_raw 로 발행 (박스 없는 원본)
video_streamer.py       구독해서 ffmpeg stdin 으로 흘림
   ↓  ffmpeg: MJPEG 디코딩 -> H.264 인코딩
   ↓  rtsp://127.0.0.1:8554/ugv1
mediamtx                RTSP 를 받아 WebRTC 로도 서빙
   ↓  WebRTC (포트 8889)
브라우저                http://<젯슨IP>:8889/ugv1 를 iframe 으로
```

**카메라를 직접 열지 않는다.** pyorbbecsdk 는 한 프로세스만 카메라를 열 수
있어서, YOLO 가 이미 연 것을 ROS 토픽으로 받아 쓴다. 그래서 USB 충돌이
원천적으로 없다.

## 실측 비용 (2026-08-27)

```
ffmpeg     42.2%  of 1 core   <- MJPEG 디코딩 + H.264 소프트웨어 인코딩
mediamtx    1.2%  of 1 core
──────────────────────────
합계       약 0.43 코어
```

`libx264` 대신 하드웨어 인코더(`h264_v4l2m2m`)를 쓰면 크게 줄 여지가 있다.
젯슨에 인코더는 있다(`ffmpeg -encoders | grep v4l2m2m`). 다만 지연·화질이
나빠지는 사례가 있어 **실측 후에 바꿀 것.** `video_streamer.py` 한 줄이다.

## 파일 위치 — 심링크로 하나만 둔다

```
/home/ship_yard/video_streamer.py  ->  edge/streaming/video_streamer.py (심링크)
```

systemd 가 `/home/ship_yard/video_streamer.py` 를 실행하는데, 그게 저장소
파일을 가리키는 심링크다. **사본을 두 벌 두지 않는다** — 젯슨 로컬본과
저장소본이 갈라지면 "고쳤는데 안 바뀐다" 가 된다. 2026-08-27 에 YOLO 에서
정확히 그 문제로 몇 시간을 썼다(systemd 가 옛 코드를 붙잡고 있었다).

`mediamtx.yml` 은 아직 사본이다 — mediamtx 가 심링크를 따라가긴 하지만
설정 변경이 드물어 그냥 두었다. 고칠 때는 **양쪽 다** 고칠 것.

## 중앙 미디어 서버로 옮기려면

지금은 mediamtx 가 **젯슨 위**에 있다. 산업 표준 배치는 미디어 서버가
**중앙**에 있고 엣지가 거기로 밀어올리는 것이다. 프로토콜과 부품은 이미
같으므로 **배치만 바꾸면 된다.**

```
지금:  젯슨 ──RTSP push──> mediamtx(젯슨)  ──WebRTC──> 시청자
표준:  젯슨 ──RTSP push──> mediamtx(중앙)  ──WebRTC──> 시청자들
```

**코드는 안 고친다.** 젯슨 쪽은 systemd 환경변수 한 줄이다:

```bash
# /etc/systemd/system/video-streamer.service 의 주석 한 줄을 푼다
Environment=RTSP_URL=rtsp://192.168.0.5:8554/ugv1

sudo systemctl daemon-reload
sudo systemctl restart video-streamer
sudo systemctl disable --now mediamtx     # 젯슨의 mediamtx 는 이제 불필요
```

서버 쪽은 같은 mediamtx 바이너리를 띄우고, 프론트의 `DIRECT_CAMERA_URL` 을
`http://<서버IP>:8889/ugv1` 로 바꾼다.

`video-streamer.service` 의 mediamtx 의존은 `Requires` 가 아니라 `Wants` 다.
`Requires` 였다면 젯슨에서 mediamtx 를 내리는 순간 video-streamer 도 안 뜬다.

**얻는 것:** 시청자가 늘어도 로봇이 아니라 서버가 감당한다. 다른 망에서도
접속된다. 중앙 인증·녹화를 붙일 수 있다.

**잃는 것:** 서버가 죽으면 영상도 죽는다(지금은 젯슨만 살아 있으면 나온다).
서버 쪽 방화벽에서 8554(RTSP 수신)·8889(WebRTC 송출)를 열어야 한다.

**시연 직전에는 하지 말 것.** 이득이 0(단일 LAN, 시청자 1~2명)인데 방화벽
승인 같은 데서 시간을 날리기 쉽다.

## ⚠️ 인증이 열려 있다

`mediamtx.yml` 의 `authInternalUsers` 가 `user: any` / 빈 비밀번호다.
**같은 망의 누구나 로봇 카메라를 볼 수 있다.** 시연장에서는 문제되지 않지만
현장 적용 시에는 반드시 막을 것.

## 설치 (젯슨을 새로 세팅할 때)

```bash
cp video_streamer.py mediamtx.yml ~/
sudo cp video-streamer.service mediamtx.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now mediamtx video-streamer
```

mediamtx 바이너리(`/home/ship_yard/mediamtx`)와 ffmpeg(`/home/ship_yard/ffmpeg`)는
용량 때문에 저장소에 없다. 공식 배포본을 받아 같은 경로에 두면 된다.
