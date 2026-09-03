#!/usr/bin/env python3
"""
video_streamer.py
------------------
[2026-07-14 저지연 버전으로 수정]
yolo_depth_publisher.py가 `/camera/color/compressed_raw` 토픽으로 발행하는
(YOLO 추론을 거치지 않은, 박스 없는) JPEG 프레임을 구독해서, ffmpeg 프로세스의
표준입력(stdin)으로 그대로 흘려보내 RTSP(mediamtx)로 송출한다.

★ QoS를 BEST_EFFORT + depth=1로 맞췄다 (발행자인 yolo_depth_publisher.py와
  동일한 설정). ROS2는 발행자/구독자 QoS가 호환되지 않으면 연결 자체가 안
  맺어지거나, 맺어져도 오래된 프레임이 큐에 계속 쌓이면서 지연이 누적될 수
  있다. depth=1로 최신 프레임 한 장만 유지하도록 해서 "버퍼링으로 인한
  지연 누적" 문제를 해결한다.

★ 이 스크립트는 물리 카메라(/dev/video0 등)를 전혀 열지 않는다.
  카메라는 yolo_depth_publisher.py(pyorbbecsdk Pipeline)가 단독으로 소유하고,
  이 스크립트는 그 결과물(이미 인코딩된 JPEG)만 ROS2 토픽으로 받아서
  ffmpeg에 넘겨주기만 한다. 따라서 카메라 USB 동시 접근 충돌이 원천적으로 없다.

참고: 박스가 그려진 시각화 영상이 필요하면 image_topic 파라미터를
  `/camera/color/compressed` 로 바꿔서 실행하면 된다 (다만 이 경우 YOLO 추론
  시간만큼 약간의 지연이 있을 수 있음).

사전 준비:
  - yolo_depth_publisher.py가 이미 실행 중이어야 함 (카메라 소유자)
  - ROS2 환경 소싱 필요: source /opt/ros/humble/setup.bash (+ 워크스페이스 setup.bash)

실행:
  python3 video_streamer.py
  (또는 파라미터 지정: python3 video_streamer.py --ros-args -p image_topic:=/camera/color/compressed)
"""

import os
import subprocess
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage

# ★ 발행자(yolo_depth_publisher.py)와 동일한 저지연 QoS
#   - BEST_EFFORT: 패킷 유실이 있어도 재전송 대기 없이 계속 진행
#   - KEEP_LAST, depth=1: 큐에 최신 프레임 1개만 유지 (오래된 프레임 자동 폐기)
LOW_LATENCY_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

FFMPEG_PATH = os.environ.get("FFMPEG_PATH", "/home/ship_yard/ffmpeg")

# ★ 송출 목적지 (2026-08-28)
#   기본값은 젯슨 자신의 mediamtx 다. 중앙 미디어 서버로 옮길 때는 **코드를
#   고치지 말고** systemd 유닛의 Environment= 만 바꾼다:
#
#       Environment=RTSP_URL=rtsp://192.168.0.5:8554/ugv1
#
#   그러면 ffmpeg 이 인코딩한 H.264 스트림이 그대로 서버로 올라간다.
#   재인코딩이 없으므로 젯슨 CPU 는 오히려 mediamtx 몫(약 1.2%)만큼 줄고,
#   시청자가 몇 명이든 젯슨은 **스트림 하나만** 올려보낸다.
RTSP_URL = os.environ.get("RTSP_URL", "rtsp://127.0.0.1:8554/ugv1")


class VideoStreamer(Node):
    def __init__(self):
        super().__init__("video_streamer")
        self.declare_parameter("image_topic", "/camera/color/compressed_raw")
        topic = self.get_parameter("image_topic").value

        self.ffmpeg_proc = self._spawn_ffmpeg()

        self.sub = self.create_subscription(
            CompressedImage, topic, self._on_image, LOW_LATENCY_QOS
        )
        self.get_logger().info(f"video_streamer 시작 — '{topic}' 구독 → {RTSP_URL} 송출")

    def _spawn_ffmpeg(self):
        return subprocess.Popen(
            [
                FFMPEG_PATH,
                "-f", "mjpeg",
                "-use_wallclock_as_timestamps", "1",  # ROS 프레임 간격이 불규칙해도 타이밍 보정
                "-i", "-",                              # 표준입력에서 JPEG 프레임을 계속 읽음
                "-c:v", "libx264",
                "-preset", "ultrafast",
                "-tune", "zerolatency",
                "-f", "rtsp",
                "-rtsp_transport", "tcp",  # UDP 패킷 유실로 인한 잦은 재연결 방지
                RTSP_URL,
            ],
            stdin=subprocess.PIPE,
            stderr=subprocess.DEVNULL,  # 필요시 로그 보려면 이 줄 지우고 파일로 리다이렉트
        )

    def _on_image(self, msg: CompressedImage):
        if self.ffmpeg_proc.stdin is None or self.ffmpeg_proc.stdin.closed:
            return
        try:
            self.ffmpeg_proc.stdin.write(bytes(msg.data))
            self.ffmpeg_proc.stdin.flush()
        except BrokenPipeError:
            self.get_logger().error("ffmpeg 프로세스가 죽었습니다 — 재시작 시도")
            self.ffmpeg_proc = self._spawn_ffmpeg()

    def destroy_node(self):
        if self.ffmpeg_proc.stdin:
            try:
                self.ffmpeg_proc.stdin.close()
            except Exception:
                pass
        self.ffmpeg_proc.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoStreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
