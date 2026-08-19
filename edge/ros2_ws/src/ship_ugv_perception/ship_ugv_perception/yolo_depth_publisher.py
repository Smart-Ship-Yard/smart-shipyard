#!/usr/bin/env python3
"""
yolo_depth_publisher.py
------------------------
Astra+ 카메라로 Color+Depth를 받아 YOLO로 객체를 검출하고,
검출된 각 객체의 (u, v, depth, depth_xyz)를 /event_detection/uvd 토픽에 발행한다.

[2026-07-14 지연 문제 근본 해결]
기존에는 카메라 캡처 -> 원본 발행 -> YOLO 추론 -> 주석 발행이 전부 하나의
콜백(타이머) 안에서 순차 실행되어, "원본을 먼저 발행"하더라도 다음 프레임을
가져오는 시점 자체가 YOLO 추론 속도에 종속되어 있었음 (추론이 느리면 전체
루프가 느려져서 원본 영상도 결과적으로 버벅임).

이번 수정: 카메라 캡처 전담 스레드를 분리함.
- 캡처 스레드: 카메라 최대 속도로 계속 프레임을 읽어 원본을 즉시 발행하고,
  최신 프레임을 락으로 보호된 공유 변수에 저장.
- ROS2 타이머(메인 스레드): 공유 변수에서 최신 프레임을 꺼내 YOLO 추론 +
  주석 영상 발행 + 이벤트 발행을 수행. 이 처리가 느려도 캡처/원본발행 속도에
  전혀 영향 주지 않음.
"""

import json
import math
import os
import re
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

from pyorbbecsdk import Pipeline, Config, OBSensorType, OBAlignMode
from ultralytics import YOLO
import cv2


def frame_to_bgr_image(frame):
    data = np.frombuffer(frame.get_data(), dtype=np.uint8)
    return cv2.imdecode(data, cv2.IMREAD_COLOR)


def is_level_class(class_name: str) -> bool:
    return re.search(r'(\d+)', str(class_name)) is not None


LOW_LATENCY_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class YoloDepthPublisher(Node):

    def __init__(self):
        super().__init__('yolo_depth_publisher')

        share_dir = get_package_share_directory('ship_ugv_perception')
        default_weights = os.path.join(share_dir, 'weights', 'best.pt')
        default_tracker_config = os.path.join(share_dir, 'config', 'custom_tracker.yaml')

        self.declare_parameter('weights_path', default_weights)
        self.declare_parameter('detection_topic', '/event_detection/uvd')
        self.declare_parameter('confidence_threshold', 0.2)
        self.declare_parameter('debug_log', True)
        self.declare_parameter('fallback_confirm_frames', 3)
        self.declare_parameter('fallback_match_dist_px', 60.0)
        self.declare_parameter('tracker_config', default_tracker_config)
        self.declare_parameter('raw_image_topic', '/camera/color/compressed_raw')
        self.declare_parameter('annotated_image_topic', '/camera/color/compressed')
        self.declare_parameter('raw_image_jpeg_quality', 60)
        # ★ YOLO 처리 주기 (카메라 fps와 분리됨, 이 값만큼만 추론 시도)
        # ★ 0.1 (10 Hz) -> 0.25 (4 Hz)  (2026-08-19)
        #   시연에서는 YOLO + 영상송출 + Nav2 를 동시에 돌려야 하는데 실측하니
        #   합계가 3.7~4.0 / 6코어였고 그중 YOLO 혼자 1.1~1.3코어였다.
        #   CPU 가 포화되면 Nav2 lifecycle 전환이 타임아웃되어 **브링업이
        #   통째로 실패한다** (2026-08-18 실제로 겪음).
        #   4 Hz 로 낮추면 0.5~0.7코어가 생긴다.
        #
        #   ⚠️ 영상 송출 프레임률과는 무관하다. 송출용 원본 프레임은
        #      _capture_loop() 이 별도 스레드에서 카메라 fps 그대로 내보내고
        #      (raw_image_pub -> /camera/color/compressed_raw -> video_streamer.py),
        #      이 타이머는 추론과 '박스 그린 영상'만 담당한다.
        #      실제로 느려지는 것은 검출 반응 시간뿐이다(최악 100ms -> 250ms).
        #      화재·쓰러진 사람은 초 단위 사건이라 문제되지 않고, 로봇이
        #      0.25 m/s 로 움직이므로 프레임 사이 이동도 6 cm 뿐이다.
        self.declare_parameter('inference_interval_s', 0.25)
        # bbox 내 유효 depth 픽셀 비율이 이 값 미만이면 해당 검출 폐기
        # (작은 소품은 배경 depth가 섞여 median이 오염되는 실패 모드 방지)
        self.declare_parameter('min_valid_ratio', 0.2)

        weights_path = self.get_parameter('weights_path').value
        topic = self.get_parameter('detection_topic').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.debug_log = self.get_parameter('debug_log').value
        self.fallback_confirm_frames = self.get_parameter('fallback_confirm_frames').value
        self.fallback_match_dist = self.get_parameter('fallback_match_dist_px').value
        self.tracker_config = self.get_parameter('tracker_config').value
        raw_image_topic = self.get_parameter('raw_image_topic').value
        annotated_image_topic = self.get_parameter('annotated_image_topic').value
        self.raw_jpeg_quality = self.get_parameter('raw_image_jpeg_quality').value
        inference_interval = self.get_parameter('inference_interval_s').value
        self.min_valid_ratio = self.get_parameter('min_valid_ratio').value

        # ★ debug_log 를 실행 중에 바꿀 수 있게 한다 (2026-08-19).
        #   지금까지는 위에서 값을 한 번 읽어 self.debug_log 에 넣어두고 다시
        #   읽지 않았다. 그래서
        #       ros2 param set /yolo_depth_publisher debug_log false
        #   가 "Set parameter successful" 을 반환하는데도 로그가 계속 찍혔다.
        #   파라미터 서버는 값을 받아들였지만 노드가 그 사실을 모르기 때문이다.
        #   **성공이라고 답이 오는데 아무 일도 안 일어나는 것**이 제일 나쁘다.
        #   (wheel_odom_bridge 의 enable_heading_hold 도 같은 문제였다.)
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.model = YOLO(weights_path)
        self.get_logger().info(f"모델 클래스 목록: {self.model.names}")
        self.pub = self.create_publisher(String, topic, 10)
        self.raw_image_pub = self.create_publisher(
            CompressedImage, raw_image_topic, LOW_LATENCY_QOS)
        self.annotated_image_pub = self.create_publisher(
            CompressedImage, annotated_image_topic, LOW_LATENCY_QOS)

        self.reported_tids = set()
        self.reported_fallbacks = []
        self.fallback_candidates = []

        # ★ 캡처 스레드와 공유할 최신 프레임 상태
        self._latest_color = None
        self._latest_depth = None
        self._frame_lock = threading.Lock()

        self.pipeline = Pipeline()
        config = Config()

        profile_list = self.pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
        color_profile = profile_list.get_default_video_stream_profile()
        config.enable_stream(color_profile)

        profile_list = self.pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
        depth_profile = profile_list.get_default_video_stream_profile()
        config.enable_stream(depth_profile)

        config.set_align_mode(OBAlignMode.SW_MODE)
        self.pipeline.start(config)

        camera_param = self.pipeline.get_camera_param()
        intrinsics = camera_param.rgb_intrinsic
        self.fx, self.fy = intrinsics.fx, intrinsics.fy
        self.cx, self.cy = intrinsics.cx, intrinsics.cy

        self.get_logger().info(
            f"YOLO+Depth publisher 시작, weights={weights_path}, "
            f"raw_topic={raw_image_topic}(캡처 스레드, 항상 빠름), "
            f"annotated_topic={annotated_image_topic}(YOLO 처리 주기 종속)"
        )

        # ★ 캡처 전담 스레드 시작 (카메라 최대 속도로 계속 실행)
        self._capture_stop = threading.Event()
        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()

        # ★ YOLO 처리는 이 타이머로만, 캡처 속도와 무관
        self.timer = self.create_timer(inference_interval, self._process_frame)

    def _match_nearby(self, records, class_name, u, v):
        for r in records:
            if r['class'] != class_name:
                continue
            if math.hypot(u - r['u'], v - r['v']) < self.fallback_match_dist:
                return r
        return None

    def _on_set_parameters(self, params):
        """실행 중 파라미터 변경을 캐시된 값에 반영한다."""
        for p in params:
            if p.name == 'debug_log':
                self.debug_log = bool(p.value)
                self.get_logger().info(
                    f'debug_log -> {self.debug_log} (즉시 반영됨)')
            elif p.name == 'confidence_threshold':
                self.conf_threshold = float(p.value)
                self.get_logger().info(
                    f'confidence_threshold -> {self.conf_threshold} (즉시 반영됨)')
        return SetParametersResult(successful=True)

    def _encode_and_publish(self, publisher, image, quality=None):
        q = quality if quality is not None else self.raw_jpeg_quality
        ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, q])
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = 'jpeg'
        msg.data = encoded.tobytes()
        publisher.publish(msg)

    def _draw_box(self, image, x1, y1, x2, y2, class_name, conf, z_m):
        p1 = (int(x1), int(y1))
        p2 = (int(x2), int(y2))
        color = (0, 255, 0)
        cv2.rectangle(image, p1, p2, color, 2)
        label = f"{class_name} {conf:.2f} {z_m:.2f}m" if z_m is not None else f"{class_name} {conf:.2f}"
        text_pos = (p1[0], max(0, p1[1] - 8))
        cv2.putText(image, label, text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    # ------------------------------------------------------------------
    # ★ 캡처 전담 스레드: YOLO와 완전히 독립적으로, 카메라 최대 속도로 실행
    def _capture_loop(self):
        while not self._capture_stop.is_set():
            try:
                frames = self.pipeline.wait_for_frames(100)
                if frames is None:
                    continue

                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                if color_frame is None or depth_frame is None:
                    continue

                color_image = frame_to_bgr_image(color_frame)
                if color_image is None:
                    # MJPG가 아닌 포맷으로 스트림이 잡히면 imdecode가 None을 반환함.
                    # 이 스레드는 데몬 스레드라 여기서 예외가 나면 노드는 안 죽고
                    # 캡처만 조용히 영원히 멈추는 "좀비 상태"가 되므로 반드시 방어.
                    continue

                depth_data = np.frombuffer(depth_frame.get_data(), dtype=np.uint16)
                depth_image = depth_data.reshape((depth_frame.get_height(), depth_frame.get_width()))

                # ★ 원본을 즉시 발행 (YOLO 대기 없음, 이게 핵심)
                self._encode_and_publish(self.raw_image_pub, color_image)

                # ★ YOLO 처리 스레드(타이머)가 쓸 수 있게 최신 프레임 저장
                with self._frame_lock:
                    self._latest_color = color_image
                    self._latest_depth = depth_image
            except Exception as e:
                self.get_logger().error(f"캡처 스레드 예외 (계속 재시도): {e}")
    def _get_latest_frame(self):
        with self._frame_lock:
            if self._latest_color is None or self._latest_depth is None:
                return None, None
            return self._latest_color.copy(), self._latest_depth.copy()

    # ------------------------------------------------------------------
    # YOLO 처리 (캡처 스레드가 채워둔 최신 프레임을 가져다 씀, 카메라 속도와 무관)
    def _process_frame(self):
        color_image, depth_image = self._get_latest_frame()
        if color_image is None:
            return

        results = self.model.track(
            color_image,
            persist=True,
            verbose=False,
            tracker=self.tracker_config
        )[0]

        display_image = color_image.copy()

        num_boxes = 0 if results.boxes is None else len(results.boxes)
        if self.debug_log:
            # ★ 5초에 한 번만 찍는다 (2026-08-18).
            #   프레임마다(약 10 Hz) 찍으면 터미널이 이 한 줄로 도배돼
            #   같은 창에 뜨는 다른 노드의 경고·에러가 전부 묻힌다.
            #   실제로 라이다가 죽은 것도, Nav2 브링업이 실패한 것도
            #   이런 도배 때문에 못 보고 지나갔다.
            #   검출이 있을 때는 어차피 아래에서 따로 로그가 나가므로
            #   여기서 매 프레임 찍을 이유가 없다.
            self.get_logger().info(f"[디버그] 검출 개수: {num_boxes}",
                                   throttle_duration_sec=5.0)

        if results.boxes is None:
            self._encode_and_publish(self.annotated_image_pub, display_image)
            return

        for box in results.boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            class_name = self.model.names[cls]

            if conf < self.conf_threshold:
                continue

            u, v = int((x1 + x2) / 2), int((y1 + y2) / 2)

            publish_ok = False
            key_info = ""

            if is_level_class(class_name):
                publish_ok = True
                key_info = "level(항상발행)"

            elif box.id is not None:
                track_id = int(box.id[0])
                if track_id in self.reported_tids:
                    self._draw_box(display_image, x1, y1, x2, y2, class_name, conf, None)
                    continue
                if self._match_nearby(self.reported_fallbacks, class_name, u, v):
                    self.reported_tids.add(track_id)
                    self._draw_box(display_image, x1, y1, x2, y2, class_name, conf, None)
                    continue
                self.reported_tids.add(track_id)
                publish_ok = True
                key_info = f"tid={track_id}"

            else:
                if self._match_nearby(self.reported_fallbacks, class_name, u, v):
                    self._draw_box(display_image, x1, y1, x2, y2, class_name, conf, None)
                    continue
                cand = self._match_nearby(self.fallback_candidates, class_name, u, v)
                if cand is None:
                    self.fallback_candidates.append({'class': class_name, 'u': u, 'v': v, 'count': 1})
                    self._draw_box(display_image, x1, y1, x2, y2, class_name, conf, None)
                    continue
                cand['count'] += 1
                cand['u'], cand['v'] = u, v
                if cand['count'] < self.fallback_confirm_frames:
                    self._draw_box(display_image, x1, y1, x2, y2, class_name, conf, None)
                    continue
                self.fallback_candidates.remove(cand)
                self.reported_fallbacks.append({'class': class_name, 'u': u, 'v': v})
                publish_ok = True
                key_info = f"fallback({u},{v})"

            if not publish_ok:
                continue

            box_w, box_h = int(x2 - x1), int(y2 - y1)
            y_min = max(0, v - box_h // 4)
            y_max = min(depth_image.shape[0], v + box_h // 4)
            x_min = max(0, u - box_w // 4)
            x_max = min(depth_image.shape[1], u + box_w // 4)
            depth_roi = depth_image[y_min:y_max, x_min:x_max]
            valid_depths = depth_roi[depth_roi > 0]
            valid_ratio = len(valid_depths) / depth_roi.size if depth_roi.size > 0 else 0.0

            if valid_ratio < self.min_valid_ratio:
                if self.debug_log:
                    self.get_logger().debug(
                        f"[{class_name}] 유효 depth 비율 {valid_ratio:.2f} < "
                        f"{self.min_valid_ratio} - 검출 폐기"
                    )
                self._draw_box(display_image, x1, y1, x2, y2, class_name, conf, None)
                continue

            z_m = float(np.median(valid_depths)) / 1000.0
            X = (u - self.cx) * z_m / self.fx
            Y = (v - self.cy) * z_m / self.fy

            self._draw_box(display_image, x1, y1, x2, y2, class_name, conf, z_m)

            if self.debug_log:
                self.get_logger().info(
                    f"[발행] class={class_name} {key_info} conf={conf:.2f} xyz=({X:.3f},{Y:.3f},{z_m:.3f})m"
                )

            msg = String()
            msg.data = json.dumps({
                'u': u, 'v': v, 'depth': z_m,
                'depth_xyz': [X, Y, z_m],
                'class_id': class_name, 'confidence': conf,
            })
            self.pub.publish(msg)

        self._encode_and_publish(self.annotated_image_pub, display_image)

    def destroy_node(self):
        self._capture_stop.set()
        self._capture_thread.join(timeout=2.0)
        self.pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloDepthPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
