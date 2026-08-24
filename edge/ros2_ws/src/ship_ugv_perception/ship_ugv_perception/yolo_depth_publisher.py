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
import time
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
        # ★ 화면 중앙 뎁스 탐침 (2026-08-21 신설)
        #   배 중심 좌표를 잴 때 쓴다. 카메라 중심을 배 중심에 맞춰 세우는
        #   것이 약속이므로, **YOLO 가 배를 찾아줄 필요가 없다.** 중앙의
        #   뎁스만 있으면 된다.
        #   왜 필요한가: 이 모델이 배를 못 잡는다. 같은 구도에서 신뢰도가
        #   0.043 까지 떨어지고(임계값 0.1) 클래스도 level1/level3/level4 로
        #   흔들린다. 측정이 모델 상태에 좌우되면 안 된다.
        self.declare_parameter('center_probe_enabled', True)
        self.declare_parameter('center_probe_w', 80)   # 중앙 ROI 가로(px)
        self.declare_parameter('center_probe_h', 60)   # 중앙 ROI 세로(px)
        self.declare_parameter('center_probe_hz', 5.0)

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
        self.probe_on = self.get_parameter('center_probe_enabled').value
        self.probe_w = int(self.get_parameter('center_probe_w').value)
        self.probe_h = int(self.get_parameter('center_probe_h').value)
        self.probe_period = 1.0 / max(0.1, self.get_parameter('center_probe_hz').value)
        self._probe_last = 0.0

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
        # 화면 중앙 뎁스 탐침 결과 (measure_ship_center.py 가 쓴다)
        self.probe_pub = self.create_publisher(
            String, '/camera/center_probe', 10)
        self.raw_image_pub = self.create_publisher(
            CompressedImage, raw_image_topic, LOW_LATENCY_QOS)
        self.annotated_image_pub = self.create_publisher(
            CompressedImage, annotated_image_topic, LOW_LATENCY_QOS)

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
        self._latest_stamp = None
        self._latest_raw = None
        self._frame_stamp = None
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

    def _publish_center_probe(self, depth_image):
        """화면 정중앙 ROI 의 뎁스 중앙값을 카메라 좌표로 내보낸다.

        배 중심 좌표를 잴 때 쓴다(scripts/measure_ship_center.py --probe).
        카메라 중심을 배 중심에 맞춰 세우는 것이 약속이라 **YOLO 가 배를
        찾아줄 필요가 없다.** 검출을 거치지 않으므로 모델 성능과 무관하다.

        검출 경로와 같은 규약을 따른다: bbox 중앙 1/4 대신 화면 중앙 ROI 를
        쓰고, 유효 픽셀의 중앙값을 z 로 삼아 내부파라미터로 역투영한다.
        """
        now = time.time()
        if now - self._probe_last < self.probe_period:
            return
        self._probe_last = now

        h, w = depth_image.shape
        u, v = w // 2, h // 2
        x0, x1 = max(0, u - self.probe_w // 2), min(w, u + self.probe_w // 2)
        y0, y1 = max(0, v - self.probe_h // 2), min(h, v + self.probe_h // 2)
        roi = depth_image[y0:y1, x0:x1]
        valid = roi[roi > 0]
        ratio = len(valid) / roi.size if roi.size else 0.0
        if ratio < self.min_valid_ratio:
            # 유효 픽셀이 너무 적으면 중앙값이 배경에 오염된다. 보내지 않는다.
            return

        z_m = float(np.median(valid)) / 1000.0
        payload = {
            'depth_xyz': [(u - self.cx) * z_m / self.fx,
                          (v - self.cy) * z_m / self.fy,
                          z_m],
            'u': u, 'v': v,
            'valid_ratio': round(ratio, 3),
            'roi': [self.probe_w, self.probe_h],
            'class_id': 'center_probe',
        }
        self.probe_pub.publish(String(data=json.dumps(payload)))

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

                raw = np.frombuffer(color_frame.get_data(), dtype=np.uint8)
                # 카메라가 MJPG 로 주면(FFD8 = JPEG 시작표식) 그대로 흘려보낸다.
                # 아니면 예전처럼 디코딩→재인코딩으로 후퇴한다.
                is_jpeg = len(raw) > 2 and raw[0] == 0xFF and raw[1] == 0xD8

                depth_data = np.frombuffer(depth_frame.get_data(), dtype=np.uint16)
                depth_image = depth_data.reshape((depth_frame.get_height(), depth_frame.get_width()))

                if self.probe_on:
                    self._publish_center_probe(depth_image)

                if is_jpeg:
                    # ★ 원본 JPEG 를 그대로 발행 (2026-08-20).
                    #   예전에는 카메라가 준 MJPG 를 풀었다가(imdecode) 다시
                    #   압축해서(imencode) 보냈다. 초당 25.6번. 프로파일 실측으로
                    #   그 왕복이 YOLO 추론의 3배를 먹고 있었다
                    #   (인코딩 3.34초 + 디코딩 3.21초 vs 추론 3.1초 / 20초 기준).
                    #   송출에 필요한 건 JPEG 바이트뿐이라 왕복이 통째로 낭비다.
                    msg = CompressedImage()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.format = 'jpeg'
                    msg.data = raw.tobytes()
                    self.raw_image_pub.publish(msg)
                    color_image = None      # 디코딩은 추론할 때만 (4 Hz)
                else:
                    color_image = frame_to_bgr_image(color_frame)
                    if color_image is None:
                        # MJPG 도 아니고 디코딩도 안 되는 포맷. 이 스레드는 데몬이라
                        # 예외가 나면 노드는 안 죽고 캡처만 조용히 멈추는 "좀비
                        # 상태"가 되므로 반드시 방어.
                        continue
                    self._encode_and_publish(self.raw_image_pub, color_image)

                # ★ YOLO 처리 스레드(타이머)가 쓸 수 있게 최신 프레임 저장
                with self._frame_lock:
                    self._latest_raw = raw if is_jpeg else None
                    self._latest_color = color_image
                    self._latest_depth = depth_image
                    # ★ 이 프레임을 언제 찍었는지 남긴다 (2026-08-19).
                    #   아래 _process_frame 이 YOLO 추론을 마친 뒤 검출을
                    #   발행하는데, 그 사이 55~305 ms 가 걸린다. 소비자
                    #   (ship_survey_node, change_point)가 "지금" 로봇 위치로
                    #   좌표를 계산하면 그 시간만큼 로봇이 움직인 오차가
                    #   그대로 실린다. 실측: 주행 중 측량이 같은 배를 네 번 재서
                    #   중심이 최대 62 cm 흩어졌다(정지 시엔 오차 0).
                    self._latest_stamp = self.get_clock().now().to_msg()
            except Exception as e:
                self.get_logger().error(f"캡처 스레드 예외 (계속 재시도): {e}")
    def _get_latest_frame(self):
        with self._frame_lock:
            if self._latest_depth is None:
                return None, None
            raw = self._latest_raw
            color = None if self._latest_color is None else self._latest_color.copy()
            depth = self._latest_depth.copy()
            self._frame_stamp = getattr(self, '_latest_stamp', None)
        if color is None:
            if raw is None:
                return None, None
            # 락 밖에서 디코딩한다 — 캡처 스레드를 붙잡고 있지 않기 위해서.
            color = cv2.imdecode(raw, cv2.IMREAD_COLOR)
            if color is None:
                return None, None
        return color, depth

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
                # ★ 위치(맵 좌표) 기준 "이미 보고했나" 판단은 change_point_detector 가
                #   한다. 여기서 화면좌표(u,v)로 한 번 더 걸러 영구히 기억해두면
                #   (예전의 reported_tids/reported_fallbacks) 불을 옮기거나 다른
                #   자리에 새로 놓아도 화면상 비슷한 위치라는 이유로 아예
                #   발행조차 안 되는 사고가 난다(2026-08-24 실측: 불을 옮겨도
                #   팝업/정지 둘 다 안 뜸). 트랙별로 매 프레임 그냥 발행하고,
                #   실제 중복 제거는 map 좌표 기반인 change_point_detector 에
                #   전부 맡긴다.
                track_id = int(box.id[0])
                publish_ok = True
                key_info = f"tid={track_id}"

            else:
                # ★ 트랙 ID가 없는 검출: 노이즈 필터링 목적으로만 연속
                #   fallback_confirm_frames 프레임 동안 비슷한 화면 위치에
                #   잡혀야 발행한다. (위치 중복 제거는 change_point_detector 몫)
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
            # ★ stamp = 이 검출이 나온 **사진을 찍은 시각** (발행 시각이 아니다).
            #   소비자는 이 시각의 TF 를 조회해야 로봇 위치가 맞다.
            st = getattr(self, '_frame_stamp', None)
            payload = {
                'u': u, 'v': v, 'depth': z_m,
                'depth_xyz': [X, Y, z_m],
                'class_id': class_name, 'confidence': conf,
            }
            if st is not None:
                payload['stamp_sec'] = int(st.sec)
                payload['stamp_nanosec'] = int(st.nanosec)
            msg.data = json.dumps(payload)
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
