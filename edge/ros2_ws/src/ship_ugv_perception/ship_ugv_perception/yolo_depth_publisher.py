cat > ~/smart-shipyard/edge/ros2_ws/src/ship_ugv_perception/ship_ugv_perception/yolo_depth_publisher.py << 'PYEOF'
#!/usr/bin/env python3
"""
yolo_depth_publisher.py
------------------------
Astra+ 카메라로 Color+Depth를 받아 YOLO로 객체를 검출하고,
검출된 각 객체의 (u, v, depth, depth_xyz)를 /event_detection/uvd 토픽에 발행한다.

[캡처 스레드 분리 - 지연 문제 해결]
캡처 스레드: 카메라 최대 속도로 계속 프레임을 읽어 원본을 즉시 발행하고,
최신 프레임을 락으로 보호된 공유 변수에 저장. ROS2 타이머(메인 스레드)는
공유 변수에서 최신 프레임을 꺼내 YOLO 추론 + 주석 영상 발행 + 이벤트 발행을
수행하며, 이 처리가 느려도 캡처/원본발행 속도에 전혀 영향 주지 않음.

[2단계 검출 - person crop 확대 재검출]
person이 검출되면, 그 영역만 잘라서(crop) 비율 유지한 채 확대(resize) 후
같은 모델로 다시 helmet/no_helmet만 검출한다. 화면 전체 기준으로는 안전모가
너무 작아 YOLO 내부 특징맵에서 정보가 손실되는 문제(작은 객체 탐지 취약점)를
완화하기 위함 (SAHI와 유사한 원리 - 새 정보를 만드는 게 아니라, 모델이 기존
정보를 놓치지 않게 '공간적 여유'를 늘려주는 것). cv2.resize는 단순 보간이라
원본에 없는 디테일을 만들어내진 않음(슈퍼레졸루션과는 다름).
crop에서 나온 좌표는 원본 프레임 좌표로 환산해서, 기존 1차 필터
(track_id/fallback 중복 방지) + depth 계산 + 발행 로직을 공통 함수
(_evaluate_box)로 재사용한다. crop 쪽 검출은 track_id가 없어 자동으로
fallback(위치 기반) 경로를 타며, 메인 검출과 위치가 겹치면 기존 dedup
로직이 자연스럽게 중복을 걸러준다.
"""

import json
import math
import os
import re
import threading
import numpy as np

import rclpy
from rclpy.node import Node
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
        self.declare_parameter('inference_interval_s', 0.1)
        # bbox 내 유효 depth 픽셀 비율이 이 값 미만이면 해당 검출 폐기
        # (작은 소품은 배경 depth가 섞여 median이 오염되는 실패 모드 방지)
        self.declare_parameter('min_valid_ratio', 0.2)

        # ★ person crop 2단계 검출 관련
        self.declare_parameter('person_crop_enabled', True)
        self.declare_parameter('person_crop_target_size', 640)
        self.declare_parameter('person_crop_min_size_px', 20)

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

        self.person_crop_enabled = self.get_parameter('person_crop_enabled').value
        self.person_crop_target_size = self.get_parameter('person_crop_target_size').value
        self.person_crop_min_size_px = self.get_parameter('person_crop_min_size_px').value

        self.model = YOLO(weights_path)
        self.get_logger().info(f"모델 클래스 목록: {self.model.names}")

        # helmet/no_helmet 클래스 인덱스 미리 찾아둠 (2단계 검출을 이걸로만 제한 -> 속도 절약)
        self._helmet_class_ids = [
            idx for idx, name in self.model.names.items()
            if name in ('helmet', 'no_helmet')
        ]
        if self.person_crop_enabled and not self._helmet_class_ids:
            self.get_logger().warn(
                "person_crop_enabled=True인데 모델에 helmet/no_helmet 클래스가 없음 - 2단계 검출 비활성화")
            self.person_crop_enabled = False

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
            f"annotated_topic={annotated_image_topic}(YOLO 처리 주기 종속), "
            f"person_crop_enabled={self.person_crop_enabled}"
        )

        # ★ 캡처 전담 스레드 시작 (카메라 최대 속도로 계속 실행)
        self._capture_stop = threading.Event()
        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()

        # ★ YOLO 처리는 이 타이머로만, 캡처 속도와 무관
        self.timer = self.create_timer(inference_interval, self._process_frame)

    # ------------------------------------------------------------------
    def _match_nearby(self, records, class_name, u, v):
        for r in records:
            if r['class'] != class_name:
                continue
            if math.hypot(u - r['u'], v - r['v']) < self.fallback_match_dist:
                return r
        return None

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
    # ★ 개별 박스 하나를 처리(1차 필터 -> depth 계산 -> 발행)하는 공통 로직.
    # track_id가 있는 박스(메인 검출)와, 없는 박스(person crop 2단계 검출) 둘 다
    # 이 함수를 거쳐서 동일하게 처리된다.
    def _evaluate_box(self, class_name, conf, x1, y1, x2, y2, track_id,
                       depth_image, display_image):
        if conf < self.conf_threshold:
            if self.debug_log:
                self.get_logger().info(
                    f"[디버그] conf 미달 스킵: class={class_name} conf={conf:.2f}")
            return

        u, v = int((x1 + x2) / 2), int((y1 + y2) / 2)

        publish_ok = False
        key_info = ""

        if is_level_class(class_name):
            publish_ok = True
            key_info = "level(항상발행)"

        elif track_id is not None:
            if track_id in self.reported_tids:
                self._draw_box(display_image, x1, y1, x2, y2, class_name, conf, None)
                return
            if self._match_nearby(self.reported_fallbacks, class_name, u, v):
                self.reported_tids.add(track_id)
                self._draw_box(display_image, x1, y1, x2, y2, class_name, conf, None)
                return
            self.reported_tids.add(track_id)
            publish_ok = True
            key_info = f"tid={track_id}"

        else:
            if self._match_nearby(self.reported_fallbacks, class_name, u, v):
                self._draw_box(display_image, x1, y1, x2, y2, class_name, conf, None)
                return
            cand = self._match_nearby(self.fallback_candidates, class_name, u, v)
            if cand is None:
                self.fallback_candidates.append({'class': class_name, 'u': u, 'v': v, 'count': 1})
                self._draw_box(display_image, x1, y1, x2, y2, class_name, conf, None)
                if self.debug_log:
                    self.get_logger().info(
                        f"[디버그] fallback 후보 신규: class={class_name} conf={conf:.2f} ({u},{v})")
                return
            cand['count'] += 1
            cand['u'], cand['v'] = u, v
            if self.debug_log:
                self.get_logger().info(
                    f"[디버그] fallback 카운트: class={class_name} conf={conf:.2f} "
                    f"count={cand['count']}/{self.fallback_confirm_frames}")
            if cand['count'] < self.fallback_confirm_frames:
                self._draw_box(display_image, x1, y1, x2, y2, class_name, conf, None)
                return
            self.fallback_candidates.remove(cand)
            self.reported_fallbacks.append({'class': class_name, 'u': u, 'v': v})
            publish_ok = True
            key_info = f"fallback({u},{v})"

        if not publish_ok:
            return

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
            return

        z_m = float(np.median(valid_depths)) / 1000.0
        X = (u - self.cx) * z_m / self.fx
        Y = (v - self.cy) * z_m / self.fy

        self._draw_box(display_image, x1, y1, x2, y2, class_name, conf, z_m)

        if self.debug_log:
            self.get_logger().info(
                f"[발행] class={class_name} {key_info} conf={conf:.2f} "
                f"xyz=({X:.3f},{Y:.3f},{z_m:.3f})m"
            )

        msg = String()
        msg.data = json.dumps({
            'u': u, 'v': v, 'depth': z_m,
            'depth_xyz': [X, Y, z_m],
            'class_id': class_name, 'confidence': conf,
        })
        self.pub.publish(msg)

    # ------------------------------------------------------------------
    # ★ person 영역을 crop -> 비율 유지 확대 -> helmet/no_helmet만 재검출
    def _detect_helmet_in_person_crop(self, color_image, px1, py1, px2, py2,
                                       depth_image, display_image):
        crop_w, crop_h = px2 - px1, py2 - py1
        if crop_w < self.person_crop_min_size_px or crop_h < self.person_crop_min_size_px:
            return

        person_crop = color_image[py1:py2, px1:px2]
        if person_crop.size == 0:
            return

        scale = self.person_crop_target_size / max(crop_w, crop_h)
        new_w, new_h = max(1, int(crop_w * scale)), max(1, int(crop_h * scale))
        person_crop_resized = cv2.resize(person_crop, (new_w, new_h))

        helmet_results = self.model(
            person_crop_resized, verbose=False, classes=self._helmet_class_ids
        )[0]

        if helmet_results.boxes is None:
            return

        for hbox in helmet_results.boxes:
            hx1, hy1, hx2, hy2 = hbox.xyxy[0].cpu().numpy()
            hconf = float(hbox.conf[0])
            hcls = int(hbox.cls[0])
            hclass_name = self.model.names[hcls]

            orig_x1 = px1 + hx1 / scale
            orig_y1 = py1 + hy1 / scale
            orig_x2 = px1 + hx2 / scale
            orig_y2 = py1 + hy2 / scale

            if self.debug_log:
                self.get_logger().info(
                    f"[디버그][person crop] class={hclass_name} conf={hconf:.2f} "
                    f"(원본좌표로 환산됨)")

            self._evaluate_box(
                hclass_name, hconf, orig_x1, orig_y1, orig_x2, orig_y2,
                None, depth_image, display_image
            )

    # ------------------------------------------------------------------
    # YOLO 처리 (캡처 스레드가 채워둔 최신 프레임을 가져다 씀, 카메라 속도와 무관)
    def _process_frame(self):
        color_image, depth_image = self._get_latest_frame()
        if color_image is None:
            return

        results = self.model.track(
            color_image, persist=True, verbose=False, tracker=self.tracker_config
        )[0]

        display_image = color_image.copy()

        num_boxes = 0 if results.boxes is None else len(results.boxes)
        if self.debug_log:
            self.get_logger().info(f"[디버그] 검출 개수: {num_boxes}")

        if results.boxes is None:
            self._encode_and_publish(self.annotated_image_pub, display_image)
            return

        # ★ 1단계: 메인 프레임 전체 기준 검출
        person_boxes = []
        for box in results.boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            class_name = self.model.names[cls]
            track_id = int(box.id[0]) if box.id is not None else None

            self._evaluate_box(class_name, conf, x1, y1, x2, y2, track_id,
                                depth_image, display_image)

            if class_name == 'person' and self.person_crop_enabled:
                person_boxes.append((int(x1), int(y1), int(x2), int(y2)))

        # ★ 2단계: person 영역마다 crop 확대 후 helmet/no_helmet 재검출
        for (px1, py1, px2, py2) in person_boxes:
            px1 = max(0, px1)
            py1 = max(0, py1)
            px2 = min(color_image.shape[1], px2)
            py2 = min(color_image.shape[0], py2)
            self._detect_helmet_in_person_crop(
                color_image, px1, py1, px2, py2, depth_image, display_image)

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
PYEOF