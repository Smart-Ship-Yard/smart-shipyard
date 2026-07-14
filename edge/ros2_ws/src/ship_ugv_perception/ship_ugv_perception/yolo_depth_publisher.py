#!/usr/bin/env python3
"""
yolo_depth_publisher.py
------------------------
Astra+ 카메라로 Color+Depth를 받아 YOLO로 객체를 검출하고,
검출된 각 객체의 (u, v, depth, depth_xyz)를 /event_detection/uvd 토픽에 발행한다.

[2026-07-14 영상 지연 개선]
- 원본 프레임을 YOLO 추론 전에 즉시 /camera/color/compressed_raw 로 발행
  (박스 없음, 추론 시간과 무관하게 항상 빠름). 프론트엔드가 지연 없는
  영상이 필요하면 이 토픽을 구독.
- 박스+텍스트가 그려진 프레임은 기존처럼 /camera/color/compressed 로
  발행 (추론이 끝난 뒤라 약간의 지연 있음, 검출 정보를 보고 싶을 때 사용).
- 두 토픽 모두 QoS를 BEST_EFFORT + depth=1 로 설정해, 구독 측 처리가
  느려도 오래된 프레임이 큐에 쌓이지 않고 최신 프레임만 전달되도록 함
  (이게 "버퍼링으로 인한 지연 누적"의 주된 해결책).
"""

import json
import math
import re
import numpy as np

import rclpy
from rclpy.node import Node
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
    """숫자가 포함된 클래스면 조립 단계(level류)로 간주."""
    return re.search(r'(\d+)', str(class_name)) is not None


# ★ 버퍼링 방지용 QoS: 최신 프레임 하나만 유지, 느린 구독자 때문에 밀리지 않음
LOW_LATENCY_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class YoloDepthPublisher(Node):

    def __init__(self):
        super().__init__('yolo_depth_publisher')

        self.declare_parameter(
            'weights_path',
            '/home/ship_yard/smart-shipyard/edge/ros2_ws/src/ship_ugv_perception/weights/best.pt'
        )
        self.declare_parameter('detection_topic', '/event_detection/uvd')
        self.declare_parameter('confidence_threshold', 0.2)
        self.declare_parameter('debug_log', True)
        self.declare_parameter('fallback_confirm_frames', 3)
        self.declare_parameter('fallback_match_dist_px', 60.0)
        self.declare_parameter(
            'tracker_config',
            '/home/ship_yard/smart-shipyard/edge/ros2_ws/src/ship_ugv_perception/ship_ugv_perception/custom_tracker.yaml'
        )
        # ★ 영상 관련 (원본 즉시 발행 + 박스 그려진 버전 둘 다 유지)
        self.declare_parameter('raw_image_topic', '/camera/color/compressed_raw')
        self.declare_parameter('annotated_image_topic', '/camera/color/compressed')
        self.declare_parameter('raw_image_jpeg_quality', 70)

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

        self.model = YOLO(weights_path)
        self.get_logger().info(f"모델 클래스 목록: {self.model.names}")
        self.pub = self.create_publisher(String, topic, 10)

        # ★ 두 영상 토픽 모두 BEST_EFFORT + depth=1 QoS 적용
        self.raw_image_pub = self.create_publisher(
            CompressedImage, raw_image_topic, LOW_LATENCY_QOS)
        self.annotated_image_pub = self.create_publisher(
            CompressedImage, annotated_image_topic, LOW_LATENCY_QOS)

        self.reported_tids = set()
        self.reported_fallbacks = []
        self.fallback_candidates = []

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
            f"raw_topic={raw_image_topic}, annotated_topic={annotated_image_topic}"
        )

        self.timer = self.create_timer(0.1, self._process_frame)

    def _match_nearby(self, records, class_name, u, v):
        for r in records:
            if r['class'] != class_name:
                continue
            if math.hypot(u - r['u'], v - r['v']) < self.fallback_match_dist:
                return r
        return None

    def _encode_and_publish(self, publisher, image):
        ok, encoded = cv2.imencode(
            '.jpg', image,
            [cv2.IMWRITE_JPEG_QUALITY, self.raw_jpeg_quality]
        )
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

        if z_m is not None:
            label = f"{class_name} {conf:.2f} {z_m:.2f}m"
        else:
            label = f"{class_name} {conf:.2f}"

        text_pos = (p1[0], max(0, p1[1] - 8))
        cv2.putText(image, label, text_pos,
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    def _process_frame(self):
        frames = self.pipeline.wait_for_frames(100)
        if frames is None:
            return

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if color_frame is None or depth_frame is None:
            return

        color_image = frame_to_bgr_image(color_frame)
        depth_data = np.frombuffer(depth_frame.get_data(), dtype=np.uint16)
        depth_image = depth_data.reshape((depth_frame.get_height(), depth_frame.get_width()))

        # ★ 추론 전에 즉시 원본부터 발행 (지연 최소화, 항상 빠름)
        self._encode_and_publish(self.raw_image_pub, color_image)

        results = self.model.track(
            color_image,
            persist=True,
            verbose=False,
            tracker=self.tracker_config
        )[0]

        display_image = color_image.copy()

        num_boxes = 0 if results.boxes is None else len(results.boxes)
        if self.debug_log:
            self.get_logger().info(f"[디버그] 검출 개수: {num_boxes}")

        if results.boxes is None:
            self._encode_and_publish(self.annotated_image_pub, display_image)
            return

        for box in results.boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            class_name = self.model.names[cls]

            if conf < self.conf_threshold:
                if self.debug_log:
                    self.get_logger().info(
                        f"[디버그] conf 미달 스킵: class={class_name} conf={conf:.2f}")
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
                    if self.debug_log:
                        self.get_logger().info(
                            f"[디버그] tid={track_id} 부여됐으나 이미 fallback으로 "
                            f"발행된 위치({u},{v}) -> 중복 스킵")
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
                    self.fallback_candidates.append(
                        {'class': class_name, 'u': u, 'v': v, 'count': 1})
                    self._draw_box(display_image, x1, y1, x2, y2, class_name, conf, None)
                    if self.debug_log:
                        self.get_logger().info(
                            f"[디버그] fallback 후보 신규: class={class_name} conf={conf:.2f} ({u},{v})")
                    continue

                cand['count'] += 1
                cand['u'], cand['v'] = u, v
                if self.debug_log:
                    self.get_logger().info(
                        f"[디버그] fallback 카운트: class={class_name} conf={conf:.2f} "
                        f"count={cand['count']}/{self.fallback_confirm_frames}")

                if cand['count'] < self.fallback_confirm_frames:
                    self._draw_box(display_image, x1, y1, x2, y2, class_name, conf, None)
                    continue

                self.fallback_candidates.remove(cand)
                self.reported_fallbacks.append(
                    {'class': class_name, 'u': u, 'v': v})
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

            if len(valid_depths) == 0:
                if self.debug_log:
                    self.get_logger().info(f"[디버그] depth 무효로 발행 실패: {class_name}")
                self._draw_box(display_image, x1, y1, x2, y2, class_name, conf, None)
                continue

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
                'u': u,
                'v': v,
                'depth': z_m,
                'depth_xyz': [X, Y, z_m],
                'class_id': class_name,
                'confidence': conf,
            })
            self.pub.publish(msg)

        # ★ 박스가 그려진 최종 프레임 발행
        self._encode_and_publish(self.annotated_image_pub, display_image)

    def destroy_node(self):
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
