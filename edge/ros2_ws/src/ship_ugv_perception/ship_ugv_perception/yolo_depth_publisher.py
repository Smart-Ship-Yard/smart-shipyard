#!/usr/bin/env python3
"""
yolo_depth_publisher.py
------------------------
Astra+ 카메라로 Color+Depth를 받아 YOLO로 객체를 검출하고,
검출된 각 객체의 (u, v, depth)를 /event_detection/uvd 토픽에 발행한다.
change_point.py가 이 토픽을 구독해서 map 좌표로 변환한다.

[알려진 이슈 - 2026-07-07]
레고 미니피규어(2~3cm급) 소품은 Astra+ 구조광 depth 센서로 안정적인 depth를
얻기엔 너무 작아서, bbox 내부 depth 값 대부분이 무효(0)로 나오고 일부만
우연히 주변 배경(선반 등)의 depth가 섞여 들어가는 문제가 확인됨.
-> 소품을 5~8cm급(듀플로 등)으로 교체 필요.
-> 코드 레벨 보완: min_valid_ratio 게이팅 추가됨 (유효 픽셀 비율이 낮은
   검출은 폐기) — 소품 교체 후에도 반사면/구멍 대응용으로 계속 유효.

[2026-07-08 코드 리뷰 반영 수정 이력]
1. weights_path 기본값: 절대경로 하드코딩 -> ament_index로 share 경로 자동 탐색
2. frame_to_bgr_image: imdecode 실패(None) 방어 (color 포맷이 MJPG가 아닐 때 크래시 방지)
3. min_valid_ratio 게이팅 추가 (레고 depth 오염 문제의 직접 보완책)
4. depth/color 해상도 불일치 방어 체크 추가
5. conf 필터를 model() 추론 인자로 이동 (후처리 -> 내부 필터링)
6. destroy_node의 pipeline.stop() 이중 호출/미초기화 방어
7. main()에서 노드 초기화 실패 시 NameError로 원인이 가려지는 문제 방어
"""

import json
import os

import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

from pyorbbecsdk import Pipeline, Config, OBSensorType, OBAlignMode
from ultralytics import YOLO
import cv2


def frame_to_bgr_image(frame):
    """color 프레임 -> BGR numpy 배열. 디코드 실패 시 None 반환 (호출부에서 방어)."""
    data = np.frombuffer(frame.get_data(), dtype=np.uint8)
    # 주의: imdecode는 MJPG 등 압축 포맷 전제. RGB/UYVY 등 비압축 포맷이면
    # None이 반환되므로 호출부에서 반드시 None 체크할 것.
    return cv2.imdecode(data, cv2.IMREAD_COLOR)


class YoloDepthPublisher(Node):

    def __init__(self):
        super().__init__('yolo_depth_publisher')

        # ---- 파라미터 ----
        # weights 기본 경로: colcon 설치 위치(share/ship_ugv_perception/weights)를
        # ament_index로 자동 탐색. 계정명/설치 위치가 달라도 동작한다.
        # (setup.py data_files가 weights/*.pt 를 share로 복사해줌.
        #  best.pt가 미설치 상태면 아래 존재 검사에서 친절한 에러로 안내)
        default_weights = os.path.join(
            get_package_share_directory('ship_ugv_perception'),
            'weights', 'best.pt')
        self.declare_parameter('weights_path', default_weights)
        self.declare_parameter('detection_topic', '/event_detection/uvd')
        self.declare_parameter('confidence_threshold', 0.5)
        # ROI(bbox 중앙 절반 영역) 중 유효 depth 픽셀이 이 비율 미만이면
        # 그 검출의 depth를 신뢰하지 않고 폐기 (배경 오염 median 방지)
        self.declare_parameter('min_valid_ratio', 0.2)
        self.declare_parameter('debug_log', True)

        weights_path = self.get_parameter('weights_path').value
        topic = self.get_parameter('detection_topic').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.min_valid_ratio = self.get_parameter('min_valid_ratio').value
        self.debug_log = self.get_parameter('debug_log').value

        if not os.path.isfile(weights_path):
            raise FileNotFoundError(
                f"YOLO 가중치 파일이 없습니다: {weights_path}\n"
                "  - weights/best.pt를 소스 폴더에 넣고 colcon build를 다시 실행하거나,\n"
                "  - ros2 run ... --ros-args -p weights_path:=/실제/경로/best.pt 로 지정하세요."
            )

        self.model = YOLO(weights_path)
        self.pub = self.create_publisher(String, topic, 10)

        # ---- 카메라 파이프라인 초기화 ----
        # destroy_node에서의 이중 stop/미초기화 방어를 위해 None으로 먼저 선언
        self.pipeline = None
        pipeline = Pipeline()
        config = Config()

        profile_list = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
        color_profile = profile_list.get_default_video_stream_profile()
        config.enable_stream(color_profile)

        profile_list = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
        depth_profile = profile_list.get_default_video_stream_profile()
        config.enable_stream(depth_profile)

        config.set_align_mode(OBAlignMode.SW_MODE)
        pipeline.start(config)
        self.pipeline = pipeline  # start까지 성공한 뒤에만 보관

        self.get_logger().info(
            f"YOLO+Depth publisher 시작, weights={weights_path}, "
            f"conf>={self.conf_threshold}, min_valid_ratio={self.min_valid_ratio}"
        )

        # 타이머로 주기적 프레임 처리 (예: 10Hz)
        self.timer = self.create_timer(0.1, self._process_frame)

    # ------------------------------------------------------------------
    def _process_frame(self):
        frames = self.pipeline.wait_for_frames(100)
        if frames is None:
            return

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if color_frame is None or depth_frame is None:
            return

        color_image = frame_to_bgr_image(color_frame)
        if color_image is None:
            # color 스트림이 MJPG가 아닌 포맷(RGB/UYVY 등)으로 잡힌 경우.
            # 크래시 대신 경고만 남기고 이 프레임은 건너뜀.
            self.get_logger().warn(
                "color 프레임 디코드 실패 — 스트림 포맷이 MJPG가 아닐 수 있음 "
                "(OrbbecViewer에서 color 포맷 확인 필요)")
            return

        depth_data = np.frombuffer(depth_frame.get_data(), dtype=np.uint16)
        depth_image = depth_data.reshape(
            (depth_frame.get_height(), depth_frame.get_width()))

        # (u,v)는 color 픽셀 좌표 기준인데 depth 배열에 그대로 인덱싱하므로,
        # SW align이 기대대로 안 걸려 해상도가 다르면 좌표가 어긋난다 -> 방어
        if (depth_image.shape[0] != color_image.shape[0]
                or depth_image.shape[1] != color_image.shape[1]):
            self.get_logger().warn(
                f"depth{depth_image.shape} != color{color_image.shape[:2]} "
                "— SW align 설정 확인 필요, 이 프레임 건너뜀")
            return

        # conf 필터를 추론 인자로 전달 -> YOLO 내부에서 걸러짐 (후처리 감소)
        results = self.model(
            color_image, conf=self.conf_threshold, verbose=False)[0]

        for box in results.boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            class_name = self.model.names[cls]

            u, v = int((x1 + x2) / 2), int((y1 + y2) / 2)

            # bbox 중앙 절반(가로/세로 각 1/2, 면적 1/4) 영역을 ROI로
            box_w, box_h = int(x2 - x1), int(y2 - y1)
            y_min = max(0, v - box_h // 4)
            y_max = min(depth_image.shape[0], v + box_h // 4)
            x_min = max(0, u - box_w // 4)
            x_max = min(depth_image.shape[1], u + box_w // 4)
            depth_roi = depth_image[y_min:y_max, x_min:x_max]
            valid_depths = depth_roi[depth_roi > 0]

            if depth_roi.size == 0 or len(valid_depths) == 0:
                continue

            # --- 유효 픽셀 비율 게이팅 ---
            # bbox 안 depth 대부분이 무효(0)이고 소수만 배경 depth로 오염된 경우
            # (레고 실험에서 확인된 실패 모드) median이 엉뚱한 값이 되므로,
            # 유효 비율이 기준 미만이면 이 검출의 depth는 신뢰하지 않는다.
            valid_ratio = len(valid_depths) / depth_roi.size
            if valid_ratio < self.min_valid_ratio:
                if self.debug_log:
                    self.get_logger().warn(
                        f"[{class_name}] 유효 depth 비율 {valid_ratio:.0%} < "
                        f"{self.min_valid_ratio:.0%} → 폐기 "
                        f"(valid={len(valid_depths)}/{depth_roi.size})")
                continue

            z_m = float(np.median(valid_depths)) / 1000.0  # mm -> m

            if self.debug_log:
                self.get_logger().info(
                    f"[{class_name}] bbox=({x1:.0f},{y1:.0f})-({x2:.0f},{y2:.0f}) "
                    f"box_size=({box_w}x{box_h}) "
                    f"valid={len(valid_depths)}/{depth_roi.size} "
                    f"({valid_ratio:.0%}) "
                    f"min={valid_depths.min()}mm max={valid_depths.max()}mm "
                    f"z_m={z_m:.3f}m")

            msg = String()
            msg.data = json.dumps({
                'u': u,
                'v': v,
                'depth': z_m,
                'class_id': class_name,
                'confidence': conf,
            })
            self.pub.publish(msg)

    # ------------------------------------------------------------------
    def destroy_node(self):
        # __init__ 도중 실패했거나 이미 정지된 상태에서 재호출돼도 안전하게
        if self.pipeline is not None:
            try:
                self.pipeline.stop()
            except Exception as e:
                self.get_logger().warn(f"pipeline.stop() 중 예외 (무시): {e}")
            self.pipeline = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = YoloDepthPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 초기화 자체가 실패한 경우 node가 None이므로 가드
        # (가드 없으면 NameError가 원래 에러 메시지를 가림)
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
