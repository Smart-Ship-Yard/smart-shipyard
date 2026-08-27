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

[2026-08-21 추가 1] 클래스별 confidence 임계값
  전역 confidence_threshold 하나로 모든 클래스를 걸렀는데, 클래스마다
  적정 임계값이 다르다. 예를 들어 no_helmet 은 놓치면 안 되니 낮게, 배경
  오탐이 잦은 fire/helmet 은 높게 두고 싶다. class_confidence_overrides
  파라미터("클래스명:값" 문자열 배열)로 클래스별로 덮어쓸 수 있게 함.
  지정하지 않은 클래스는 기존처럼 confidence_threshold 를 쓴다.

[2026-08-21 추가 2] person crop 2단계 helmet/no_helmet 재검출
  화면 전체 기준으로는 안전모가 너무 작아 YOLO 내부 특징맵에서 정보가
  손실된다(작은 객체 탐지의 고질적 취약점). person 이 검출되면 그 영역만
  잘라 비율을 유지한 채 확대한 뒤 helmet/no_helmet 만 재검출한다.
  cv2.resize 는 단순 보간이라 원본에 없는 디테일을 만들어내지 않는다
  (슈퍼레졸루션과 다름). 화질을 올리는 게 아니라 "특징맵 상의 공간 여유"를
  늘려주는 것이다.
  개별 박스 처리(1차 필터 + depth 계산 + 발행)를 _evaluate_box() 로 공통화해
  메인 검출과 crop 검출이 같은 로직을 쓰게 했다. crop 좌표는 원본 프레임 좌표로 환산해서 넘긴다.

  ⚠️ CPU 주의: person 한 명당 추론이 한 번씩 더 돈다. 2026-08-18 에 CPU 포화로
     Nav2 lifecycle 전환이 타임아웃되어 브링업이 통째로 실패한 적이 있으므로,
     Nav2 와 동시에 돌릴 때는 반드시 부하를 실측할 것. 문제가 되면
     person_crop_enabled 를 끄거나(런타임에 ros2 param set 으로 즉시 가능)
     person_crop_max_count / person_crop_target_size 를 줄인다.
"""

import array
import base64
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


# ★ 클래스별 confidence 임계값 — 여기 값만 고치면 바로 적용됩니다.
#   여기 없는 클래스는 아래 declare_parameter('confidence_threshold', ...) 의
#   전역값(기본 0.2)을 그대로 씁니다.
#   예: no_helmet/fallen_person 은 놓치면 안 되니 낮게, fire/helmet 처럼
#   배경 오탐이 잦은 클래스는 높게 두는 식으로 조정하세요.
# 이벤트 스냅샷을 남길 클래스 (위험 이벤트만). 프론트가 핑을 눌렀을 때
# "그때 무슨 일이 있었나" 를 보여주는 사진 한 장이다.
SNAPSHOT_CLASSES = ('fire', 'fallen_person', 'no_helmet')

CLASS_CONFIDENCE = {
    'no_helmet': 0.15,
    'fallen_person': 0.15,
    'fire': 0.45,
    'helmet': 0.35,
    # ★ 조립 단계(level1~5) 임계값 신설 (2026-08-27).
    #   여태 여기 없어서 전역값(0.1)을 그대로 썼고, 그 결과
    #   conf 0.02~0.08 짜리 쓰레기 검출이 그대로 발행돼 대시보드 공정률이
    #   요동쳤다. 실측: 깨끗한 정면 프레임에서 level3 0.434 가 1위인데
    #   level1 0.036 / 0.014 가 같이 딸려 나왔다.
    #   진짜 판정은 0.43~0.83 에서 나오므로 0.35 로 가른다.
    'level1': 0.35, 'level2': 0.35, 'level3': 0.35,
    'level4': 0.35, 'level5': 0.35,
}


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
        # ★ 클래스별 confidence 임계값
        #   기본값은 파일 위쪽의 CLASS_CONFIDENCE 딕셔너리를 그대로 씀 —
        #   값을 바꾸고 싶으면 재빌드 없이도 이 파라미터로 임시 조정 가능:
        #     ros2 run ... --ros-args \
        #       -p "class_confidence_overrides:=['no_helmet:0.15','fire:0.45']"
        #   실행 중에도 ros2 param set 으로 즉시 반영됨. 여기서 지정 안 한
        #   클래스는 CLASS_CONFIDENCE -> 없으면 confidence_threshold 순으로 적용.
        #   평소엔 이 파라미터를 건드릴 필요 없이, 그냥 CLASS_CONFIDENCE
        #   딕셔너리 값만 고쳐서 재빌드하면 됨.
        self.declare_parameter('class_confidence_overrides', [''])
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
        # ★ person crop 2단계 재검출 (2026-08-21 신설). 위 docstring 의 CPU 주의 참고.
        self.declare_parameter('person_crop_enabled', True)
        self.declare_parameter('person_crop_target_size', 640)   # 확대 후 긴 변 크기
        self.declare_parameter('person_crop_min_size_px', 20)    # 이보다 작은 person 은 건너뜀
        self.declare_parameter('person_crop_max_count', 2)       # 한 프레임에 처리할 최대 인원

        weights_path = self.get_parameter('weights_path').value
        topic = self.get_parameter('detection_topic').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        # ★ CLASS_CONFIDENCE(코드 상단 딕셔너리)를 기본값으로 삼고, 파라미터로
        #   지정한 게 있으면 그 클래스만 덮어씀 (재빌드 없는 임시 실험용).
        self.class_conf = dict(CLASS_CONFIDENCE)
        self.class_conf.update(self._parse_class_conf(
            self.get_parameter('class_confidence_overrides').value))
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
        self.person_crop_enabled = self.get_parameter('person_crop_enabled').value
        self.person_crop_target_size = int(self.get_parameter('person_crop_target_size').value)
        self.person_crop_min_size = int(self.get_parameter('person_crop_min_size_px').value)
        self.person_crop_max_count = int(self.get_parameter('person_crop_max_count').value)

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
        if self.class_conf:
            self.get_logger().info(f"클래스별 confidence 임계값: {self.class_conf} "
                                   f"(그 외 전역값 {self.conf_threshold})")

        # person crop 재검출을 helmet/no_helmet 으로만 제한한다(속도).
        self._helmet_class_ids = [
            idx for idx, name in self.model.names.items()
            if name in ('helmet', 'no_helmet')
        ]
        if self.person_crop_enabled and not self._helmet_class_ids:
            self.get_logger().warn(
                "person_crop_enabled=True 이지만 모델에 helmet/no_helmet 이 없음 - 비활성화")
            self.person_crop_enabled = False

        # 1차 통과 문턱은 '가장 낮은 클래스 임계값'. 클래스별 판정은
        # _evaluate_box 가 _conf_for() 로 다시 한다.
        self._min_conf = min([self.conf_threshold] + list(self.class_conf.values()))

        # ★ 이벤트 스냅샷 (2026-08-27).
        #   프론트에서 핑을 누르면 "그 이벤트가 감지되던 순간의 사진" 을 본다.
        #   ★★ 인코딩은 **중복 제거를 통과한 새 이벤트에 대해서만** 한다.
        #   매 프레임 crop->JPEG 하면 초당 수십 번이 되어 CPU 그림이 완전히
        #   달라진다(실측: crop 인코딩 1.28 ms + base64 0.12 ms). 그래서
        #   여기서는 **자르기만 해서 들고 있고**(numpy 슬라이스 복사, 0.05 ms
        #   수준), change_point_detector 가 새 이벤트를 확정해 발행했을 때
        #   그 시점에 딱 한 번 인코딩한다.
        # ★ 배가 화면에서 잘리면 조립 단계를 오판한다 (2026-08-27 실측).
        #   같은 배를 잘라가며 재보니:
        #       전체        level3 0.434
        #       왼쪽 40% 잘림 level3 0.629 + level4 0.345
        #       왼쪽 50% 잘림 level4 0.561   <- 뒤집힘
        #   그래서 bbox 가 화면 테두리에 닿으면(= 잘린 것) 조립 단계 판정에서
        #   제외한다.
        #
        #   ★ 실물에서도 True 로 둔다 (2026-08-27 판단).
        #   실물 배는 순찰 거리에서 대개 화면에 다 안 들어온다. 그래서 처음엔
        #   "실물이면 꺼야 한다" 고 봤는데, 다시 따져보니 그 반대다:
        #     - 조립 단계는 **위험 이벤트가 아니다.** 빨리 갱신될 필요가 없고,
        #       한 바퀴에 한 번만 맞게 갱신돼도 충분하다.
        #     - 순찰 궤도 어딘가에는 배가 온전히 들어오는 지점이 생긴다.
        #       그 지점의 판정만 받으면 된다.
        #     - 잘린 화면으로 자주 갱신하는 것보다, 드물게 맞는 편이 낫다.
        #   즉 **갱신 빈도를 내주고 정확도를 산다.** 위험 이벤트였다면
        #   반대로 골랐겠지만 조립 단계는 그래도 된다.
        #
        #   ⚠️ 대신 위험이 하나 생긴다: 배가 **한 번도** 온전히 안 잡히면
        #   공정률이 조용히 멈춘다. 그래서 아래 _warn_if_level_starved 로
        #   시끄럽게 알린다. 조용한 실패가 제일 나쁘다.
        self.declare_parameter('reject_level_touching_edge', True)
        self.declare_parameter('edge_margin_px', 3)
        self.reject_level_edge = bool(
            self.get_parameter('reject_level_touching_edge').value)
        self.edge_margin = int(self.get_parameter('edge_margin_px').value)
        self.declare_parameter('level_starved_warn_s', 120.0)
        self.level_starved_warn = float(
            self.get_parameter('level_starved_warn_s').value)
        self._level_last_pass = time.time()   # 마지막으로 '온전한 배'를 본 시각
        self._level_edge_rejects = 0

        self.declare_parameter('snapshot_enabled', True)
        self.declare_parameter('snapshot_jpeg_quality', 70)
        self.declare_parameter('snapshot_margin_px', 40)
        self.snapshot_enabled = bool(self.get_parameter('snapshot_enabled').value)
        self.snapshot_quality = int(self.get_parameter('snapshot_jpeg_quality').value)
        self.snapshot_margin = int(self.get_parameter('snapshot_margin_px').value)
        self._last_crop = {}          # class_name -> BGR crop (인코딩 전)
        self._crop_lock = threading.Lock()

        if self.snapshot_enabled:
            self.snapshot_pub = self.create_publisher(
                String, '/event_detection/snapshot', 10)
            self.create_subscription(
                String, '/event_detection/map_point', self._snapshot_cb, 10)

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

        # ★ 뎁스-컬러 정렬은 소프트웨어 모드로 둔다.
        #   HW_MODE(카메라 칩이 정렬)로 바꿔 CPU 를 아낄 수 있는지 실측해
        #   봤으나 차이가 없었다(2026-08-27: 1.77코어 -> 1.77코어). 이 카메라
        #   에서는 HW_MODE 를 켜도 결국 같은 경로를 타는 것으로 보인다.
        #   검증된 이득이 없는 변경은 되돌린다 — 정렬 방식이 바뀌면 뎁스
        #   품질도 미묘하게 달라질 수 있어 굳이 위험을 질 이유가 없다.
        #   (실제 CPU 범인은 영상 발행의 rclpy 원소 검사였다. _encode_and_publish
        #    주석 참고)
        #   DISABLE 은 선택지가 아니다 — (u,v) 로 뎁스를 못 찾아 좌표 변환이
        #   통째로 깨진다.
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
            f"person_crop={self.person_crop_enabled}"
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

    # ------------------------------------------------------------------
    @staticmethod
    def _parse_class_conf(raw_list):
        """['no_helmet:0.15', 'fire:0.45'] -> {'no_helmet': 0.15, 'fire': 0.45}

        빈 문자열/형식 오류 항목은 조용히 건너뛴다(파라미터 하나 잘못 적었다고
        노드가 죽으면 안 되므로).
        """
        out = {}
        for item in (raw_list or []):
            s = str(item).strip()
            if not s or ':' not in s:
                continue
            name, _, value = s.partition(':')
            name = name.strip()
            try:
                out[name] = float(value)
            except ValueError:
                continue
        return out

    def _conf_for(self, class_name):
        """이 클래스에 적용할 confidence 임계값. 지정 없으면 전역값."""
        return self.class_conf.get(class_name, self.conf_threshold)

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
            elif p.name == 'class_confidence_overrides':
                self.class_conf = dict(CLASS_CONFIDENCE)
                self.class_conf.update(self._parse_class_conf(p.value))
                self.get_logger().info(
                    f'class_confidence_overrides -> {self.class_conf} (즉시 반영됨)')
            elif p.name == 'person_crop_enabled':
                self.person_crop_enabled = bool(p.value) and bool(self._helmet_class_ids)
                self.get_logger().info(
                    f'person_crop_enabled -> {self.person_crop_enabled} (즉시 반영됨)')
        return SetParametersResult(successful=True)

    def _encode_and_publish(self, publisher, image, quality=None):
        q = quality if quality is not None else self.raw_jpeg_quality
        ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, q])
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = 'jpeg'
        # ★ array.array 로 넣는다 — bytes 를 넣으면 안 된다 (2026-08-27).
        #   rclpy 가 만든 uint8[] setter 는 array.array('B') 면 그대로 받고
        #   끝내지만(O(1)), 그 외에는 원소를 전부 훑어 검사한다:
        #       all(isinstance(v, int) for v in value)
        #       all(0 <= val < 256 for val in value)
        #   JPEG 한 장이 ~100KB 니 프레임당 20만 번, 30fps 면 초당 600만 번의
        #   파이썬 루프다. py-spy 실측에서 이 한 줄이 프로세스 CPU 의 70% 를
        #   먹고 있었다(추론보다 3.5배). CPU 포화로 Nav2 가 밀려 로봇이 멈췄다.
        msg.data = array.array('B', encoded.tobytes())
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
                    # 위 _encode_and_publish 의 주석 참고 — bytes 를 넣으면 안 된다
                    msg.data = array.array('B', raw.tobytes())
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
                    # ★ .copy() 필수 (2026-08-27).
                    #   raw 는 np.frombuffer 로 만든 **SDK 프레임 버퍼의 뷰**이고
                    #   복사본이 아니다. 캡처 스레드가 다음 프레임으로 넘어가면
                    #   SDK 가 그 버퍼를 재사용하므로, 추론 스레드가 250ms 뒤에
                    #   읽을 때는 이미 다른 내용일 수 있다.
                    #   영상 발행의 rclpy 원소 검사를 없애(2400배) 캡처 루프가
                    #   빨라지자 이 경합이 바로 드러났다 — 카메라 화면은 멀쩡한데
                    #   (발행 경로는 tobytes() 로 복사본을 쓴다) 추론만 검출 0건이
                    #   되어 불을 봐도 로봇이 안 멈췄다.
                    self._latest_raw = raw.copy() if is_jpeg else None
                    self._latest_color = color_image
                    self._latest_depth = depth_image.copy()   # 위와 같은 이유(뷰 -> 복사)
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
    # ★ 박스 하나를 처리하는 공통 로직 (1차 필터 -> depth 계산 -> 발행).
    #   메인 검출과 person crop 재검출이 함께 쓴다. track_id 인자는 예전
    #   트래커 시절의 잔재로 지금은 항상 None 이 들어온다(위 predict 주석
    #   참고) — 즉 모든 검출이 연속 N프레임 확인 경로를 탄다.
    def _evaluate_box(self, class_name, conf, x1, y1, x2, y2, track_id,
                      depth_image, display_image, draw_box=True):
        if conf < self._conf_for(class_name):
            return

        # ★ 잘린 배로 조립 단계를 판정하지 않는다 (위 reject_level_touching_edge
        #   주석 참고). 위험 이벤트(fire 등)는 잘려도 "거기 있다" 는 사실이
        #   중요하므로 이 규칙을 적용하지 않는다 — 조립 단계만 대상이다.
        if self.reject_level_edge and class_name.startswith('level'):
            h, w = depth_image.shape[:2]
            m = self.edge_margin
            if x1 <= m or y1 <= m or x2 >= w - m or y2 >= h - m:
                self._level_edge_rejects += 1
                self.get_logger().info(
                    f"[조립단계] {class_name} conf={conf:.2f} — 배가 화면에서 "
                    "잘려 판정 제외", throttle_duration_sec=10.0)
                self._warn_if_level_starved()
                return
            # 온전히 들어온 배를 봤다 — 굶주림 시계를 되돌린다
            self._level_last_pass = time.time()
            self._level_edge_rejects = 0

        u, v = int((x1 + x2) / 2), int((y1 + y2) / 2)

        def _maybe_draw(z_m=None):
            if draw_box:
                self._draw_box(display_image, x1, y1, x2, y2, class_name, conf, z_m)

        publish_ok = False
        key_info = ""

        if is_level_class(class_name):
            publish_ok = True
            key_info = "level(항상발행)"

        else:
            # ★ 트랙 ID가 없는 검출: 노이즈 필터링 목적으로만 연속
            #   fallback_confirm_frames 프레임 동안 비슷한 화면 위치에
            #   잡혀야 발행한다. (위치 중복 제거는 change_point_detector 몫)
            cand = self._match_nearby(self.fallback_candidates, class_name, u, v)
            if cand is None:
                self.fallback_candidates.append({'class': class_name, 'u': u, 'v': v, 'count': 1})
                _maybe_draw()
                return
            cand['count'] += 1
            cand['u'], cand['v'] = u, v
            if cand['count'] < self.fallback_confirm_frames:
                _maybe_draw()
                return
            self.fallback_candidates.remove(cand)
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
            _maybe_draw()
            return

        z_m = float(np.median(valid_depths)) / 1000.0
        X = (u - self.cx) * z_m / self.fx
        Y = (v - self.cy) * z_m / self.fy

        _maybe_draw(z_m)

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

    # ------------------------------------------------------------------
    # ★ person 영역들을 잘라 확대한 뒤 helmet/no_helmet 만 배치로 재검출.
    #   호출당 추론 1회(배치)로 처리해 인원수만큼 호출이 늘지 않게 한다.
    #
    #   위치(u,v)와 거리(depth)는 helmet 자신의 작은 bbox 가 아니라 **그
    #   helmet 을 찾아낸 person 의 bbox**를 그대로 쓴다. 위험 이벤트의 관심사는
    #   "안전모 픽셀이 정확히 어디 있는가"가 아니라 "안전모가 없는 그 사람이
    #   어디 있는가"이고, person bbox 는 훨씬 커서 depth 유효 픽셀 비율이
    #   안정적으로 min_valid_ratio 를 넘는다(반대로 작은 helmet bbox 는 depth
    #   가 자주 무효 처리되어 검출이 통째로 폐기되곤 했다). 화면에 그려지는
    #   박스만 실제 helmet 위치(정확한 시각화용)를 쓰고, 발행되는 이벤트의
    #   좌표는 person 기준이다.
    def _detect_helmet_in_person_crops(self, color_image, person_boxes,
                                       depth_image, display_image):
        crops = []
        metas = []   # (px1, py1, px2, py2, scale) — 원본 person bbox 그대로 들고 다님

        for (px1, py1, px2, py2) in person_boxes[:self.person_crop_max_count]:
            px1 = max(0, int(px1))
            py1 = max(0, int(py1))
            px2 = min(color_image.shape[1], int(px2))
            py2 = min(color_image.shape[0], int(py2))
            cw, ch = px2 - px1, py2 - py1
            if cw < self.person_crop_min_size or ch < self.person_crop_min_size:
                continue
            crop = color_image[py1:py2, px1:px2]
            if crop.size == 0:
                continue
            # 비율 유지 확대 (정사각형으로 늘리면 사람이 찌그러져 특징이 왜곡됨)
            scale = self.person_crop_target_size / max(cw, ch)
            nw, nh = max(1, int(cw * scale)), max(1, int(ch * scale))
            crops.append(cv2.resize(crop, (nw, nh)))
            metas.append((px1, py1, px2, py2, scale))

        if not crops:
            return

        try:
            results_list = self.model(
                crops, verbose=False, classes=self._helmet_class_ids)
        except Exception as e:
            self.get_logger().warn(f"person crop 재검출 실패(무시하고 계속): {e}")
            return

        for res, (px1, py1, px2, py2, scale) in zip(results_list, metas):
            if res.boxes is None:
                continue
            for hbox in res.boxes:
                hx1, hy1, hx2, hy2 = hbox.xyxy[0].cpu().numpy()
                hconf = float(hbox.conf[0])
                hname = self.model.names[int(hbox.cls[0])]

                # 시각화용: helmet 실제 위치를 원본 좌표로 환산해서 화면에는
                # 정확한 자리에 박스를 그린다 (사람 전체 박스를 그리면 어디에
                # helmet/no_helmet 이 있는지 눈으로 확인하기 어려워지므로).
                ox1 = px1 + hx1 / scale
                oy1 = py1 + hy1 / scale
                ox2 = px1 + hx2 / scale
                oy2 = py1 + hy2 / scale
                self._draw_box(display_image, ox1, oy1, ox2, oy2, hname, hconf, None)

                if self.debug_log:
                    self.get_logger().info(
                        f"[person crop] {hname} conf={hconf:.2f} "
                        f"-> person bbox 기준 좌표/거리로 발행",
                        throttle_duration_sec=5.0)

                # 발행 좌표/거리는 person bbox 기준 (위 docstring 참고).
                # draw_box=False: 정확한 helmet 위치 박스는 위에서 이미 그렸으므로,
                # 여기서 person 크기 박스를 또 그리면 화면에 두 개가 겹쳐 지저분해짐.
                self._evaluate_box(hname, hconf, px1, py1, px2, py2,
                                   None, depth_image, display_image, draw_box=False)

    # ------------------------------------------------------------------
    def _warn_if_level_starved(self):
        """배가 계속 잘려서 조립 단계가 한 번도 갱신되지 못하는 상황을 알린다.

        reject_level_touching_edge 는 "온전히 보일 때만 판정한다" 는 정책이라,
        순찰 궤도가 배에 너무 붙었거나 배가 커지면 **판정이 영영 안 나올 수
        있다.** 그런데 그건 에러가 아니라 그냥 조용함이라 눈치채기 어렵다.
        조용한 실패가 제일 나쁘므로 여기서 시끄럽게 만든다.
        """
        idle = time.time() - self._level_last_pass
        if idle < self.level_starved_warn:
            return
        self.get_logger().warn(
            f"[조립단계] {idle/60:.0f}분째 배가 화면에 온전히 안 담긴다 "
            f"(잘려서 제외한 검출 {self._level_edge_rejects}건). "
            "공정률이 갱신되지 않고 있다 — 순찰 반경을 넓히거나, "
            "reject_level_touching_edge 를 False 로 두고 부분 뷰 학습 모델을 쓸 것",
            throttle_duration_sec=60.0)

    # ------------------------------------------------------------------
    def _stash_crop(self, class_name, color_image, x1, y1, x2, y2):
        """검출 영역을 잘라서 들고만 있는다. 인코딩은 하지 않는다."""
        h, w = color_image.shape[:2]
        m = self.snapshot_margin
        cx1 = max(0, int(x1) - m)
        cy1 = max(0, int(y1) - m)
        cx2 = min(w, int(x2) + m)
        cy2 = min(h, int(y2) + m)
        if cx2 - cx1 < 8 or cy2 - cy1 < 8:
            return
        crop = color_image[cy1:cy2, cx1:cx2].copy()
        with self._crop_lock:
            self._last_crop[class_name] = crop

    def _snapshot_cb(self, msg):
        """change_point_detector 가 **새 이벤트**를 확정했을 때만 불린다.

        중복 제거를 통과한 것만 여기 오므로, 인코딩은 이벤트당 한 번뿐이다.
        한 바퀴에 몇 건 수준이라 CPU 부담은 사실상 없다.
        """
        try:
            det = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        cls = str(det.get('class_id', ''))
        if cls not in SNAPSHOT_CLASSES:
            return
        with self._crop_lock:
            crop = self._last_crop.get(cls)
        if crop is None:
            self.get_logger().warn(
                f"[스냅샷] {cls} 이벤트인데 들고 있는 crop 이 없다 - 건너뜀")
            return
        try:
            ok, enc = cv2.imencode(
                '.jpg', crop, [cv2.IMWRITE_JPEG_QUALITY, self.snapshot_quality])
            if not ok:
                return
            b64 = base64.b64encode(enc.tobytes()).decode('ascii')
        except Exception as e:
            self.get_logger().warn(f"[스냅샷] 인코딩 실패(무시): {e}")
            return
        out = String()
        out.data = json.dumps({
            'event_id': det.get('event_id'),
            'class_id': cls,
            'image_b64': b64,
        })
        self.snapshot_pub.publish(out)
        self.get_logger().info(
            f"[스냅샷] {cls} event_id={det.get('event_id')} "
            f"{crop.shape[1]}x{crop.shape[0]} -> {len(b64)/1024:.0f} KB(base64)")

    # ------------------------------------------------------------------
    # YOLO 처리 (캡처 스레드가 채워둔 최신 프레임을 가져다 씀, 카메라 속도와 무관)
    def _process_frame(self):
        color_image, depth_image = self._get_latest_frame()
        if color_image is None:
            return

        # ★ track() 이 아니라 predict() 를 쓴다 (2026-08-27 실측 사고).
        #   track(persist=True) 는 트래커 상태를 계속 누적하는데, 그 상태가
        #   망가지면 **검출이 통째로 사라진다.** 실측: 같은 프레임에 대해
        #       predict()  -> level3=0.79, fire=0.89
        #       실행 중 노드 -> 검출 개수 0 (주석 영상에 박스가 하나도 없음)
        #   YOLO 를 재시작하면 잠깐 정상으로 돌아왔다가 몇 분 뒤 다시 0이
        #   되는 것도 "누적된 상태" 라는 설명과 맞는다.
        #
        #   그리고 우리는 이제 track_id 가 필요 없다. 예전에는 reported_tids
        #   로 "이 트랙은 이미 보고했다" 를 기억했지만, 그 화면좌표 기반
        #   중복 제거는 걷어냈다(커밋 057d5f3). 지금 위치 기반 중복 제거는
        #   change_point_detector 가 map 좌표로 한다.
        #   -> 트래커를 빼면 불안정 요인과 칼만 필터 CPU 가 함께 사라지고,
        #      모든 검출이 아래 fallback 경로(연속 N프레임 확인)를 타므로
        #      노이즈 필터는 오히려 일관돼진다.
        #   ★ classes=None / conf 을 **매번 명시**해야 한다 (2026-08-27 실측 사고).
        #   ultralytics 는 Model 하나가 predictor 하나를 공유하고, predict()
        #   kwargs 를 그 predictor.args 에 **누적 병합**한다. 그래서 아래
        #   _run_person_crop 이 self.model(crops, classes=[helmet,no_helmet])
        #   를 한 번 부르면 그 classes 필터가 메인 추론에 눌러붙어, 그 뒤로는
        #   fire/level 이 통째로 걸러진다. 재현:
        #       predict()                 -> ['fire','fire']
        #       model(x, classes=[2,8])   -> (person crop 경로)
        #       predict()                 -> []        영구히 0건
        #   화면에 사람이 한 번만 들어와도 그때부터 불을 영영 못 봤다.
        #   conf 도 같은 이유로 명시한다 — 안 주면 ultralytics 기본값 0.25 라
        #   confidence_threshold:=0.1 과 no_helmet/fallen_person 0.15 가
        #   조용히 무시됐다(파라미터가 먹은 척만 하는 상태).
        results = self.model.predict(color_image, verbose=False,
                                     classes=None, conf=self._min_conf)[0]

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

        # ---- 1단계: 프레임 전체 검출 ----
        person_boxes = []
        for box in results.boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            class_name = self.model.names[cls]
            # track_id 는 더 이상 쓰지 않는다(위 predict 주석 참고).
            # None 을 넘겨 모든 검출이 연속 프레임 확인 경로를 타게 한다.
            self._evaluate_box(class_name, conf, x1, y1, x2, y2, None,
                               depth_image, display_image)

            # ★ 자르기만 해두고 인코딩은 안 한다 (위 snapshot 주석 참고).
            #   display_image 가 아니라 color_image 를 쓴다 — 박스가 그려지기
            #   전의 깨끗한 화면이어야 사진으로서 쓸모가 있다.
            if (self.snapshot_enabled and class_name in SNAPSHOT_CLASSES
                    and conf >= self._conf_for(class_name)):
                self._stash_crop(class_name, color_image, x1, y1, x2, y2)

            # person 은 confidence 미달이면 crop 대상에서도 제외
            if (self.person_crop_enabled and class_name == 'person'
                    and conf >= self._conf_for(class_name)):
                person_boxes.append((x1, y1, x2, y2))

        # ---- 2단계: person 영역 확대 후 helmet/no_helmet 재검출 ----
        if person_boxes:
            self._detect_helmet_in_person_crops(
                color_image, person_boxes, depth_image, display_image)

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