#!/usr/bin/env python3
"""
ship_survey_node.py
--------------------
배(모형 선박 블록)의 중심좌표·방향·크기를 뎁스카메라로 측량해서
/ship_survey/pose 로 1회 발행하고, 결과를 파일로도 남기는 노드.

[왜 라이다가 아니라 카메라인가]
RPLidar는 바닥에서 약 21cm 높이의 수평면 한 장만 스캔하는데, 모형 배는 그
높이에 선체가 걸리지 않아 맵에 아예 안 찍힌다. 반면 Astra+ RGB 렌즈는
base_link 기준 z=0.110m (change_point.py의 camera_offset_z)로 라이다보다
낮게 달려 있어서, 라이다가 못 보는 낮은 배를 오히려 정면으로 본다.
그래서 라이다 대신 YOLO 검출 + depth로 측량한다.

[측량 원리]
yolo_depth_publisher가 발행하는 검출 1건 = 배 표면의 3D 점 1개다.
(bbox 중앙 1/4 영역의 depth median → 카메라 좌표계 depth_xyz)
이 점은 "카메라를 향한 표면의 한 점"이지 배 중심이 아니므로, 한 프레임으로는
중심을 알 수 없다. 로봇이 매핑 랩을 돌며 여러 각도에서 모으면 점들이 배
둘레에 흩뿌려지고, 여기에 cv2.minAreaRect로 최소외접 회전사각형을 피팅하면
중심·크기·회전각이 한 번에 나온다.

단순 평균(centroid)을 안 쓰는 이유: 한 면 앞에 오래 서 있으면 그쪽 점이 많이
쌓여 평균이 끌려간다. minAreaRect는 볼록껍질에만 의존해서 같은 점이 몇 번
찍히든 결과가 안 변하므로 이 편향이 원리적으로 없다.

[측량 시점 - interface.md ④]
세션 시작(매핑 랩)에 1회 + 조립 단계가 바뀔 때마다 1회. 조립하면서 배가 밀리거나
돌아갈 수 있고, 단계가 올라가면 배의 형태·크기 자체가 달라지기 때문.
재측량은 websocket_client가 발행하는 /block_level/confirmed로 트리거된다.
"매 프레임 갱신"은 하지 않는다 - Nav2 코스트맵과 경로가 매 순간 무효화되어
로봇이 진동한다.

[입력]
  /event_detection/uvd (std_msgs/String, JSON) - yolo_depth_publisher 원본 출력.
  ★ /event_detection/map_point(change_point.py 출력)를 쓰면 안 된다. 거기엔
    반경 1m 중복제거 필터가 있어서(change_point.py의 _find_matching_event)
    같은 배를 반복 관측한 게 전부 걸러지고 점이 거의 안 쌓인다.

  /block_level/confirmed (std_msgs/Int32, latched) - 재측량 트리거.
  ★ 같은 판정을 여기서 또 하지 않는 이유: 프레임마다 흔들리는 YOLO를 3초
    안정화로 거르는 로직이 websocket_client._handle_block_level에 이미 있다.
    두 벌이 되면 서버가 아는 단계와 측량이 아는 단계가 어긋나는 순간이 생긴다.

[출력 ①] /ship_survey/pose (std_msgs/String, JSON) - docs/interface.md ④ 스펙
  {"event_type": "ship_pose", "block_id": "B1", "map_xy": [x, y], "yaw": rad}
  websocket_client.py가 이 토픽을 이미 구독 중이라 발행만 하면 서버까지 자동으로 간다.
  QoS는 TRANSIENT_LOCAL(latched) - 1회만 발행해도 나중에 붙는 구독자가 마지막
  값을 받는다. ROS2 기본값은 VOLATILE이라 명시하지 않으면 값이 사라진다.

[출력 ②] /tmp/ship_survey_results/ship_pose_<회차>.json
  위 내용 + size_xy(배 크기). Nav2 쪽 finalize_map.py가 읽어서 keepout 마스크를
  만든다. 그 스크립트는 ROS 노드가 아니라 토픽을 못 받으므로 파일이 필요하다.
  uwb_map_calibration / slam_map_alignment의 결과 저장 방식과 동일하게 맞췄다.
"""

import json
import math
import os
import time

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String, Int32
from std_srvs.srv import Trigger
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401  (PointStamped 변환 등록용)


def wrap_angle(angle: float) -> float:
    """각도를 (-pi, pi] 범위로 정규화."""
    return math.atan2(math.sin(angle), math.cos(angle))


def fit_rect(points_xy, yaw_hint: float = 0.0, outlier_radius_m: float = 2.0):
    """
    map 좌표 2D 점 집합에 최소외접 회전사각형을 피팅해 배의 중심·방향·크기를 구한다.

    ROS에 의존하지 않는 순수 함수로 분리했다. 측량 로직의 핵심이자 유일하게
    틀리기 쉬운 부분이라, 노드를 안 띄우고도 테스트할 수 있어야 하기 때문.
    (test/test_ship_survey.py 참고)

    반환: (center_x, center_y, yaw_rad, long_side_m, short_side_m, num_used)
          점이 모자라면 None.
    """
    pts = np.asarray(points_xy, dtype=np.float64).reshape(-1, 2)
    if len(pts) < 3:
        return None

    # --- 이상치 제거 ---
    # minAreaRect는 볼록껍질 기반이라 "가장 바깥 점"이 결과를 전부 지배한다.
    # YOLO가 배경을 잘못 물어서 점 하나가 10m 밖에 찍히면 사각형이 통째로
    # 터지는데, 그 결과가 Nav2 keepout 마스크로 들어가므로 조용히 넘기면 안 된다.
    # 중앙값은 이상치에 강하므로 중앙값 기준 반경으로 자른다.
    center_guess = np.median(pts, axis=0)
    dist = np.hypot(pts[:, 0] - center_guess[0], pts[:, 1] - center_guess[1])
    pts = pts[dist <= outlier_radius_m]
    if len(pts) < 3:
        return None

    rect = cv2.minAreaRect(pts.astype(np.float32))
    (cx, cy), _wh, _angle_deg = rect

    # --- 최장변으로 yaw 계산 ---
    # cv2.minAreaRect가 돌려주는 angle은 OpenCV 버전마다 규약이 달라서
    # (4.5 이전 [-90,0), 이후 (0,90]) 그대로 쓰면 버전 바뀔 때 조용히 틀어진다.
    # boxPoints로 꼭짓점을 받아 최장변 각도를 직접 재면 버전과 무관하게 정확하다.
    box = cv2.boxPoints(rect).astype(np.float64)   # (4,2), 인접 순서 보장됨
    edges = box[[1, 2, 3, 0]] - box                # 각 변 벡터
    lengths = np.hypot(edges[:, 0], edges[:, 1])
    long_i = int(np.argmax(lengths))
    yaw = math.atan2(edges[long_i, 1], edges[long_i, 0])

    # --- 180도 모호성 해소 ---
    # 최소외접사각형은 배의 앞뒤를 구분할 수 없다(장축만 알 뿐 어느 쪽이 뱃머리인지
    # 모름). 하지만 이건 수학적 모호성이지 물리적 문제가 아니다 - 배가 두 측량
    # 사이에 180도 뒤집힐 일은 없기 때문. 사람이 세션 시작 때 대충 실측한
    # yaw_hint에 가까운 쪽을 고르면 끝난다 (hint는 5도 정확도도 필요 없음).
    if abs(wrap_angle(yaw - yaw_hint)) > math.pi / 2.0:
        yaw = wrap_angle(yaw + math.pi)
    else:
        yaw = wrap_angle(yaw)

    long_side = float(lengths[long_i])
    short_side = float(lengths[(long_i + 1) % 4])
    return float(cx), float(cy), yaw, long_side, short_side, len(pts)


def coverage_bin_count(points_xy, bin_deg: float) -> int:
    """
    점들이 배 둘레 몇 방향에서 관측됐는지 센다 (한 바퀴 완료 판정용).

    중앙값 중심에서 각 점을 바라본 방위각을 bin_deg 간격으로 묶어 서로 다른
    칸의 개수를 반환. 한 면만 계속 봤으면 1~2칸, 한 바퀴 돌았으면 전 칸이 찬다.
    """
    pts = np.asarray(points_xy, dtype=np.float64).reshape(-1, 2)
    if len(pts) < 3:
        return 0
    center = np.median(pts, axis=0)
    bearings = np.degrees(
        np.arctan2(pts[:, 1] - center[1], pts[:, 0] - center[0])) % 360.0
    return int(len(np.unique((bearings // bin_deg).astype(int))))


class ShipSurveyNode(Node):

    def __init__(self):
        super().__init__('ship_survey_node')

        # ---- 입출력 ----
        self.declare_parameter('detection_topic', '/event_detection/uvd')
        self.declare_parameter('output_topic', '/ship_survey/pose')
        self.declare_parameter('result_save_dir', '/tmp/ship_survey_results')
        self.declare_parameter('block_id', 'B1')
        # 조립 단계 확정 신호 (websocket_client가 3초 안정화 후 발행). 재측량 트리거.
        self.declare_parameter('block_level_topic', '/block_level/confirmed')

        # ---- 배 클래스 판별 ----
        # YOLO 클래스는 조립 단계별로 level1~level5. 전부 "배"를 가리키므로
        # 접두사로 거른다. 위험 이벤트 클래스(fallen_person/fire/no_helmet/
        # ship_defect)는 이 접두사가 없어서 자동으로 걸러진다.
        self.declare_parameter('ship_class_prefix', 'level')
        self.declare_parameter('min_confidence', 0.3)

        # ---- 카메라 장착 실측값 ----
        # ★ change_point.py / websocket_client.py와 반드시 같은 값을 유지할 것.
        #   다르면 측량된 배 위치가 위험 이벤트 좌표와 서로 어긋난 맵에 찍힌다.
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('camera_offset_x', 0.135)
        self.declare_parameter('camera_offset_y', -0.089)
        self.declare_parameter('camera_offset_z', 0.110)
        self.declare_parameter('camera_yaw_deg', -90.0)
        self.declare_parameter('tf_timeout_s', 0.3)

        # ---- 측량 종료 조건 ----
        self.declare_parameter('min_points', 50)          # 10Hz 검출 기준 약 5초분
        self.declare_parameter('bearing_bin_deg', 30.0)   # 360/30 = 12칸
        self.declare_parameter('min_covered_bins', 8)     # 12칸 중 8칸 = 약 240도
        self.declare_parameter('max_depth_m', 4.0)        # 이보다 먼 검출은 배가 아님
        self.declare_parameter('outlier_radius_m', 2.0)   # 중앙값에서 이 이상 떨어지면 버림
        # ★ 사람이 세션 시작 때 대충 실측한 배 방향(라디안). 장축의 180도 모호성만
        #   푸는 용도라 정밀할 필요 없음. 배를 반대로 놓았으면 이 값을 바꿀 것.
        self.declare_parameter('yaw_hint_rad', 0.0)

        self.detection_topic = self.get_parameter('detection_topic').value
        self.save_dir = self.get_parameter('result_save_dir').value
        self.block_id = self.get_parameter('block_id').value
        self.class_prefix = self.get_parameter('ship_class_prefix').value
        self.min_confidence = self.get_parameter('min_confidence').value

        self.map_frame = self.get_parameter('map_frame_id').value
        self.base_frame = self.get_parameter('base_frame_id').value
        self.cam_offset_x = self.get_parameter('camera_offset_x').value
        self.cam_offset_y = self.get_parameter('camera_offset_y').value
        self.cam_offset_z = self.get_parameter('camera_offset_z').value
        self.cam_yaw = math.radians(self.get_parameter('camera_yaw_deg').value)
        self.tf_timeout = Duration(seconds=self.get_parameter('tf_timeout_s').value)

        self.min_points = self.get_parameter('min_points').value
        self.bin_deg = self.get_parameter('bearing_bin_deg').value
        self.min_covered_bins = self.get_parameter('min_covered_bins').value
        self.max_depth = self.get_parameter('max_depth_m').value
        self.outlier_radius = self.get_parameter('outlier_radius_m').value
        self.yaw_hint = self.get_parameter('yaw_hint_rad').value

        os.makedirs(self.save_dir, exist_ok=True)
        self.survey_count = self._load_last_index() + 1

        # ---- 상태 ----
        self.points = []        # [(map_x, map_y), ...] 배 표면점 누적
        self.finalized = False  # 확정 후 다음 재측량 트리거까지 수집 중단
        self.current_level = None   # 지금 수집 중인 점들이 어느 조립 단계의 배인지

        # ---- TF ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- 통신 ----
        # ★ TRANSIENT_LOCAL(latched): 측량은 1회만 발행하는데, ROS2 기본값인
        #   VOLATILE이면 그 순간 떠 있지 않던 구독자는 값을 영영 못 받는다.
        #   이걸로 재접속 시 재전송 로직을 따로 짤 필요가 없어진다.
        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.pose_pub = self.create_publisher(
            String, self.get_parameter('output_topic').value, latched_qos)

        self.create_subscription(String, self.detection_topic, self._detection_cb, 10)

        # 재측량 트리거. 발행 쪽(websocket_client)이 단계가 바뀔 때만 latched로
        # 발행하므로 구독도 TRANSIENT_LOCAL이라야 이미 확정된 현재 단계를 받는다.
        self.create_subscription(
            Int32, self.get_parameter('block_level_topic').value,
            self._block_level_cb, latched_qos)

        # 수집 진행상황 감시 (콜백 안에서 확정 판정하면 매 검출마다 도는 낭비)
        self.check_timer = self.create_timer(1.0, self._check_done)

        # 한 바퀴를 못 돌았는데 그만 둬야 할 때 작업자가 강제 확정하는 경로
        self.srv = self.create_service(Trigger, '~/finalize', self._finalize_srv_cb)

        total_bins = int(360.0 / self.bin_deg)
        self.get_logger().info(
            f"ship_survey_node 수집 시작: topic={self.detection_topic}, "
            f"클래스 접두사='{self.class_prefix}', "
            f"종료조건=점 {self.min_points}개 이상 + 방위 {self.min_covered_bins}/{total_bins}칸 커버. "
            f"확정 후에는 조립 단계가 바뀔 때마다 자동 재측량. "
            f"배 주위를 한 바퀴 도세요. 강제 확정은 "
            f"'ros2 service call /ship_survey_node/finalize std_srvs/srv/Trigger'"
        )

    # ------------------------------------------------------------------
    def _load_last_index(self) -> int:
        """기존 결과 파일 중 가장 큰 회차 번호를 찾는다 (uwb_map_calibration과 동일 방식)."""
        try:
            files = [f for f in os.listdir(self.save_dir) if f.startswith('ship_pose_')]
            if not files:
                return 0
            indices = [int(f.split('_')[2].split('.')[0]) for f in files]
            return max(indices)
        except Exception:
            return 0

    # ------------------------------------------------------------------
    def _camera_xyz_to_map_xy(self, depth_xyz):
        """
        카메라 좌표계 3D 점을 map 좌표 2D로 변환.
        change_point.py / websocket_client.py와 동일한 보정 순서를 따른다.
        (depth_xyz는 yolo_depth_publisher가 실제 카메라 내부파라미터로 역투영한
         값이라, change_point.py의 hfov 근사보다 정확해서 그대로 쓴다.)
        """
        x_cam, _y_cam, z_cam = depth_xyz  # y_cam(상하)은 2D 측량에 안 씀

        # 카메라 기준 "전방"은 z_cam, "좌측"은 -x_cam (OpenCV: x=우측)
        cam_local_x = z_cam
        cam_local_y = -x_cam
        cos_yaw = math.cos(self.cam_yaw)
        sin_yaw = math.sin(self.cam_yaw)
        rotated_x = cam_local_x * cos_yaw - cam_local_y * sin_yaw
        rotated_y = cam_local_x * sin_yaw + cam_local_y * cos_yaw

        point_in_base = PointStamped()
        point_in_base.header.frame_id = self.base_frame
        point_in_base.header.stamp = self.get_clock().now().to_msg()
        point_in_base.point.x = rotated_x + self.cam_offset_x
        point_in_base.point.y = rotated_y + self.cam_offset_y
        point_in_base.point.z = self.cam_offset_z

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time(), timeout=self.tf_timeout)
        except Exception as e:
            self.get_logger().warn(
                f"TF 조회 실패 ({self.map_frame}<-{self.base_frame}): {e}",
                throttle_duration_sec=5.0)
            return None

        point_in_map = tf2_geometry_msgs.do_transform_point(point_in_base, transform)
        return point_in_map.point.x, point_in_map.point.y

    # ------------------------------------------------------------------
    def _detection_cb(self, msg: String):
        if self.finalized:
            return

        try:
            det = json.loads(msg.data)
            class_id = str(det.get('class_id', ''))
            confidence = float(det.get('confidence', 0.0))
            depth_xyz = det.get('depth_xyz')
        except (json.JSONDecodeError, ValueError) as e:
            self.get_logger().warn(f"검출 메시지 파싱 실패: {e}", throttle_duration_sec=5.0)
            return

        # 배(level*) 클래스만 사용
        if not class_id.startswith(self.class_prefix):
            return
        if confidence < self.min_confidence:
            return
        if depth_xyz is None or len(depth_xyz) != 3:
            self.get_logger().warn(
                "depth_xyz 없음 - yolo_depth_publisher 최신 버전인지 확인",
                throttle_duration_sec=10.0)
            return

        # depth 유효성: 0 이하는 측정 실패, 너무 멀면 배가 아니라 배경을 문 것
        depth = float(depth_xyz[2])
        if depth <= 0.0 or depth > self.max_depth:
            return

        map_xy = self._camera_xyz_to_map_xy(depth_xyz)
        if map_xy is None:
            return

        self.points.append(map_xy)

    # ------------------------------------------------------------------
    def _block_level_cb(self, msg: Int32):
        """
        조립 단계가 바뀌면 재측량한다 (interface.md ④).
        조립하면서 배가 밀리거나 돌아갈 수 있고, 단계가 올라가면 배의 형태와
        크기 자체가 달라지기 때문.

        ★ 이전 단계에서 모은 점은 전부 버린다. 그 점들은 지금과 다른 형태의 배를
          측량한 것이라, 남겨두면 옛 형태와 새 형태를 합친 엉뚱한 사각형이 나온다.
        """
        level = int(msg.data)

        # 첫 수신은 "변경"이 아니라 현재 단계를 처음 알게 된 것뿐이다. 여기서
        # 초기화해버리면 매핑 랩에서 모으던 점을 통째로 날린다.
        if self.current_level is None:
            self.current_level = level
            self.get_logger().info(f"현재 조립 단계 확인: level={level} (수집 계속)")
            return

        if level == self.current_level:
            return

        self.get_logger().info(
            f"조립 단계 변경 감지: level {self.current_level} -> {level}. "
            f"이전 점 {len(self.points)}개 폐기하고 재측량 시작 "
            f"(Nav2 순찰이 배 주위를 돌면 자동으로 다시 확정됨)")

        self.current_level = level
        self.points = []
        self.finalized = False
        self.survey_count = self._load_last_index() + 1

    # ------------------------------------------------------------------
    def _check_done(self):
        """1초마다 종료조건을 확인하고, 충족되면 자동 확정."""
        if self.finalized:
            return

        num = len(self.points)
        if num < self.min_points:
            self.get_logger().info(
                f"[측량중] 점 {num}/{self.min_points}개 - 배가 카메라(로봇 오른쪽)에 "
                f"들어오게 주행하세요", throttle_duration_sec=5.0)
            return

        covered = coverage_bin_count(self.points, self.bin_deg)
        total_bins = int(360.0 / self.bin_deg)

        if covered < self.min_covered_bins:
            self.get_logger().info(
                f"[측량중] 점 {num}개, 방위 커버 {covered}/{total_bins}칸 "
                f"(목표 {self.min_covered_bins}칸) - 계속 한 바퀴 도세요",
                throttle_duration_sec=5.0)
            return

        self.get_logger().info(f"[측량완료] 점 {num}개, 방위 {covered}/{total_bins}칸 커버 -> 확정")
        self._finalize(trigger='auto')

    # ------------------------------------------------------------------
    def _finalize_srv_cb(self, request, response):
        """작업자가 한 바퀴 못 돌고 강제 확정할 때 쓰는 서비스."""
        if self.finalized:
            response.success = False
            response.message = ("이미 확정됨 - 재측량은 조립 단계가 바뀌면 자동으로 "
                                "시작됩니다. 강제로 다시 하려면 노드를 재시작하세요")
            return response

        if len(self.points) < self.min_points:
            response.success = False
            response.message = (
                f"점 부족 ({len(self.points)}/{self.min_points}) - 확정 거부. "
                f"배가 카메라에 들어오게 더 주행하세요")
            return response

        result = self._finalize(trigger='manual')
        if result is None:
            response.success = False
            response.message = "사각형 피팅 실패 (이상치 제거 후 점 부족)"
            return response

        response.success = True
        response.message = (
            f"측량 확정: map_xy=({result[0]:.3f}, {result[1]:.3f}) "
            f"yaw={result[2]:.3f} size=({result[3]:.3f}, {result[4]:.3f})")
        return response

    # ------------------------------------------------------------------
    def _finalize(self, trigger: str):
        """사각형 피팅 -> 토픽 1회 발행 -> 파일 저장. 이후 수집 중단."""
        result = fit_rect(self.points, self.yaw_hint, self.outlier_radius)
        if result is None:
            self.get_logger().error("사각형 피팅 실패 - 이상치 제거 후 점이 3개 미만")
            return None

        cx, cy, yaw, long_side, short_side, num_used = result

        # 커버리지가 부족한 채로 강제 확정하면 안 본 쪽 면이 사각형에 안 들어가서
        # 크기가 과소평가된다. Nav2 keepout 마스크가 배보다 작아지므로 경고를 남긴다.
        covered = coverage_bin_count(self.points, self.bin_deg)
        total_bins = int(360.0 / self.bin_deg)
        if covered < self.min_covered_bins:
            self.get_logger().warn(
                f"방위 커버 {covered}/{total_bins}칸으로 확정됨 - 못 본 면이 있어 "
                f"size_xy가 실제보다 작을 수 있음 (keepout 마스크 주의)")

        # ---- 출력 ① 토픽 (docs/interface.md ④ 스펙 그대로, size_xy는 안 넣음) ----
        payload = {
            'event_type': 'ship_pose',
            'block_id': self.block_id,
            'map_xy': [cx, cy],
            'yaw': yaw,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.pose_pub.publish(msg)

        # ---- 출력 ② 파일 (size_xy 포함, Nav2 keepout 마스크용) ----
        # size_xy = [장축 길이, 단축 길이]. yaw가 장축 방향이므로 이 순서라야
        # (yaw, size_xy) 조합만으로 사각형이 유일하게 복원된다.
        path = os.path.join(self.save_dir, f'ship_pose_{self.survey_count:03d}.json')
        data = dict(payload)
        data['size_xy'] = [long_side, short_side]
        # 어느 조립 단계의 배를 잰 것인지. 단계마다 배 형태가 달라지므로 파일이
        # 여러 개 쌓였을 때 어느 것이 지금 배인지 구분하는 데 필요하다.
        data['block_level'] = self.current_level
        data['timestamp'] = time.time()
        data['num_points'] = num_used
        data['covered_bins'] = covered
        data['total_bins'] = total_bins
        data['trigger'] = trigger
        try:
            with open(path, 'w') as f:
                json.dump(data, f, indent=2)
            self.get_logger().info(f"측량 결과 저장: {path}")
        except OSError as e:
            # 파일 저장이 실패해도 토픽은 이미 나갔으므로 노드를 죽이지 않는다.
            self.get_logger().error(f"결과 파일 저장 실패 ({path}): {e}")

        # 타이머는 살려둔다 (재측량 트리거가 오면 다시 써야 함). finalized 플래그로
        # 게이팅하므로 확정 상태에서는 1초마다 즉시 return만 한다.
        self.finalized = True
        self.points = []

        self.get_logger().info(
            f"[배 측량 확정] map_xy=({cx:.3f}, {cy:.3f}) yaw={yaw:.3f}rad "
            f"({math.degrees(yaw):.1f}deg) size_xy=({long_side:.3f}, {short_side:.3f})m "
            f"점 {num_used}개 사용")
        return cx, cy, yaw, long_side, short_side


def main(args=None):
    rclpy.init(args=args)
    node = ShipSurveyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
