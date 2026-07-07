#!/usr/bin/env python3
"""
change_point.py (리팩터링판)
----------------------------
Depth camera 이벤트 감지 (u, v, depth)를 map 좌표계의 절대 위치로 변환한다.

과거 버전과의 차이 (리팩터링 핵심)
-----------------------------------
1. 자체 UART 연결(UwbTagReader)과 자체 이동벡터 기반 heading 추정(HeadingEstimator)을
   완전히 제거했다. 이제는 ekf_global이 발행하는 map -> base_link TF를 조회해서
   로봇의 현재 pose(위치+자세)를 얻는다. 중복된 위치추정 로직을 두 곳에서
   유지보수하는 것을 방지하고, 시스템 전체의 단일 진실 공급원(EKF)을 따른다.
2. 좌표축 버그 수정: 카메라 좌표계에서 화면 왼쪽(-u 방향)에 있는 물체가
   로봇 기준 왼쪽(+y, REP-103/CCW 규약)에 있어야 하는데, 과거에는
   local_y = x_cam 으로 부호가 뒤집혀 있어 로봇이 회전할 때마다 감지된
   물체 위치가 좌우 반전되어 나타났다. local_y = -x_cam 으로 수정.
3. quality(과거 UWB QF 기반)를 ekf_global의 pose covariance 기반
   "위치 불확실성" 지표로 대체했다. UWB 품질이 아니라 최종 위치 추정치의
   실제 신뢰도를 반영하는 것이 더 정확하다.
4. 카메라 감지 입력은 아직 AI 파이프라인이 없으므로 /event_detection/uvd
   placeholder 토픽(u, v, depth, class_id 등 JSON)을 임시로 구독한다.

REP-103 / CCW 좌표계 규약
-------------------------
base_link: x=전방, y=좌측, z=상방 (오른손 좌표계, yaw는 CCW 양수)
카메라(광학) 좌표계: x=우측, y=하방, z=전방 (표준 OpenCV/광학 좌표계)
따라서 카메라의 +x_cam(화면 오른쪽)은 로봇 기준 -y(우측) 방향이 된다.
  local_y = -x_cam   <-- 이번에 고친 부분 (과거: local_y = x_cam, 좌우 반전 버그)
  local_x = depth (전방 거리, 카메라 z_cam)
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401  (PointStamped 변환을 위해 필요한 등록)


class ChangePointDetector(Node):

    def __init__(self):
        super().__init__('change_point_detector')

        # ---- 파라미터 ----
        self.declare_parameter('detection_topic', '/event_detection/uvd')
        self.declare_parameter('output_topic', '/event_detection/map_point')
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('base_frame_id', 'base_link')
        # base_link -> camera_link 오프셋 (아직 TF로 편입 전이므로 코드 상수로 유지)
        self.declare_parameter('camera_offset_x', 0.15)  # 전방 오프셋 (m)
        self.declare_parameter('camera_offset_y', 0.0)
        self.declare_parameter('camera_offset_z', 0.20)
        # 카메라 내부 파라미터 (u,v를 각도로 변환하기 위한 간이 핀홀 모델)
        self.declare_parameter('camera_hfov_deg', 69.0)  # 예: RealSense D435 수평 FOV
        self.declare_parameter('image_width', 640)
        # depth 의미 해석 (카메라 모델 확정 시 datasheet로 반드시 확인!):
        #   False (기본): depth = Z-depth (광축에 수직인 평면까지의 거리; RealSense 등 대부분의 뎁스카메라 표준)
        #   True        : depth = radial distance (카메라 원점에서 픽셀 방향으로의 직선거리/빗변)
        # 잘못 설정하면 화각 가장자리에서 위치 오차가 커진다 (중심부는 차이 미미).
        self.declare_parameter('depth_is_radial', False)
        self.declare_parameter('tf_timeout_s', 0.3)

        self.map_frame = self.get_parameter('map_frame_id').value
        self.base_frame = self.get_parameter('base_frame_id').value
        self.cam_offset = (
            self.get_parameter('camera_offset_x').value,
            self.get_parameter('camera_offset_y').value,
            self.get_parameter('camera_offset_z').value,
        )
        self.hfov = math.radians(self.get_parameter('camera_hfov_deg').value)
        self.image_width = self.get_parameter('image_width').value
        self.depth_is_radial = self.get_parameter('depth_is_radial').value
        self.tf_timeout = Duration(seconds=self.get_parameter('tf_timeout_s').value)

        # ---- TF ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- 통신 ----
        self.create_subscription(
            String, self.get_parameter('detection_topic').value,
            self._detection_cb, 10)
        self.pub = self.create_publisher(
            String, self.get_parameter('output_topic').value, 10)

        self.get_logger().info(
            "change_point_detector 시작: map->base_link TF 조회 기반 (독자 UART/heading 로직 없음)"
        )

    # ------------------------------------------------------------------
    def _detection_cb(self, msg: String):
        try:
            det = json.loads(msg.data)
            u = float(det['u'])
            v = float(det['v'])
            depth = float(det['depth'])
            class_id = det.get('class_id', 'unknown')
            confidence = det.get('confidence', None)
        except (json.JSONDecodeError, KeyError, ValueError) as e:
            self.get_logger().warn(f"감지 메시지 파싱 실패: {e}")
            return

        if depth <= 0.0:
            self.get_logger().debug("depth<=0, 무효 감지 스킵")
            return

        # --- 1) (u, v, depth) -> 카메라 좌표계 (x_cam, y_cam, z_cam) ---
        # 간이 핀홀 모델: 수평 각도만 이용 (2D 로봇 평면 투영 목적이므로 v는 참고용)
        focal_px = (self.image_width / 2.0) / math.tan(self.hfov / 2.0)
        cx = self.image_width / 2.0
        angle = math.atan2(u - cx, focal_px)  # 화면 중심 기준 좌우 각도

        if self.depth_is_radial:
            # depth = 빗변 (radial): 삼각분해
            x_cam = depth * math.sin(angle)   # 카메라 좌우축상의 오프셋
            z_cam = depth * math.cos(angle)   # 카메라 전방 거리
        else:
            # depth = Z-depth (기본, RealSense류 표준): 광축 성분이 그대로 depth
            z_cam = depth
            x_cam = depth * math.tan(angle)

        # --- 2) 카메라 좌표계 -> base_link 좌표계 (REP-103, 좌우축 버그 수정 지점) ---
        local_x = z_cam + self.cam_offset[0]
        local_y = -x_cam + self.cam_offset[1]   # <-- 수정: 과거 local_y = x_cam (좌우 반전 버그)
        local_z = self.cam_offset[2]

        # --- 3) base_link -> map 변환 (TF 조회) ---
        point_in_base = PointStamped()
        point_in_base.header.frame_id = self.base_frame
        point_in_base.header.stamp = msg.header.stamp if hasattr(msg, 'header') else self.get_clock().now().to_msg()
        point_in_base.point.x = local_x
        point_in_base.point.y = local_y
        point_in_base.point.z = local_z

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame,
                Time(),  # 최신 TF 사용 (헤더 타임스탬프에 타임아웃 이슈 있어 임시로 최신값 사용 중 - TODO 항목)
                timeout=self.tf_timeout)
        except Exception as e:
            self.get_logger().warn(f"TF 조회 실패 ({self.map_frame}<-{self.base_frame}): {e}")
            return

        point_in_map = tf2_geometry_msgs.do_transform_point(point_in_base, transform)

        # --- 4) 위치 불확실성: ekf_global pose covariance 기반 (TODO: 실제 구독 연동) ---
        # 현재는 placeholder 상수. ekf_global의 /odometry/filtered 또는
        # /amcl_pose 유사 covariance 토픽을 구독해 xy covariance trace로 대체 예정.
        position_uncertainty_m = self._estimate_position_uncertainty()

        out = {
            'stamp': self.get_clock().now().to_msg().sec,
            'class_id': class_id,
            'confidence': confidence,
            'map_x': point_in_map.point.x,
            'map_y': point_in_map.point.y,
            'position_uncertainty_m': position_uncertainty_m,
        }
        out_msg = String()
        out_msg.data = json.dumps(out)
        self.pub.publish(out_msg)

    # ------------------------------------------------------------------
    def _estimate_position_uncertainty(self) -> float:
        """
        ekf_global pose covariance 기반 위치 불확실성.
        TODO: /odometry/filtered(ekf_global) 구독 후 covariance[0], covariance[7]의
        trace(sqrt(cov_xx + cov_yy))로 대체. 현재는 임시 고정값.
        """
        return 0.15  # meters, placeholder


def main(args=None):
    rclpy.init(args=args)
    node = ChangePointDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
