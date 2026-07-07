#!/usr/bin/env python3
"""
UWB <-> Map Calibration Node (ROS2)
------------------------------------
앵커가 매일 재배치되는 운영 환경을 전제로 설계됨.

목적: uwb_frame(UWB 앵커 좌표계, 원점/축이 매일 바뀜) -> map(고정 세계 좌표계)
      의 회전 + 평행이동을 구해 정적 TF(map -> uwb_frame)로 발행한다.

동작 방식
---------
1. 노드는 시작 시 IDLE 상태로 대기한다. (재시작 없이 언제든 재캘리브레이션 가능해야 하므로
   노드 자체는 계속 떠 있고, 실제 캘리브레이션 로직만 서비스 호출로 트리거됨)
2. 사용자가 ~/calibrate (std_srvs/Trigger) 서비스를 호출하면:
   a. 로봇을 정지 상태에서 알고 있는 방향(예: map 좌표계 +x 방향)으로 직진시킨다는
      전제 하에, 그 구간 동안의 /uwb/pose 샘플을 수집한다 (COLLECTING 상태).
   b. 수집 시간(기본 5초) 종료 후 시작점/끝점을 직선 피팅하여 uwb_frame 상에서의
      진행 방향 벡터를 구하고, 이를 map 좌표계 상의 알려진 직진 방향과 비교해
      회전각(theta)을 역산한다.
   c. 시작점을 두 좌표계의 원점 오프셋 계산에 사용해 평행이동(tx, ty)을 구한다.
   d. map -> uwb_frame 정적 TF를 발행(갱신)한다.
   e. 결과를 회차 번호를 붙여 파일로 저장한다 (재현/디버깅용).
3. 서비스는 몇 번이고 다시 호출 가능 (앵커 재배치 후 노드 재시작 불필요).

주의: 로봇이 실제로 알려진 방향(관례상 map +x축)으로 "직선으로" 주행해야 한다.
이 노드는 주행 자체를 제어하지 않는다 - 사람이 조종하거나 별도 직진 액션이
이 서비스 호출과 동시에 실행되어야 한다.
"""

import json
import math
import os
import time
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster


class CalibState(Enum):
    IDLE = auto()
    COLLECTING = auto()


def yaw_to_quaternion(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class UwbMapCalibration(Node):

    def __init__(self):
        super().__init__('uwb_map_calibration')

        # ---- 파라미터 ----
        self.declare_parameter('uwb_pose_topic', '/uwb/pose')
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('uwb_frame_id', 'uwb_frame')
        self.declare_parameter('collection_duration_s', 5.0)
        self.declare_parameter('min_travel_distance_m', 1.0)  # 직선 피팅 신뢰를 위한 최소 이동거리
        self.declare_parameter('known_heading_in_map_rad', 0.0)  # 로봇이 직진한 방향 (map 기준, 보통 +x = 0)
        self.declare_parameter('result_save_dir', '/tmp/uwb_calibration_results')

        self.map_frame_id = self.get_parameter('map_frame_id').value
        self.uwb_frame_id = self.get_parameter('uwb_frame_id').value
        self.collection_duration = self.get_parameter('collection_duration_s').value
        self.min_travel = self.get_parameter('min_travel_distance_m').value
        self.known_heading = self.get_parameter('known_heading_in_map_rad').value
        self.save_dir = self.get_parameter('result_save_dir').value
        os.makedirs(self.save_dir, exist_ok=True)

        # ---- 상태 ----
        self.state = CalibState.IDLE
        self.samples = []          # [(t, x, y), ...] 수집 중 UWB 원시 샘플
        self.collect_start_time = None
        self.calibration_count = self._load_last_index() + 1

        # 현재 유효한 변환 (theta, tx, ty). 최초 기본값은 항등변환.
        self.theta = 0.0
        self.tx = 0.0
        self.ty = 0.0

        # ---- 통신 ----
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter('uwb_pose_topic').value,
            self._uwb_cb, 20)

        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_tf()  # 시작 시 항등변환으로 일단 발행 (TF tree 끊김 방지)

        self.srv = self.create_service(Trigger, '~/calibrate', self._calibrate_cb)

        # 수집 종료 감시 타이머 (서비스 콜백 블로킹 금지 → 타이머 기반 상태머신)
        self.check_timer = self.create_timer(0.2, self._check_collection_done)

        self.get_logger().info(
            "uwb_map_calibration IDLE 상태로 대기 중. "
            "'~/calibrate' 서비스 호출 시 로봇을 known_heading_in_map_rad 방향으로 "
            f"{self.collection_duration}초간 직진시키세요."
        )

    # ------------------------------------------------------------------
    def _load_last_index(self) -> int:
        try:
            files = [f for f in os.listdir(self.save_dir) if f.startswith('calib_')]
            if not files:
                return 0
            indices = [int(f.split('_')[1].split('.')[0]) for f in files]
            return max(indices)
        except Exception:
            return 0

    # ------------------------------------------------------------------
    def _uwb_cb(self, msg: PoseWithCovarianceStamped):
        if self.state != CalibState.COLLECTING:
            return
        t = self._stamp_to_sec(msg.header.stamp)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.samples.append((t, x, y))

    # ------------------------------------------------------------------
    def _calibrate_cb(self, request, response):
        """
        주의: 서비스 콜백 안에서 rclpy.spin_once()로 블로킹 대기하면 안 된다.
        (이미 executor가 이 콜백을 실행 중이므로 데드락/예외 발생)
        따라서 이 서비스는 수집 '시작'만 트리거하고 즉시 리턴하며,
        실제 수집 종료와 계산은 타이머 콜백(_check_collection_done)이 담당한다.
        결과는 로그와 저장 파일로 확인한다.
        """
        if self.state == CalibState.COLLECTING:
            response.success = False
            response.message = "이미 캘리브레이션 수집 중입니다."
            return response

        self.get_logger().info(
            f"캘리브레이션 시작: {self.collection_duration}초간 "
            "/uwb/pose 샘플을 수집합니다. 지금부터 로봇을 직진시키세요. "
            "결과는 수집 종료 후 로그/저장 파일로 확인하세요."
        )
        self.samples = []
        self.state = CalibState.COLLECTING
        self.collect_start_time = time.time()

        response.success = True
        response.message = (
            f"수집 시작됨 ({self.collection_duration}s). "
            "종료 후 결과는 로그와 result_save_dir 파일로 확인."
        )
        return response

    def _check_collection_done(self):
        """주기 타이머: 수집 시간이 다 되면 계산을 수행."""
        if self.state != CalibState.COLLECTING:
            return
        if time.time() - self.collect_start_time < self.collection_duration:
            return

        self.state = CalibState.IDLE
        success, message = self._compute_calibration()
        if success:
            self.get_logger().info(f"[캘리브레이션 성공] {message}")
        else:
            self.get_logger().error(f"[캘리브레이션 실패] {message}")

    # ------------------------------------------------------------------
    def _compute_calibration(self):
        if len(self.samples) < 10:
            return False, f"샘플 부족 ({len(self.samples)}개). 재시도하세요."

        start = self.samples[0]
        end = self.samples[-1]
        dx = end[1] - start[1]
        dy = end[2] - start[2]
        travel = math.hypot(dx, dy)

        if travel < self.min_travel:
            return False, (
                f"이동거리 부족 ({travel:.2f}m < {self.min_travel}m). "
                "더 길게, 더 곧게 직진 후 재시도하세요."
            )

        # uwb_frame 상에서 관측된 진행 방향
        uwb_heading = math.atan2(dy, dx)

        # map 상에서는 known_heading_in_map_rad 방향으로 움직였다고 알고 있으므로
        # 회전각 theta = (map에서의 방향) - (uwb_frame에서의 방향)
        # 이 theta는 uwb_frame -> map 회전. 우리는 map -> uwb_frame TF를 발행해야 하므로
        # 최종적으로 역변환을 적용한다.
        theta_uwb_to_map = self._wrap(self.known_heading - uwb_heading)

        # 시작점을 이용한 평행이동 계산:
        # map 좌표 = R(theta) * uwb 좌표 + T  =>  T = map_start - R(theta) * uwb_start
        # 여기서는 시작점의 map 좌표를 별도로 알 수 없으므로, 시작점을 캘리브레이션
        # 기준 원점(0,0)으로 정의하는 실용적 규약을 사용한다. (조선소 현장에서
        # 직진 시작 지점을 map 원점 부근의 알려진 기준점에 두는 운영 절차 전제)
        cos_t, sin_t = math.cos(theta_uwb_to_map), math.sin(theta_uwb_to_map)
        rotated_start_x = cos_t * start[1] - sin_t * start[2]
        rotated_start_y = sin_t * start[1] + cos_t * start[2]
        tx_uwb_to_map = 0.0 - rotated_start_x
        ty_uwb_to_map = 0.0 - rotated_start_y

        # 우리가 실제로 발행할 static TF는 map -> uwb_frame (부모: map, 자식: uwb_frame)
        # 이는 uwb_frame -> map 변환의 역변환이다.
        theta_map_to_uwb = -theta_uwb_to_map
        cos_i, sin_i = math.cos(theta_map_to_uwb), math.sin(theta_map_to_uwb)
        tx_map_to_uwb = -(cos_i * tx_uwb_to_map - sin_i * ty_uwb_to_map)
        ty_map_to_uwb = -(sin_i * tx_uwb_to_map + cos_i * ty_uwb_to_map)

        self.theta = theta_map_to_uwb
        self.tx = tx_map_to_uwb
        self.ty = ty_map_to_uwb

        self._publish_static_tf()
        self._save_result(travel, uwb_heading, theta_uwb_to_map)

        msg = (
            f"캘리브레이션 완료 #{self.calibration_count}: "
            f"theta={math.degrees(theta_uwb_to_map):.2f}deg (uwb->map), "
            f"travel={travel:.2f}m, samples={len(self.samples)}"
        )
        self.get_logger().info(msg)
        self.calibration_count += 1
        return True, msg

    # ------------------------------------------------------------------
    def _publish_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame_id
        t.child_frame_id = self.uwb_frame_id
        t.transform.translation.x = self.tx
        t.transform.translation.y = self.ty
        t.transform.translation.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(self.theta)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

    def _save_result(self, travel, uwb_heading, theta_uwb_to_map):
        path = os.path.join(self.save_dir, f'calib_{self.calibration_count:03d}.json')
        data = {
            'timestamp': time.time(),
            'travel_distance_m': travel,
            'uwb_heading_rad': uwb_heading,
            'theta_uwb_to_map_rad': theta_uwb_to_map,
            'published_map_to_uwb_theta_rad': self.theta,
            'published_map_to_uwb_tx': self.tx,
            'published_map_to_uwb_ty': self.ty,
            'num_samples': len(self.samples),
        }
        with open(path, 'w') as f:
            json.dump(data, f, indent=2)
        self.get_logger().info(f"캘리브레이션 결과 저장: {path}")

    @staticmethod
    def _wrap(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return stamp.sec + stamp.nanosec * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = UwbMapCalibration()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
