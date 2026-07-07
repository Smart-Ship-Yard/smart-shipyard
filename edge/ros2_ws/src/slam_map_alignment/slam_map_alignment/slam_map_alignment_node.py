#!/usr/bin/env python3
"""
slam_map_alignment_node.py
----------------------------
매핑 세션(slam_toolbox mapping 모드) 동안 동시에 기록되는 두 궤적:
  - /uwb/pose        (uwb_frame, uwb_map_calibration의 정적 TF로 이미 'map'으로 변환 가능)
  - /slam_pose       (slam_toolbox의 위치 추정, frame_id='slam_map' — 반드시 'map'과
                       다른 이름으로 리매핑해서 매핑 중 이름 충돌을 피해야 함! 아래 참고)

을 시간 정렬해 대응점 쌍으로 모으고, RANSAC 기반 2D 강체변환으로
slam_map -> map 변환(theta, tx, ty)을 구해 정적 TF로 발행한다.

*** 중요: slam_toolbox 파라미터 설정 ***
매핑 세션 동안 slam_toolbox의 map_frame 파라미터를 'map'이 아니라 'slam_map'으로
설정해야 한다. 그렇지 않으면 uwb_map_calibration이 이미 정의해 둔 'map'과
이름은 같지만 원점이 다른 '동명이인 프레임'이 TF 트리에 동시에 존재하게 되어,
어느 소스의 'map'인지 구분할 수 없는 위험한 상태가 된다.

  slam_toolbox 파라미터 예:
    map_frame: slam_map
    odom_frame: odom
    base_frame: base_link

동작 순서
---------
1. 노드는 시작하자마자 COLLECTING 상태로 buffer에 계속 (t, x, y) 쌍을 쌓는다
   (매핑 주행 내내 자동으로 기록됨. 별도 시작 트리거 불필요).
2. UWB 포인트는 map 프레임으로, SLAM 포인트는 slam_map 프레임으로 각각 버퍼에 쌓인다.
3. 사용자가 매핑을 마친 뒤 ~/align (std_srvs/Trigger)를 호출하면:
   a. 두 버퍼를 시간 기준으로 짝짓는다 (UWB 두 샘플 사이를 선형보간해서
      SLAM 포즈 타임스탬프에 맞춤).
   b. RANSAC 강체변환으로 (theta, tx, ty)를 추정한다.
   c. map -> slam_map 정적 TF를 발행한다.
   d. 결과를 회차 번호를 붙여 파일로 저장한다.
"""

import json
import math
import os
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, PointStamped
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster, Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401 (PointStamped 변환 등록)

from slam_map_alignment.rigid_transform_2d import ransac_rigid_transform_2d


def yaw_to_quaternion(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class SlamMapAlignmentNode(Node):

    def __init__(self):
        super().__init__('slam_map_alignment')

        # ---- 파라미터 ----
        self.declare_parameter('uwb_pose_topic', '/uwb/pose')       # frame_id='uwb_frame' — _uwb_cb에서 map←uwb_frame TF로 변환 후 사용 (캘리브레이션 선행 필수)
        self.declare_parameter('slam_pose_topic', '/slam_toolbox/pose')  # frame_id='slam_map'
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('slam_map_frame_id', 'slam_map')
        self.declare_parameter('buffer_max_size', 20000)
        self.declare_parameter('max_interp_gap_s', 0.5)   # 이보다 UWB 샘플 간격이 벌어지면 보간 안 함(신뢰 불가)
        self.declare_parameter('inlier_threshold_m', 0.3)
        self.declare_parameter('ransac_iterations', 500)
        self.declare_parameter('min_inlier_ratio', 0.4)
        self.declare_parameter('result_save_dir', '/tmp/slam_map_alignment_results')

        self.map_frame_id = self.get_parameter('map_frame_id').value
        self.slam_map_frame_id = self.get_parameter('slam_map_frame_id').value
        self.max_interp_gap = self.get_parameter('max_interp_gap_s').value
        self.inlier_threshold = self.get_parameter('inlier_threshold_m').value
        self.ransac_iterations = self.get_parameter('ransac_iterations').value
        self.min_inlier_ratio = self.get_parameter('min_inlier_ratio').value
        self.save_dir = self.get_parameter('result_save_dir').value
        os.makedirs(self.save_dir, exist_ok=True)

        buf_size = self.get_parameter('buffer_max_size').value
        self.uwb_buffer = deque(maxlen=buf_size)   # [(t, x, y), ...] 시간순
        self.slam_buffer = deque(maxlen=buf_size)  # [(t, x, y), ...] 시간순

        # 현재 유효한 변환 (theta, tx, ty): slam_map -> map. 기본값은 항등변환.
        self.theta = 0.0
        self.tx = 0.0
        self.ty = 0.0
        self.align_count = self._load_last_index() + 1

        # ---- 통신 ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._last_tf_warn = 0.0

        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter('uwb_pose_topic').value,
            self._uwb_cb, 50)
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter('slam_pose_topic').value,
            self._slam_cb, 50)

        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_tf()  # 시작 시 항등변환으로 우선 발행 (TF 트리 끊김 방지)

        self.srv = self.create_service(Trigger, '~/align', self._align_cb)

        self.get_logger().info(
            "slam_map_alignment 시작: 매핑 주행 동안 자동으로 궤적을 기록합니다. "
            "매핑이 끝나면 '~/align' 서비스를 호출하세요. "
            "(slam_toolbox의 map_frame 파라미터가 'slam_map'으로 설정되어 있는지 꼭 확인하세요)"
        )

    # ------------------------------------------------------------------
    def _load_last_index(self) -> int:
        try:
            files = [f for f in os.listdir(self.save_dir) if f.startswith('align_')]
            if not files:
                return 0
            indices = [int(f.split('_')[1].split('.')[0]) for f in files]
            return max(indices)
        except Exception:
            return 0

    # ------------------------------------------------------------------
    def _uwb_cb(self, msg: PoseWithCovarianceStamped):
        """
        중요 (좌표계 정합 버그 수정): /uwb/pose는 uwb_frame 좌표다.
        대응점의 dst는 반드시 'map' 좌표여야 하므로, uwb_map_calibration이
        발행해 둔 map←uwb_frame TF로 변환한 뒤 버퍼에 넣는다.
        이 변환 없이 원시값을 쓰면 계산 결과가 map→slam_map이 아니라
        uwb_frame→slam_map이 되어 발행 TF의 의미가 틀어진다.
        전제: 이 노드로 정합을 수행하기 전에 uwb_map_calibration의
        ~/calibrate가 먼저 완료되어 있어야 한다.
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame_id, msg.header.frame_id, Time())
        except Exception:
            # 캘리브레이션 TF가 아직 없으면 이 샘플은 버림 (경고는 스로틀)
            now_s = time.time()
            if now_s - self._last_tf_warn > 5.0:
                self.get_logger().warn(
                    f"'{self.map_frame_id}'<-'{msg.header.frame_id}' TF 조회 실패. "
                    "uwb_map_calibration의 ~/calibrate를 먼저 수행했는지 확인하세요."
                )
                self._last_tf_warn = now_s
            return

        p = PointStamped()
        p.header = msg.header
        p.point.x = msg.pose.pose.position.x
        p.point.y = msg.pose.pose.position.y
        p_map = tf2_geometry_msgs.do_transform_point(p, tf)

        t = stamp_to_sec(msg.header.stamp)
        self.uwb_buffer.append((t, p_map.point.x, p_map.point.y))

    def _slam_cb(self, msg: PoseWithCovarianceStamped):
        if msg.header.frame_id == self.map_frame_id:
            self.get_logger().warn(
                f"slam pose의 frame_id가 '{self.map_frame_id}'입니다. "
                "slam_toolbox의 map_frame 파라미터를 'slam_map'으로 리매핑했는지 확인하세요 "
                "(이름 충돌 위험)."
            )
        t = stamp_to_sec(msg.header.stamp)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.slam_buffer.append((t, x, y))

    # ------------------------------------------------------------------
    def _interpolate_uwb_at(self, t_query: float):
        """
        UWB 버퍼에서 t_query 시점을 감싸는 두 샘플을 찾아 선형보간.
        감싸는 샘플이 없거나 간격이 너무 크면 None.
        """
        buf = self.uwb_buffer
        if len(buf) < 2:
            return None

        # 이진탐색 대신 선형탐색 (버퍼 크기가 크지 않다는 전제, 필요시 bisect로 교체 가능)
        prev = None
        for sample in buf:
            if sample[0] <= t_query:
                prev = sample
            else:
                nxt = sample
                if prev is None:
                    return None
                dt = nxt[0] - prev[0]
                if dt <= 0 or dt > self.max_interp_gap:
                    return None
                ratio = (t_query - prev[0]) / dt
                x = prev[1] + ratio * (nxt[1] - prev[1])
                y = prev[2] + ratio * (nxt[2] - prev[2])
                return (x, y)
        return None  # t_query가 버퍼 마지막 샘플보다 뒤

    # ------------------------------------------------------------------
    def _build_correspondences(self):
        src_points = []  # slam_map 프레임
        dst_points = []  # map 프레임 (UWB, 보간됨)

        for t_slam, x_slam, y_slam in self.slam_buffer:
            uwb_xy = self._interpolate_uwb_at(t_slam)
            if uwb_xy is None:
                continue
            src_points.append((x_slam, y_slam))
            dst_points.append(uwb_xy)

        return np.array(src_points), np.array(dst_points)

    # ------------------------------------------------------------------
    def _align_cb(self, request, response):
        src_points, dst_points = self._build_correspondences()

        if len(src_points) < 10:
            response.success = False
            response.message = (
                f"대응점 부족 ({len(src_points)}개). 매핑 주행 데이터가 충분히 "
                "쌓였는지, UWB/SLAM 토픽이 정상 발행 중인지 확인하세요."
            )
            return response

        try:
            theta, tx, ty, inlier_mask = ransac_rigid_transform_2d(
                src_points, dst_points,
                inlier_threshold_m=self.inlier_threshold,
                num_iterations=self.ransac_iterations,
                min_inlier_ratio=self.min_inlier_ratio,
            )
        except ValueError as e:
            response.success = False
            response.message = f"정합 실패: {e}"
            return response

        self.theta = theta
        self.tx = tx
        self.ty = ty
        self._publish_static_tf()

        inlier_ratio = float(inlier_mask.sum()) / len(inlier_mask)
        self._save_result(len(src_points), inlier_ratio, theta, tx, ty)

        msg = (
            f"정합 완료 #{self.align_count}: theta={math.degrees(theta):.2f}deg, "
            f"tx={tx:.3f}, ty={ty:.3f}, "
            f"대응점={len(src_points)}, inlier 비율={inlier_ratio:.1%}"
        )
        self.get_logger().info(msg)
        self.align_count += 1

        response.success = True
        response.message = msg
        return response

    # ------------------------------------------------------------------
    def _publish_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame_id
        t.child_frame_id = self.slam_map_frame_id
        t.transform.translation.x = self.tx
        t.transform.translation.y = self.ty
        t.transform.translation.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(self.theta)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

    def _save_result(self, num_points, inlier_ratio, theta, tx, ty):
        path = os.path.join(self.save_dir, f'align_{self.align_count:03d}.json')
        data = {
            'timestamp': time.time(),
            'num_correspondence_points': num_points,
            'inlier_ratio': inlier_ratio,
            'theta_rad': theta,
            'tx': tx,
            'ty': ty,
        }
        with open(path, 'w') as f:
            json.dump(data, f, indent=2)
        self.get_logger().info(f"정합 결과 저장: {path}")


def main(args=None):
    rclpy.init(args=args)
    node = SlamMapAlignmentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
