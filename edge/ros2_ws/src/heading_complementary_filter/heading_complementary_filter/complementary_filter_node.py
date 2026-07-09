#!/usr/bin/env python3
"""
Heading Complementary Filter (ROS2)
------------------------------------
IMU 자이로(각속도, 빠르지만 drift O)와 UWB course-over-ground(느리지만 절대적, 튐 있음)를
원형(circular) 상보필터로 블렌딩하여 orientation-only IMU 메시지를 발행한다.

발행: /heading/imu_uwb_fused (sensor_msgs/Imu) - orientation.z(yaw)만 유효.
      이 토픽은 ekf_global의 imu 입력으로 들어가 yaw만 fuse된다.

왜 필요한가
-----------
조선소는 금속 구조물 때문에 마그네토미터 기반 절대 yaw를 신뢰할 수 없다.
그렇다고 자이로 적분만 쓰면 시간이 지날수록 yaw가 drift한다.
UWB 두 시점 사이의 이동 벡터로부터 계산한 course-over-ground(진행방향)는
절대적이지만, (a) 저속/정지 시 노이즈로 무의미해지고 (b) 후진 시 진행방향이
로봇의 실제 heading과 180도 반대가 된다.

따라서:
  - 자이로 적분 각도를 "빠른 성분", UWB course를 "느린 보정 성분"으로 삼는
    상보필터(complementary filter)를 원형 통계로 구성 (wraparound 안전 처리)
  - ekf_local의 linear velocity 부호로 전진/후진을 판별해 course 보정
  - 최소 이동거리(30cm) 미만 구간에서는 UWB course를 신뢰하지 않고
    자이로 적분만 사용 (저속 노이즈 방지)

[2026-07-08 좌표계 편향 버그 수정 — 시뮬레이션 테스트로 발견]
--------------------------------------------------------------
기존 버그: /uwb/pose의 '원시 uwb_frame 좌표'로 course를 계산했다.
이 yaw는 ekf_global에 "map 기준 절대 yaw"로 들어가는데, 실제로는
uwb_frame 기준이라 캘리브레이션 회전각만큼 항상 편향된 yaw가 주입됐다
(가짜 센서 테스트: uwb_frame 15도 회전 시뮬레이션 -> yaw가 -15도 근처로
편향 초기화되는 것을 실측으로 확인). 앵커가 매일 임의 방향으로 재배치되는
운영 환경에서는 이 편향이 매일 랜덤하게 발생한다.
slam_map_alignment에서 예전에 고친 것과 동일한 유형의 버그.

수정 내용:
1. _uwb_cb에서 map <- uwb_frame TF를 조회해 좌표를 map 프레임으로 변환한 뒤
   course를 계산한다. (uwb_map_calibration이 발행하는 정적 TF 사용.
   따라서 이 필터도 캘리브레이션 선행이라는 순서 의존성을 가진다 —
   slam_map_alignment와 동일한 의존성.)
2. 캘리브레이션 재실행으로 TF 회전이 크게(기본 2도 이상) 바뀌면, 기존
   yaw_est는 옛 좌표계 기준의 편향된 값이므로 yaw를 재초기화한다
   (None으로 리셋 -> 다음 30cm 이동에서 올바른 map 기준 course로 재탄생).
   uwb_map_calibration이 시작 시 항등변환을 먼저 발행하므로, 항등(미보정)
   상태에서 초기화된 yaw도 캘리브레이션 완료 순간 자동으로 교정된다.
3. TF 조회 실패 시(캘리브레이션 노드 미기동 등) 원시 좌표를 쓰지 않고
   그 샘플을 건너뛴다 (틀린 절대값 주입 방지 — "모르면 말하지 않는다").
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener


def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_to_quaternion(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """쿼터니언 -> yaw (2D 평면 가정). 표준 변환 공식의 yaw 성분."""
    return math.atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz))


class HeadingComplementaryFilter(Node):

    def __init__(self):
        super().__init__('heading_complementary_filter')

        # ---- 파라미터 ----
        self.declare_parameter('alpha', 0.98)  # 자이로(빠름) 가중치, (1-alpha)는 UWB course
        self.declare_parameter('min_move_distance_m', 0.30)  # 최소 이동거리 임계값
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('uwb_pose_topic', '/uwb/pose')
        self.declare_parameter('local_odom_topic', '/odometry/local')  # ekf_local 출력
        self.declare_parameter('output_topic', '/heading/imu_uwb_fused')
        self.declare_parameter('output_frame_id', 'base_link')
        self.declare_parameter('map_frame_id', 'map')
        # 이름 주의: 발행되는 orientation_covariance[8](yaw '각도' 분산)에 들어가는 값.
        self.declare_parameter('yaw_variance', 0.01)
        # 캘리브레이션(TF) 회전이 이 값(rad) 이상 바뀌면 yaw 재초기화.
        # 기본 2도: UWB 노이즈로 인한 미세 재캘리브레이션 차이는 무시하고,
        # 앵커 재배치급 변화만 감지.
        self.declare_parameter('recalib_yaw_reset_threshold_rad', math.radians(2.0))
        self.declare_parameter('tf_timeout_s', 0.1)

        self.alpha = self.get_parameter('alpha').value
        self.min_move = self.get_parameter('min_move_distance_m').value
        self.output_frame_id = self.get_parameter('output_frame_id').value
        self.map_frame_id = self.get_parameter('map_frame_id').value
        self.recalib_threshold = self.get_parameter('recalib_yaw_reset_threshold_rad').value
        self.tf_timeout = Duration(seconds=self.get_parameter('tf_timeout_s').value)

        # ---- 상태 ----
        self.yaw_est = None            # 융합된 현재 yaw 추정치 (rad)
        self.last_imu_stamp = None
        self.last_uwb_xy = None        # map 프레임으로 변환된 좌표 저장
        self.last_uwb_stamp = None
        self.is_reverse = False        # ekf_local 선속도 부호로 판별
        self.last_tf_yaw = None        # 마지막으로 사용한 map<-uwb_frame TF의 회전각
        self._tf_warn_logged = False   # TF 실패 경고 도배 방지

        # ---- TF (map <- uwb_frame 변환용) ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- 구독 ----
        self.create_subscription(Imu, self.get_parameter('imu_topic').value,
                                  self._imu_cb, 50)
        self.create_subscription(PoseWithCovarianceStamped,
                                  self.get_parameter('uwb_pose_topic').value,
                                  self._uwb_cb, 10)
        self.create_subscription(Odometry,
                                  self.get_parameter('local_odom_topic').value,
                                  self._local_odom_cb, 20)

        # ---- 발행 ----
        self.pub = self.create_publisher(
            Imu, self.get_parameter('output_topic').value, 10)

        self.get_logger().info(
            f"heading_complementary_filter 시작 (alpha={self.alpha}, "
            f"min_move={self.min_move}m, TF 기반 map 좌표 변환 사용)"
        )

    # ------------------------------------------------------------------
    def _local_odom_cb(self, msg: Odometry):
        vx = msg.twist.twist.linear.x
        # 부호 기반 전진/후진 판별. 데드밴드를 둬서 정지시 잦은 토글 방지.
        if vx > 0.02:
            self.is_reverse = False
        elif vx < -0.02:
            self.is_reverse = True
        # |vx| <= 0.02 이면 이전 상태 유지

    # ------------------------------------------------------------------
    def _imu_cb(self, msg: Imu):
        now = self._stamp_to_sec(msg.header.stamp)

        gyro_z = msg.angular_velocity.z  # rad/s

        if self.last_imu_stamp is None:
            self.last_imu_stamp = now
            return

        dt = now - self.last_imu_stamp
        self.last_imu_stamp = now
        if dt <= 0.0 or dt > 1.0:
            # 비정상 dt (타임점프 등) 방어
            return

        # 중요: yaw_est는 최초 UWB course-over-ground로만 초기화된다.
        # 초기화 전에는 절대 임의값(0 등)으로 시작하지 않고, 발행도 하지 않는다.
        # (초기화 전에 yaw=0을 발행하면 ekf_global에 틀린 절대방향이 주입됨)
        if self.yaw_est is None:
            return

        # 자이로 적분 (빠른 성분)
        self.yaw_est = wrap_to_pi(self.yaw_est + gyro_z * dt)

        self._publish(msg.header.stamp)

    # ------------------------------------------------------------------
    def _transform_uwb_to_map(self, msg: PoseWithCovarianceStamped):
        """
        UWB pose(uwb_frame)를 map 프레임 좌표로 변환.
        반환: (map_x, map_y, tf_yaw) 또는 None (TF 미가용 시).

        uwb_map_calibration이 map -> uwb_frame 정적 TF를 발행한다
        (노드 시작 시 항등변환, 캘리브레이션 완료 시 실제 변환으로 갱신).
        """
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame_id, msg.header.frame_id,
                Time(),  # 정적 TF이므로 최신값 사용
                timeout=self.tf_timeout)
        except Exception as e:
            # TF 미가용: 원시 uwb_frame 좌표를 map인 척 쓰면 안 되므로 스킵.
            # (과거 버그가 정확히 그 동작이었음 — 편향된 절대 yaw 주입)
            if not self._tf_warn_logged:
                self.get_logger().warn(
                    f"map<-{msg.header.frame_id} TF 조회 실패 — UWB course 계산 보류. "
                    f"uwb_map_calibration 노드가 떠 있는지 확인하세요. ({e})")
                self._tf_warn_logged = True
            return None
        self._tf_warn_logged = False

        q = t.transform.rotation
        tf_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        tx = t.transform.translation.x
        ty = t.transform.translation.y

        # map 좌표 = R(tf_yaw) * uwb 좌표 + T
        # (lookup_transform(target=map, source=uwb_frame)은 source 좌표를
        #  target 좌표로 옮기는 변환을 반환하므로 그대로 순방향 적용)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        c, s = math.cos(tf_yaw), math.sin(tf_yaw)
        map_x = c * x - s * y + tx
        map_y = s * x + c * y + ty
        return map_x, map_y, tf_yaw

    # ------------------------------------------------------------------
    def _uwb_cb(self, msg: PoseWithCovarianceStamped):
        result = self._transform_uwb_to_map(msg)
        if result is None:
            return
        x, y, tf_yaw = result
        now = self._stamp_to_sec(msg.header.stamp)

        # --- 재캘리브레이션 감지: TF 회전이 크게 바뀌면 yaw 재초기화 ---
        # 기존 yaw_est와 last_uwb_xy는 '옛 변환' 기준의 값이라 새 좌표계와
        # 섞으면 안 된다. 통째로 리셋하고 새 좌표계에서 다시 초기화.
        if self.last_tf_yaw is not None:
            tf_change = abs(wrap_to_pi(tf_yaw - self.last_tf_yaw))
            if tf_change > self.recalib_threshold:
                self.get_logger().info(
                    f"캘리브레이션 변경 감지 (TF 회전 {math.degrees(tf_change):.1f}deg 변화) "
                    "→ yaw 재초기화 (다음 이동 구간에서 새 좌표계 기준으로 재탄생)")
                self.yaw_est = None
                self.last_uwb_xy = None
                self.last_uwb_stamp = None
        self.last_tf_yaw = tf_yaw

        if self.last_uwb_xy is None:
            self.last_uwb_xy = (x, y)
            self.last_uwb_stamp = now
            return

        dx = x - self.last_uwb_xy[0]
        dy = y - self.last_uwb_xy[1]
        dist = math.hypot(dx, dy)

        # 최소 이동거리 미만이면 course-over-ground가 노이즈에 지배됨 -> 스킵
        if dist < self.min_move:
            return

        course = math.atan2(dy, dx)  # map 프레임에서의 진행 방향 (전진 기준)

        if self.is_reverse:
            # 후진 중이면 진행 벡터 방향은 로봇 heading과 180도 반대
            course = wrap_to_pi(course + math.pi)

        if self.yaw_est is None:
            # 최초 초기화: 이 시점부터 필터가 살아나고 발행이 시작됨
            self.yaw_est = course
            self.get_logger().info(
                f"yaw 초기화 완료 (UWB course 기반, map 프레임): {math.degrees(course):.1f}deg"
            )
        else:
            # 원형 상보필터 보정: 각도 차이를 [-pi, pi]로 wrap한 뒤 (1-alpha) 만큼 반영
            error = wrap_to_pi(course - self.yaw_est)
            self.yaw_est = wrap_to_pi(self.yaw_est + (1.0 - self.alpha) * error)

        self.last_uwb_xy = (x, y)
        self.last_uwb_stamp = now

    # ------------------------------------------------------------------
    def _publish(self, stamp):
        if self.yaw_est is None:
            return

        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self.output_frame_id

        qx, qy, qz, qw = yaw_to_quaternion(self.yaw_est)
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        # orientation-only 메시지: 각속도/선가속도는 무효 처리 (covariance[0] = -1 관례)
        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0

        # yaw covariance만 유효값
        cov = [0.0] * 9
        cov[8] = self.get_parameter('yaw_variance').value
        # roll/pitch는 미사용 -> 매우 크게 하여 EKF가 무시하도록
        cov[0] = 999999.0
        cov[4] = 999999.0
        msg.orientation_covariance = cov

        self.pub.publish(msg)

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return stamp.sec + stamp.nanosec * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = HeadingComplementaryFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
