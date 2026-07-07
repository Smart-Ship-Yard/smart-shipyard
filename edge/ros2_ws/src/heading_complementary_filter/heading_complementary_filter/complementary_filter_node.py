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
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_to_quaternion(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


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
        # 이름 주의: 발행되는 orientation_covariance[8](yaw '각도' 분산)에 들어가는 값.
        # 과거 이름 yaw_rate_variance는 각'속도' 분산으로 오인 소지가 있어 정정함.
        self.declare_parameter('yaw_variance', 0.01)

        self.alpha = self.get_parameter('alpha').value
        self.min_move = self.get_parameter('min_move_distance_m').value
        self.output_frame_id = self.get_parameter('output_frame_id').value

        # ---- 상태 ----
        self.yaw_est = None            # 융합된 현재 yaw 추정치 (rad)
        self.last_imu_stamp = None
        self.last_uwb_xy = None
        self.last_uwb_stamp = None
        self.is_reverse = False        # ekf_local 선속도 부호로 판별

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
            f"min_move={self.min_move}m)"
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
    def _uwb_cb(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        now = self._stamp_to_sec(msg.header.stamp)

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

        course = math.atan2(dy, dx)  # UWB 상에서의 진행 방향 (전진 기준)

        if self.is_reverse:
            # 후진 중이면 진행 벡터 방향은 로봇 heading과 180도 반대
            course = wrap_to_pi(course + math.pi)

        if self.yaw_est is None:
            # 최초 초기화: 이 시점부터 필터가 살아나고 발행이 시작됨
            self.yaw_est = course
            self.get_logger().info(
                f"yaw 초기화 완료 (UWB course 기반): {math.degrees(course):.1f}deg"
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
