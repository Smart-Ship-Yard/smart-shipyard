#!/usr/bin/env python3
"""
amcl_seed_node.py
==================
AMCL 에게 초기 위치를 한 번 알려주고 스스로 종료하는 노드.

왜 필요한가
-----------
AMCL 은 "지금 로봇이 대충 어디 있는지"를 모르면 수렴하지 못한다. 기본값
(set_initial_pose: false)으로 두면 누군가 /initialpose 를 줄 때까지 원점
근처에서 헤매고, 그 상태의 /amcl_pose 를 ekf_global 이 그대로 먹으면
위치 추정이 오히려 나빠진다. RViz 의 "2D Pose Estimate" 로 사람이 매번
찍어 줄 수도 있지만, 시연에서 그걸 잊으면 그대로 사고다.

이 프로젝트에는 이미 그 초기값을 알고 있는 것이 있다 — **UWB** 다.
ekf_global 이 UWB 로 map 프레임 위치를 계속 추정하고 있으므로,
그 값을 그대로 AMCL 의 씨앗으로 넣어 주면 된다.

    UWB(절대 위치, 노이즈 있음)  ->  AMCL 초기값
    AMCL(라이다-맵 매칭, 정밀)   ->  ekf_global 의 pose1 입력

서로가 서로를 돕는 구조라 어느 쪽도 버려지지 않는다.

동작
----
1. /odometry/global (ekf_global 출력, map 프레임) 한 개를 받는다.
2. /initialpose 로 발행한다.
3. AMCL 이 아직 활성화 전일 수 있으므로 /amcl_pose 가 나올 때까지
   주기적으로 다시 보낸다 (최대 max_attempts 회).
4. 확인되면 로그를 남기고 스스로 종료한다. 계속 떠 있지 않는다.

주의: 이 노드는 "한 번 씨앗을 심는" 역할만 한다. 주행 중 위치가 틀어지는
것을 고치지는 않는다 (그건 AMCL 과 EKF 가 할 일이다).
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class AmclSeedNode(Node):

    def __init__(self):
        super().__init__('amcl_seed_node')

        self.declare_parameter('odom_topic', '/odometry/global')
        self.declare_parameter('initialpose_topic', '/initialpose')
        self.declare_parameter('amcl_pose_topic', '/amcl_pose')
        # 재시도 간격/횟수: AMCL 이 lifecycle activate 되기까지 몇 초 걸린다.
        self.declare_parameter('retry_period_s', 2.0)
        self.declare_parameter('max_attempts', 15)
        # 씨앗의 불확실성. UWB 오차(±0.15 m)보다 넉넉히 잡아 AMCL 이
        # 초기에 충분히 넓게 퍼진 파티클로 탐색하게 한다.
        self.declare_parameter('initial_cov_xy', 0.25)      # m^2
        self.declare_parameter('initial_cov_yaw', 0.20)     # rad^2

        self.max_attempts = self.get_parameter('max_attempts').value
        self.cov_xy = self.get_parameter('initial_cov_xy').value
        self.cov_yaw = self.get_parameter('initial_cov_yaw').value

        self.latest_odom = None
        self.amcl_alive = False
        self.attempts = 0

        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self._odom_cb, 10)
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter('amcl_pose_topic').value, self._amcl_cb, 10)

        self.pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.get_parameter('initialpose_topic').value, 10)

        self.timer = self.create_timer(
            self.get_parameter('retry_period_s').value, self._tick)

        self.get_logger().info(
            'amcl_seed_node 시작 — ekf_global(UWB) 추정치를 AMCL 초기값으로 넣는다')

    # ------------------------------------------------------------------
    def _odom_cb(self, msg: Odometry):
        self.latest_odom = msg

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        self.amcl_alive = True

    # ------------------------------------------------------------------
    def _tick(self):
        if self.amcl_alive:
            self.get_logger().info('AMCL 이 /amcl_pose 를 내기 시작했다 — 씨앗 심기 완료')
            self._shutdown()
            return

        if self.latest_odom is None:
            self.get_logger().warn(
                'ekf_global 추정치(/odometry/global)를 아직 못 받았다 — 대기 중')
            return

        self.attempts += 1
        if self.attempts > self.max_attempts:
            self.get_logger().error(
                f'{self.max_attempts}회 보냈는데도 /amcl_pose 가 없다. '
                'AMCL 이 떠 있는지 확인할 것. 씨앗 심기를 포기한다.')
            self._shutdown()
            return

        p = self.latest_odom.pose.pose
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose = p

        cov = [0.0] * 36
        cov[0] = self.cov_xy      # x
        cov[7] = self.cov_xy      # y
        cov[35] = self.cov_yaw    # yaw
        msg.pose.covariance = cov

        self.pub.publish(msg)

        yaw = math.atan2(2.0 * (p.orientation.w * p.orientation.z),
                         1.0 - 2.0 * (p.orientation.z ** 2))
        self.get_logger().info(
            f'초기 위치 발행 #{self.attempts}: '
            f'({p.position.x:.2f}, {p.position.y:.2f}) yaw={math.degrees(yaw):.1f}deg')

    def _shutdown(self):
        self.timer.cancel()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = AmclSeedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
