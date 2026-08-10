#!/usr/bin/env python3
"""
fake_global_localization.py — 시뮬 전용
========================================
실물의 `ekf_global`이 하는 일(map -> odom TF 발행)을 시뮬에서 대신한다.

왜 필요한가
-----------
실물은 UWB로 절대 위치를 얻어 `ekf_global`이 `map -> odom`을 발행한다.
시뮬에는 UWB가 없으므로 이 TF를 만들어 줄 주체가 없고, 그러면
`map` 프레임 자체가 존재하지 않아 Nav2(코스트맵·플래너)가 동작하지 못한다.

대신 Gazebo의 참값(ground truth)을 절대 위치로 사용한다.
UWB보다 정확하지만 **TF 트리 구조는 실물과 완전히 동일**하므로,
여기서 튜닝한 Nav2 파라미터가 실물에 그대로 이식된다.

    실물:  UWB + ekf_local  --(ekf_global)-->  map -> odom
    시뮬:  Gazebo 참값       --(이 노드)  -->  map -> odom

계산 방법 (실물 ekf_global과 같은 방식)
---------------------------------------
`ekf_global.yaml` 주석에 적힌 것과 동일한 역산을 쓴다.
TF 트리는 한 프레임의 부모가 하나여야 하므로 `map -> base_link`를 직접
발행할 수 없다(base_link의 부모는 이미 odom이다). 그래서:

    map->odom  =  map->base_link(참값)  ⊗  (odom->base_link)⁻¹

`odom->base_link`는 `ekf_local`이 발행한 것을 TF로 조회한다.
따라서 이 노드는 `ekf_local`이 떠 있을 때만 동작한다(없으면 대기).

입출력
------
    구독:  /odometry/ground_truth  (nav_msgs/Odometry)
              Gazebo p3d 플러그인이 발행. world 좌표계 기준 참값.
    조회:  TF odom -> base_link    (ekf_local 발행분)
    발행:  TF map -> odom          (동적 TF)
           /odometry/global        (nav_msgs/Odometry) — 실물 ekf_global과
                                   같은 토픽명. websocket_client가 위치 핑에
                                   쓰므로 시뮬에서도 같은 이름으로 낸다.

★ 실물에는 이 노드를 실행하지 않는다 ★
    실물에서 켜면 ekf_global과 map->odom을 이중 발행해 TF 트리가 깨진다.
    `sim_bringup.launch.py`에서만 기동한다.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster


def yaw_of(q) -> float:
    """쿼터니언 -> yaw(rad). 2D 주행이라 yaw만 쓴다."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def compose(a, b):
    """2D 변환 합성: a ⊗ b. 각 변환은 (x, y, yaw)."""
    ax, ay, ath = a
    bx, by, bth = b
    c, s = math.cos(ath), math.sin(ath)
    return (ax + c * bx - s * by,
            ay + s * bx + c * by,
            ath + bth)


def invert(t):
    """2D 변환의 역변환."""
    x, y, th = t
    c, s = math.cos(-th), math.sin(-th)
    return (-(c * x - s * y), -(s * x + c * y), -th)


class FakeGlobalLocalization(Node):

    def __init__(self):
        super().__init__('fake_global_localization')

        self.declare_parameter('ground_truth_topic', '/odometry/ground_truth')
        self.declare_parameter('global_odom_topic', '/odometry/global')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        # (발행 주기 파라미터는 없앴다 — 참값 메시지 1건당 1건을 발행하므로
        #  주기가 참값 주기에 자동으로 맞고, 별도 타이머는 스탬프 중복을 만든다)
        # ★ TF 유효 시간 여유 (AMCL의 transform_tolerance와 같은 개념)
        #   TF 스탬프를 "지금"으로 찍으면, RViz나 코스트맵이 "스캔이 찍힌 시각"
        #   기준으로 map->odom을 찾을 때 미세한 시간 틈에서 조회가 실패한다.
        #   그 결과 화면이 깜빡이고 코스트맵이 끊긴다.
        #   스탬프를 조금 미래로 찍어 그 틈을 덮는다. 표준 관행이다.
        #
        #   ★ 값의 트레이드오프 (2026-08-07 실측)
        #     크게 잡으면 조회 실패는 줄지만, 그만큼 "오래된 map->odom"이
        #     현재값으로 쓰여 오차가 된다. 회전 중에는 이 영향이 특히 크다.
        #       0.1초 + 회전 0.5 rad/s -> 각도오차 약 2.9도, 위치오차 40~57 mm
        #       0.03초                 -> 각도오차 약 0.9도, 위치오차 15 mm 내외
        #     50 Hz로 발행하므로(주기 20 ms) 0.03초면 조회 실패 없이 충분하다.
        self.declare_parameter('transform_tolerance_s', 0.03)
        # Gazebo world 원점과 map 원점이 어긋나 있을 때 보정한다.
        # 맵을 world와 같은 소스(make_demo_world.py)에서 생성했다면 0으로 둔다.
        self.declare_parameter('map_offset_x', 0.0)
        self.declare_parameter('map_offset_y', 0.0)
        self.declare_parameter('map_offset_yaw', 0.0)

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.tf_tolerance = Duration(
            seconds=self.get_parameter('transform_tolerance_s').value)
        self.map_offset = (
            self.get_parameter('map_offset_x').value,
            self.get_parameter('map_offset_y').value,
            self.get_parameter('map_offset_yaw').value,
        )

        self._pending = None       # 한 건 늦춰 처리할 참값 (stamp, truth, twist)
        self._last_stamp = None    # 발행한 마지막 스탬프 (단조증가 보장용)
        self._warned = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(
            Odometry, self.get_parameter('ground_truth_topic').value,
            self._truth_cb, 20)
        self.global_pub = self.create_publisher(
            Odometry, self.get_parameter('global_odom_topic').value, 10)

        self.get_logger().info(
            'fake_global_localization 시작 (시뮬 전용). '
            f'{self.map_frame}->{self.odom_frame} 발행. '
            '실물에서는 절대 실행하지 말 것 (ekf_global과 충돌).'
        )

    # ------------------------------------------------------------------
    def _truth_cb(self, msg: Odometry):
        """참값 1건 -> TF 1건. 타이머를 쓰지 않는 이유는 아래 참고.

        ★★ 타이머 발행에서 콜백 발행으로 바꾼 이유 (2026-08-07 실측) ★★
          타이머(50 Hz)와 참값 도착(50 Hz)은 서로 동기되지 않는다. 그래서
          타이머가 두 번 도는 사이 참값이 한 번만 오면 **같은 스탬프를 두 번**
          발행하고, 예외 경로에서 now()를 쓰면 **스탬프가 거꾸로** 가기도 한다.
          실측: map->odom 42건 중 중복 25건, 시간역행 5건.
          tf2는 시간이 역행하면 캐시를 무효화하므로 RViz에
            "timestamp on the message is earlier than all the data in
             the transform cache"
          가 뜨면서 로봇 모델과 라이다 스캔이 사라졌다 나타났다 한다.

          참값 메시지 하나당 정확히 하나를 발행하면 스탬프가 항상 유일하고
          단조증가한다. 발행 주기도 참값 주기(50 Hz)에 자동으로 맞는다.

        ★ 한 건 늦춰서 처리하는 이유
          참값이 시각 T로 도착한 순간, ekf_local이 아직 T의 odom->base_link를
          발행하지 않았을 수 있다. 다음 메시지가 올 때(약 20 ms 뒤) 처리하면
          그 사이에 도착해 있어 조회가 안정적으로 성공한다.
        """
        p = msg.pose.pose
        raw = (p.position.x, p.position.y, yaw_of(p.orientation))
        pending = self._pending
        # world -> map 오프셋 적용 (기본값 0이면 그대로)
        self._pending = (Time.from_msg(msg.header.stamp),
                         compose(self.map_offset, raw),
                         msg.twist.twist)
        if pending is not None:
            self._publish(*pending)

    # ------------------------------------------------------------------
    def _publish(self, stamp, truth, twist):
        # ekf_local이 발행한 odom->base_link를 "참값과 같은 시각"으로 조회한다.
        #
        # ★ 같은 시각이어야 하는 이유
        #   map->odom = 참값 ⊗ (odom->base)⁻¹ 인데, 두 값의 시각이 다르면
        #   그 사이 로봇이 움직인 만큼이 그대로 오차가 된다. 엔코더 오도메트리는
        #   빠르게 드리프트하므로 수십 ms 차이도 수십 cm 오차가 된다.
        #   AMCL 등 표준 로컬라이제이션 노드가 모두 이 방식을 쓴다.
        #
        # ★ timeout을 0으로 두는 이유
        #   콜백 안에서 블로킹으로 기다리면, 단일 스레드 실행기라 그동안
        #   TransformListener의 /tf 수신 콜백이 실행되지 못한다. 기다릴수록
        #   오히려 버퍼가 갱신되지 않아 조회 성공률이 떨어진다(실측 80 %).
        try:
            tf = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, stamp,
                timeout=Duration(seconds=0.0))
        except Exception:
            # 이번 건은 건너뛴다. 다음 참값에서 다시 시도하면 되고,
            # 억지로 다른 시각의 값을 끌어다 쓰면 위 오차가 생긴다.
            if not self._warned:
                self.get_logger().warn(
                    f"'{self.odom_frame}'->'{self.base_frame}' TF 대기 중. "
                    'ekf_local이 떠 있고 /wheel/odom · /imu/data가 흐르는지 확인.'
                )
                self._warned = True
            return
        if self._warned:
            self.get_logger().info('odom->base_link TF 확보. 정상 동작 시작.')
            self._warned = False

        # 스탬프 단조증가 보장 (같거나 이전 시각이면 버린다).
        # tf2는 시간이 역행하면 캐시를 무효화한다.
        if self._last_stamp is not None and stamp <= self._last_stamp:
            return
        self._last_stamp = stamp

        t = tf.transform
        odom_to_base = (t.translation.x, t.translation.y, yaw_of(t.rotation))

        # map->odom = map->base_link(참값) ⊗ (odom->base_link)⁻¹
        mx, my, mth = compose(truth, invert(odom_to_base))

        # TF 스탬프에 tolerance를 더해 조금 미래까지 유효하게 만든다.
        # (RViz·코스트맵이 스캔 시각으로 조회할 때 생기는 미세한 틈을 덮는다)
        out = TransformStamped()
        out.header.stamp = (stamp + self.tf_tolerance).to_msg()
        out.header.frame_id = self.map_frame
        out.child_frame_id = self.odom_frame
        out.transform.translation.x = mx
        out.transform.translation.y = my
        out.transform.translation.z = 0.0
        out.transform.rotation.z = math.sin(mth / 2.0)
        out.transform.rotation.w = math.cos(mth / 2.0)
        self.tf_broadcaster.sendTransform(out)

        # 실물 ekf_global과 같은 토픽으로 절대 위치도 발행
        # (websocket_client가 위치 핑에 /odometry/global을 쓴다)
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.map_frame
        odom.child_frame_id = self.base_frame
        tx, ty, tth = truth
        odom.pose.pose.position.x = tx
        odom.pose.pose.position.y = ty
        odom.pose.pose.orientation.z = math.sin(tth / 2.0)
        odom.pose.pose.orientation.w = math.cos(tth / 2.0)
        odom.twist.twist = twist
        self.global_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = FakeGlobalLocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
