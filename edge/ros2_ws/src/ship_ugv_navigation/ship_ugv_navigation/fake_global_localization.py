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
from collections import deque

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

        self._buf = deque()        # 최근 참값 (stamp, truth, twist)
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
        """참값을 버퍼에 넣고 곧바로 발행을 시도한다.

        ★★ "TF가 실제로 있는 시각"에 맞추는 이유 (2026-08-07 실측) ★★
          앞서 두 방식이 모두 실패했다.
            (1) 타이머로 발행 -> 참값 도착과 동기가 안 맞아 같은 스탬프를
                중복 발행하고 시간이 역행했다(42건 중 중복 25, 역행 5).
                tf2가 캐시를 무효화해 RViz 표시가 사라졌다.
            (2) 참값 시각으로 조회 -> ekf_local의 TF가 참값보다 약 80 ms 늦게
                도착해 "미래로 외삽 불가"로 조회가 계속 실패했다.
                map 프레임이 아예 안 생겨 스캔도 로봇도 보이지 않았다.

          그래서 **이미 도착해 있는 odom->base_link의 시각(t_tf)을 기준**으로
          삼고, 그 시각에 가장 가까운 참값을 골라 합성한다.
            - 있는 것을 쓰므로 조회가 실패할 일이 없다
            - 스탬프가 ekf_local의 발행 시각을 따라 단조증가한다
            - 참값과의 시차는 최대 참값 주기의 절반(약 10 ms)
              -> 0.15 m/s에서 1.5 mm. 무시할 수준.
        """
        p = msg.pose.pose
        self._buf.append((Time.from_msg(msg.header.stamp),
                          compose(self.map_offset,
                                  (p.position.x, p.position.y,
                                   yaw_of(p.orientation))),
                          msg.twist.twist))
        while len(self._buf) > 200:      # 약 4초분만 유지
            self._buf.popleft()
        self._publish()

    # ------------------------------------------------------------------
    def _publish(self):
        if not self._buf:
            return

        # 1) 이미 도착해 있는 odom->base_link 중 최신을 가져온다.
        #    Time()은 "버퍼에 있는 가장 최근 것"을 뜻하므로 실패하지 않는다.
        #    timeout=0: 콜백 안에서 블로킹하면 단일 스레드라 그동안 /tf 수신
        #    콜백이 못 돌아 오히려 버퍼가 갱신되지 않는다(실측 성공률 80%).
        try:
            tf = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, Time(),
                timeout=Duration(seconds=0.0))
        except Exception:
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

        t_tf = Time.from_msg(tf.header.stamp)

        # 2) 스탬프 단조증가 보장. tf2는 시간이 역행하면 캐시를 무효화한다.
        if self._last_stamp is not None and t_tf <= self._last_stamp:
            return

        # 3) t_tf에 가장 가까운 참값을 고른다 (시차 최대 약 10 ms)
        _, truth, twist = min(self._buf,
                              key=lambda e: abs((e[0] - t_tf).nanoseconds))
        self._last_stamp = t_tf

        tr = tf.transform
        odom_to_base = (tr.translation.x, tr.translation.y, yaw_of(tr.rotation))

        # map->odom = map->base_link(참값) (X) (odom->base_link)^-1
        mx, my, mth = compose(truth, invert(odom_to_base))

        # TF 스탬프에 tolerance를 더해 조금 미래까지 유효하게 만든다.
        out = TransformStamped()
        out.header.stamp = (t_tf + self.tf_tolerance).to_msg()
        out.header.frame_id = self.map_frame
        out.child_frame_id = self.odom_frame
        out.transform.translation.x = mx
        out.transform.translation.y = my
        out.transform.translation.z = 0.0
        out.transform.rotation.z = math.sin(mth / 2.0)
        out.transform.rotation.w = math.cos(mth / 2.0)
        self.tf_broadcaster.sendTransform(out)

        # 실물 ekf_global과 같은 토픽으로 절대 위치도 발행
        odom = Odometry()
        odom.header.stamp = t_tf.to_msg()
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
