#!/usr/bin/env python3
"""
event_gate_node.py — 이벤트 정지/재개 판정 (Step 7)
====================================================
"지금 멈춰야 하나"만 판단해서 `/event/active` 하나를 발행한다.
로봇을 직접 멈추지는 않는다 — 그건 patrol_mission_node 가 한다.

    [YOLO]  /event_detection/uvd ─┐
    [관제]  /server/inbound      ─┼─> event_gate_node ─> /event/active ─> 순찰 노드
    [수동]  /event/ack           ─┘

인터페이스 (nav2_작업_정리.md 7-2절 표 그대로)
-----------------------------------------------
    구독  /event_detection/uvd  std_msgs/String  욜로 감지(JSON). 위험 클래스면 정지
    구독  /server/inbound       std_msgs/String  서버 수신분(JSON). event_ack 면 재개
    구독  /event/ack            std_msgs/Empty   수동 재개 (시연 폴백·디버깅)
    발행  /event/active         std_msgs/Bool    true=정지, false=주행

★ 정지와 재개가 비대칭이다 (2026-08-07 확정 설계)
--------------------------------------------------
    정지 : 로봇 혼자, 즉시.  감지->서버->판단->명령 왕복은 지연이 붙고,
           와이파이가 끊기면 로봇이 영영 멈추지 않는다. 안전 로직은 무조건 로컬.
    재개 : 관제에서 사람이 "확인" 버튼.  화재 진화는 오래 걸리는데 그동안
           로봇이 묶여 있으면 다른 위험 상황을 놓친다.

    ⚠️ 그래서 **감지가 사라져도 자동으로 재개하지 않는다.**
       버튼의 의미는 "해결"이 아니라 "확인(ack)"이다.
       한 번 걸리면 ack 가 올 때까지 계속 true 를 유지한다(래치).

★ 히스테리시스가 없는 이유
---------------------------
욜로 발행기(yolo_depth_publisher.py)가 **track_id 로 이미 중복을 제거**한다.
`reported_tids` 집합에 들어간 객체는 다시 발행되지 않으므로,
`/event_detection/uvd` 는 연속 스트림이 아니라 **새 객체당 한 번**만 온다.
떨림을 뗄 대상이 애초에 없다. 신뢰도(0.2)도 발행기가 이미 거른다.
반복 수신이 생겨도 이미 true 이므로 무해하다.

★ 정지 대상 클래스
--------------------
모델 클래스는 4개다: fallen_person · fire · no_helmet · ship_defect
이 중 **ship_defect 는 정지 대상이 아니다.** 사람 안전과 무관하고 순찰 중
계속 나올 수 있어 멈추면 순찰이 진행되지 않는다. 기록만 하고 계속 주행한다.
(정상 상태 클래스인 person·helmet 은 발행기가 애초에 보내지 않는다)

trigger_classes 에 없는 클래스는 그냥 무시하므로, 나중에 ship_defect 를
모델에서 빼더라도 이 노드는 고칠 것이 없다.

★ /server/inbound 는 아직 없을 수 있다
----------------------------------------
젯슨 담당자가 websocket_client 에 수신 루프를 넣어야 생기는 토픽이다.
없어도 이 노드는 정상 동작한다 — 구독만 걸어두고, 그동안은 /event/ack 로
수동 재개해 전 구간을 시험할 수 있다. 나중에 배선만 살아난다.

파라미터
--------
    trigger_classes        정지시킬 클래스 목록
    min_confidence         이 값 미만이면 무시 (발행기가 이미 거르지만 이중 방어)
    republish_period_s     현재 상태를 주기적으로 재발행 (구독자가 늦게 떠도 받게)
    fallback_auto_resume_s > 0 이면 이 시간 뒤 자동 재개 (시연 폴백. 기본 꺼짐)
"""

import json

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from std_msgs.msg import Bool, Empty, String


class EventGateNode(Node):

    def __init__(self):
        super().__init__('event_gate_node')

        self.declare_parameters('', [
            ('trigger_classes', ['fire', 'fallen_person', 'no_helmet']),
            # ★ 0.2 -> 0.5 (2026-08-19). 새 YOLO 모델(fire/fallen_person/no_helmet
            #   포함)로 바꾼 뒤, **주변에 아무것도 없는데** conf 0.24~0.32 짜리
            #   fallen_person 오검출이 계속 떠서 순찰이 반복 정지했다.
            #   실측 오검출: 0.2369 / 0.2511 / 0.3209 — 0.2 로는 전부 통과한다.
            #   진짜 검출은 실측 0.55~0.83 이었으므로 0.5 면 충분히 구분된다.
            ('min_confidence', 0.5),
            # ★ 신설 (2026-08-19). 이 거리보다 먼 검출은 정지 사유로 보지 않는다.
            #   오검출들이 전부 4.0~4.2 m 에서 떴다. 순찰 반경이 0.6 m 인데
            #   4 m 밖의 물체 때문에 멈추면 시연이 진행되지 않는다.
            #   0 이하면 거리 제한을 끈다.
            ('max_trigger_depth_m', 2.5),
            ('republish_period_s', 1.0),
            ('fallback_auto_resume_s', 0.0),
            ('detection_topic', '/event_detection/uvd'),
            ('server_inbound_topic', '/server/inbound'),
        ])
        g = self.get_parameter
        self.trigger = set(g('trigger_classes').value)
        self.min_conf = float(g('min_confidence').value)
        self.max_depth = float(g('max_trigger_depth_m').value)
        self.auto_resume_s = float(g('fallback_auto_resume_s').value)
        det_topic = g('detection_topic').value
        inbound_topic = g('server_inbound_topic').value

        # ★ 늦게 뜬 구독자도 현재 상태를 받게 한다.
        #   순찰 노드가 이 노드보다 먼저/나중에 떠도 상태가 어긋나지 않도록
        #   transient_local 로 마지막 값을 남기고, 주기 재발행도 함께 한다.
        qos = QoSProfile(depth=1,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=QoSReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(Bool, '/event/active', qos)

        self.create_subscription(String, det_topic, self._detection_cb, 10)
        self.create_subscription(String, inbound_topic, self._inbound_cb, 10)
        self.create_subscription(Empty, '/event/ack', self._ack_cb, 10)

        self.active = False
        self.cause = None
        self.stopped_at = None

        self.create_timer(float(g('republish_period_s').value), self._republish)
        self._publish()

        self.get_logger().info(
            f'이벤트 게이트 시작 — 정지 대상 {sorted(self.trigger)}, '
            f'conf>={self.min_conf}, 거리<={self.max_depth}m')
        self.get_logger().info(
            f'  감지 입력 {det_topic} / 서버 입력 {inbound_topic} / 수동 재개 /event/ack')
        if self.auto_resume_s > 0:
            self.get_logger().warn(
                f'  ⚠️ 자동 재개가 켜져 있다 ({self.auto_resume_s:.0f}초). '
                f'시연 폴백용이며 평상시엔 꺼두는 것이 맞다')

    # ------------------------------------------------------------------
    def _publish(self):
        msg = Bool()
        msg.data = self.active
        self.pub.publish(msg)

    def _republish(self):
        """상태를 주기적으로 다시 알린다.

        구독자가 나중에 떠도 현재 상태를 받게 하려는 것이다. transient_local
        만으로도 대부분 해결되지만, 순찰 노드가 재시작되는 경우까지 확실히
        덮으려면 주기 재발행이 가장 단순하고 확실하다.
        """
        self._publish()
        # 시연 폴백 — 백엔드 연동이 안 됐을 때 로봇이 영영 멈춰 있지 않도록
        if self.active and self.auto_resume_s > 0 and self.stopped_at is not None:
            elapsed = (self.get_clock().now() - self.stopped_at).nanoseconds / 1e9
            if elapsed >= self.auto_resume_s:
                self.get_logger().warn(
                    f'자동 재개 ({elapsed:.0f}초 경과) — 시연 폴백 동작')
                self._resume('자동(폴백)')

    def _stop(self, cause):
        if self.active:
            return                       # 이미 정지 중 — 반복 감지는 무해
        self.active = True
        self.cause = cause
        self.stopped_at = self.get_clock().now()
        self.get_logger().warn(f'🛑 정지 — {cause}')
        self._publish()

    def _resume(self, how):
        if not self.active:
            self.get_logger().info(f'재개 요청({how})을 받았으나 이미 주행 중이다')
            return
        self.get_logger().info(f'▶️ 재개 — {how} (정지 사유: {self.cause})')
        self.active = False
        self.cause = None
        self.stopped_at = None
        self._publish()

    # ------------------------------------------------------------------
    def _detection_cb(self, msg: String):
        try:
            d = json.loads(msg.data)
        except (ValueError, TypeError):
            self.get_logger().warn(f'감지 JSON 파싱 실패(무시): {msg.data[:80]}')
            return

        # ★ 키 이름이 class_id 지만 값은 **클래스 이름 문자열**이다.
        #   (yolo_depth_publisher.py 가 model.names[cls] 를 그대로 넣는다)
        cls = d.get('class_id')
        conf = d.get('confidence', 1.0)

        if cls not in self.trigger:
            # ship_defect 등 정지 대상이 아닌 것. 기록만 하고 계속 주행한다.
            self.get_logger().info(f'감지 {cls} — 정지 대상 아님, 계속 주행')
            return
        try:
            if float(conf) < self.min_conf:
                self.get_logger().info(f'감지 {cls} conf={conf} — 임계값 미만, 무시')
                return
        except (TypeError, ValueError):
            pass

        depth = d.get('depth')

        # ★ 너무 먼 검출은 무시한다 (2026-08-19 추가).
        #   순찰 반경이 0.6 m 인데 4 m 밖 오검출로 멈추면 시연이 안 된다.
        #   진짜 위험이라면 로봇이 다가가면서 다시, 더 높은 확신도로 잡힌다.
        if (self.max_depth > 0 and isinstance(depth, (int, float))
                and depth > self.max_depth):
            self.get_logger().info(
                f'감지 {cls} {depth:.2f} m — {self.max_depth:.1f} m 밖이라 무시',
                throttle_duration_sec=5.0)
            return

        where = f' {depth:.2f} m 앞' if isinstance(depth, (int, float)) else ''
        self._stop(f'{cls} 감지{where} (conf {conf})')

    def _inbound_cb(self, msg: String):
        """서버에서 온 것을 그대로 받는다. event_ack 만 우리 관심사다."""
        try:
            d = json.loads(msg.data)
        except (ValueError, TypeError):
            return                       # 우리 것이 아닌 메시지 — 조용히 무시
        if not isinstance(d, dict):
            return
        if d.get('event_type') == 'event_ack':
            self._resume('관제 확인 버튼')

    def _ack_cb(self, _msg: Empty):
        self._resume('수동 /event/ack')


def main(args=None):
    rclpy.init(args=args)
    node = EventGateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
