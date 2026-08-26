"""실측한 배 중심좌표를 켜질 때마다 자동으로 서버에 보낸다.

왜 필요한가
-----------
배 위치는 매핑 때 한 번 실측해서 config/ship_center_measured.json 에 남긴다.
그런데 그 값을 서버로 보내는 것이 지금은 사람이 직접 치는
publish_ship_pose.py 뿐이라, 며칠 뒤에 맵만 불러와 자율주행을 돌리면
프론트엔드 화면에 배 위치가 안 간다. 그 화면은 배 pose 를 **기준 좌표계**로
써서 로봇·이벤트를 전부 변환해 그리므로, 없으면 화면 전체가 어긋난다.

그래서 이 노드가 localization.launch.py 에 같이 떠서, 저장된 값을 읽어
/ship_survey/pose 로 latch 발행한다. websocket_client 가 그것을 받아
서버로 넘기고, 서버는 MongoDB 에 저장한다.

■ 왜 latch(TRANSIENT_LOCAL) 인가
  websocket_client 가 이 노드보다 늦게 떠도 과거 값을 받아야 한다.
  둘 다 TRANSIENT_LOCAL 이라야 성립한다 (발행자만으로는 부족하다).

■ 5초마다 상기시키는 이유
  이 값이 틀리면 마스크와 관제화면이 같이 틀어지는데, 조용하면 언제 잰
  값인지 모른 채 지나간다. 언제 잰 것인지, 실제로 누가 받아갔는지 같이 찍는다.
"""
import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String


def _age_text(ts):
    if not ts:
        return '측정 시각 모름'
    age = time.time() - float(ts)
    when = time.strftime('%m-%d %H:%M', time.localtime(ts))
    if age < 3600:
        return when
    if age < 86400:
        return f'{when} ({age / 3600:.1f}시간 전)'
    return f'{when} ({age / 86400:.0f}일 전)'


class ShipPosePublisher(Node):

    def __init__(self):
        super().__init__('ship_pose_publisher')
        self.declare_parameter('measured_file', '')
        self.declare_parameter('block_id', 'B1')
        self.declare_parameter('remind_period_s', 5.0)
        self.path = self.get_parameter('measured_file').value
        self.block_id = self.get_parameter('block_id').value

        qos = QoSProfile(depth=1,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=QoSReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(String, '/ship_survey/pose', qos)

        self.data = self._load()
        if self.data:
            self.pub.publish(String(data=json.dumps(self.data['payload'])))

        self.create_timer(self.get_parameter('remind_period_s').value,
                          self._remind)

    def _load(self):
        if not self.path or not os.path.exists(self.path):
            return None
        try:
            with open(self.path) as f:
                d = json.load(f)
            x, y = float(d['x']), float(d['y'])
        except (OSError, ValueError, KeyError, TypeError) as e:
            self.get_logger().error(
                f'배 중심 측정값을 못 읽었다 ({self.path}): {e}')
            return None
        return {
            'payload': {
                'event_type': 'ship_pose', 'block_id': self.block_id,
                'map_xy': [x, y], 'yaw': 0.0,
            },
            'x': x, 'y': y, 'when': _age_text(d.get('timestamp')),
        }

    def _remind(self):
        """지금 어떤 배 위치가 서버에 올라가 있는지 5초마다 상기시킨다.

        전송 자체는 시작할 때 **딱 한 번**이다. latch(TRANSIENT_LOCAL) 라
        나중에 붙는 구독자도 그 값을 받아간다. 이 로그는 재전송이 아니라
        확인용이다 — 그 점을 문구에 적어 오해를 없앤다.

        계속 찍는 이유: 이 값이 틀리면 마스크와 관제화면이 같이 틀어지는데
        조용하면 언제 잰 값인지 모른 채 지나간다. 화면에 늘 보이면
        "아, 저 값이 올라가 있구나" 하고 안심할 수 있다.
        못 간 경우도 마찬가지로 계속 알린다 — 실제로 이 경고가
        websocket_client 가 죽어 있는 것을 잡아냈다(2026-08-20).
        """
        if self.data is None:
            self.get_logger().warn(
                '❌ 실측한 배 중심값이 없다 — 프론트엔드 화면에 배 위치가 '
                '안 간다. 매핑했으면 scripts/measure_ship_center.py 로 재고, '
                '이미 쟀으면 scripts/publish_ship_pose.py 로 한 번 보낼 것')
            return

        d = self.data
        if self.pub.get_subscription_count():
            self.get_logger().info(
                f"✅ {d['when']} 에 측정한 배 중심 "
                f"(X={d['x']:+.3f}, Y={d['y']:+.3f}) 을 서버로 보냈습니다 "
                f"— 딱 한 번 전송했을 뿐, 로그는 확인용입니다")
            return

        self.get_logger().warn(
            f"⚠️ {d['when']} 측정값을 발행했지만 받는 노드가 없다 — "
            'websocket_client 가 떠 있나?')


def main(args=None):
    rclpy.init(args=args)
    node = ShipPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
