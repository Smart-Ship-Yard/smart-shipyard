#!/usr/bin/env python3
"""
fake_rotation_odom_publisher.py
---------------------------------
Arduino/실제 바퀴 없이 motion_controller의 회전 제어 로직만 따로 검증하기 위한
가짜 로봇 시뮬레이터.

[동작 원리]
  /cmd_vel(Twist)을 구독해서, 그 안의 angular.z를 "정확히 문서화된 대로"
  적분해 가짜 yaw를 만들고 /odometry/local로 발행한다.

  적분 규칙 (REP-103, 이 프로젝트 전체가 따르는 규칙과 동일):
    yaw(t+dt) = yaw(t) + angular.z * dt
    즉 angular.z가 양수(+)면 yaw가 증가한다 (CCW = 양수).

  이건 "이상적인 가짜 로봇"이라 배선 실수나 모터 개체차, PID 발산 같은 하드웨어
  요인이 전혀 없다. 그래서 이 시뮬레이터로 회전이 정상적으로 20도에서 멈추면,
  motion_controller의 계산 로직(부호, 오차 계산, 종료 판정) 자체는 문제가
  없다는 뜻이 되고, 그러면 실제 로봇에서 겪은 문제는 하드웨어 쪽(좌우 배선,
  PID, 엔코더 등)에 있다고 좁혀서 볼 수 있다.

[사용법]
  터미널 1:
    ros2 run ship_ugv_motion_control fake_rotation_odom_publisher   (이 파일)
  터미널 2:
    ros2 launch ship_ugv_motion_control motion_control.launch.py
  터미널 3:
    ros2 topic pub --once /motion/rotate_angle std_msgs/msg/Float64 "{data: 20.0}"
  터미널 4 (지켜보기):
    ros2 topic echo /motion/status

  20도만큼 돌고 "DONE_ROTATE"로 끝나면 정상. 몇 바퀴씩 계속 돌면 이 스크립트
  자체나 motion_controller 로직에 버그가 있다는 뜻이니 그때 코드를 다시 봐야 함.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class FakeRotationOdomPublisher(Node):

    def __init__(self):
        super().__init__('fake_rotation_odom_publisher')

        self.declare_parameter('odom_topic', '/odometry/local')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('publish_rate_hz', 50.0)

        odom_topic = self.get_parameter('odom_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        rate = self.get_parameter('publish_rate_hz').value

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_v = 0.0
        self.last_w = 0.0

        self.create_subscription(Twist, cmd_vel_topic, self._cmd_vel_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 20)

        self.last_time = self.get_clock().now()
        self.create_timer(1.0 / rate, self._step)

        self.get_logger().warn(
            "★ 가짜 오도메트리 시뮬레이터 실행 중 - 실제 하드웨어 아님. "
            "실차 테스트에는 이 노드를 쓰지 말 것! "
            f"cmd_vel={cmd_vel_topic} 구독, {odom_topic} 발행"
        )

    def _cmd_vel_cb(self, msg: Twist):
        self.last_v = msg.linear.x
        self.last_w = msg.angular.z

    def _step(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt <= 0.0:
            return

        # REP-103 규칙 그대로: angular.z 양수 = CCW = yaw 증가
        self.x += self.last_v * math.cos(self.yaw) * dt
        self.y += self.last_v * math.sin(self.yaw) * dt
        self.yaw += self.last_w * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation = yaw_to_quaternion(self.yaw)
        msg.twist.twist.linear.x = self.last_v
        msg.twist.twist.angular.z = self.last_w
        self.odom_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeRotationOdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
