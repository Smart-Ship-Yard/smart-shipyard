#!/usr/bin/env python3
"""
imu_axis_correction_node.py
-----------------------------
공간 제약으로 IMU(IM10A)를 차체 바닥에 뒤집어(x축 기준 180도 회전) 장착한 것을
보정하는 노드. x축(전방)은 회전축이라 그대로지만, y(좌측)와 z(상방)는 부호가
반전된 채로 센서가 값을 낸다.

[2026-07-11 실측 확인] 뒤집기 전: 정지 시 accel.z=+9.8, CCW 회전 시 gyro.z=양수
뒤집은 후: 정지 시 accel.z=-9.8, CCW 회전 시 gyro.z=-1.3(음수) 확인.
x축 기준 180도 회전 가정과 정확히 일치 (z 반전 확인됨, y는 동일 회전 기하학상
반전되는 게 맞음 - 두 테스트 모두 y가 회전/중력의 주축이 아니라 직접 실측은
안 됐으나 기하학적으로 자명함).

이 노드는 드라이버의 raw 출력(/imu/data_raw)을 구독해 y, z만 부호를 뒤집은 뒤
/imu/data로 재발행한다. 이렇게 하면 ekf_local, heading_complementary_filter 등
하위 소비자들은 IMU가 정상 장착된 것처럼 그대로 /imu/data를 쓰면 된다
(코드 수정 불필요, 보정 지점을 한 곳으로 격리).

주의: orientation(쿼터니언) 필드는 보정하지 않는다. 현재 스택에서 orientation은
아무도 쓰지 않기 때문이다 (ekf_local.yaml의 imu0_config는 roll/pitch/yaw를
전부 false로 두고, heading_complementary_filter는 gyro_z만 직접 사용한다).
나중에 orientation을 쓰게 되면 쿼터니언도 같은 180도 x축 회전으로 별도
보정해야 한다.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuAxisCorrectionNode(Node):

    def __init__(self):
        super().__init__('imu_axis_correction')

        self.declare_parameter('input_topic', '/imu/data_raw')
        self.declare_parameter('output_topic', '/imu/data')

        self.sub = self.create_subscription(
            Imu, self.get_parameter('input_topic').value,
            self._cb, 50)
        self.pub = self.create_publisher(
            Imu, self.get_parameter('output_topic').value, 50)

        self.get_logger().info(
            "imu_axis_correction 시작: x축 180도 뒤집힘 장착 보정 "
            "(y, z 부호 반전, orientation은 미보정)"
        )

    def _cb(self, msg: Imu):
        # x는 회전축이라 그대로, y/z만 부호 반전
        msg.linear_acceleration.y *= -1.0
        msg.linear_acceleration.z *= -1.0
        msg.angular_velocity.y *= -1.0
        msg.angular_velocity.z *= -1.0
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuAxisCorrectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
