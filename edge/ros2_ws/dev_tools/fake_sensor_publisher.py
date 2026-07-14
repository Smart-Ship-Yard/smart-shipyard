#!/usr/bin/env python3
"""
fake_sensor_publisher.py
--------------------------
하드웨어(UWB/IMU/엔코더) 없이 실제 스택(ekf_local, ekf_global,
heading_complementary_filter, uwb_map_calibration)을 통합 테스트하기 위한
가짜 센서 발행 노드.

시나리오: 로봇이 map +x축 방향으로 3m 직진한 뒤, 왼쪽으로 90도 회전하는
궤적을 흉내낸다. 이 동안:
  - /wheel/odom  (엔코더 대역, vx/vyaw만)
  - /imu/data    (IMU 대역, angular_velocity.z / linear_acceleration만)
  - /uwb/pose    (UWB 대역, frame_id='uwb_frame', 약간의 회전+노이즈 포함)
을 실제 하드웨어와 같은 주기로 발행한다.

사용법 (터미널 3개 이상 필요):
  터미널 A: source install/setup.bash && python3 fake_sensor_publisher.py
  터미널 B: source install/setup.bash && ros2 launch ship_ugv_localization localization.launch.py
            (단, uwb_dwm1001_driver 노드만 launch에서 빼고 실행할 것 -
             가짜 노드와 실제 드라이버가 같은 /uwb/pose를 중복 발행하면 안 됨)
  터미널 C: 아래 "검증 체크리스트"의 echo/service call 명령어들

주의: uwb_map_calibration의 known_heading_in_map_rad 기본값(0.0)에 맞춰
이 스크립트는 "map +x 방향으로 직진"하는 궤적을 만든다. 실제 캘리브레이션
서비스 호출 타이밍(5초)과 이 스크립트의 직진 구간 길이가 겹치게 설계돼있다.
"""

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class FakeSensorPublisher(Node):

    def __init__(self):
        super().__init__('fake_sensor_publisher')

        self.odom_pub = self.create_publisher(Odometry, '/wheel/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.uwb_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/uwb/pose', 10)

        # ---- 궤적 파라미터 ----
        # UWB 좌표계가 map 좌표계보다 15도 돌아가 있다고 가정 (캘리브레이션이
        # 정말로 이 회전을 역산해내는지 검증하기 위해 일부러 0이 아닌 값을 둠)
        self.uwb_frame_offset_rad = math.radians(15.0)
        self.forward_speed = 0.3      # m/s, 직진 구간 속도
        self.forward_duration = 30.0   # s, 직진 지속 시간 (>= calibrate 수집시간 5s)
        self.turn_speed = 0.5         # rad/s, 회전 구간 각속도
        self.turn_duration = math.pi / 2 / self.turn_speed  # 90도 회전 소요시간

        self.start_time = time.time()
        # 실제 센서 하드웨어 주기에 맞춤: 엔코더/IMU 50~100Hz, UWB 10Hz
        self.odom_timer = self.create_timer(0.02, self._pub_odom_imu)  # 50Hz
        self.uwb_timer = self.create_timer(0.1, self._pub_uwb)         # 10Hz

        # UWB는 실측처럼 자기 자신의 (uwb_frame 기준) 위치를 내부에서 적분
        self._uwb_x, self._uwb_y, self._uwb_yaw = 0.0, 0.0, 0.0
        self._last_uwb_time = self.start_time

        self.get_logger().info(
            f"가짜 센서 발행 시작 — 직진 {self.forward_duration}s -> "
            f"90도 회전, uwb_frame이 map보다 "
            f"{math.degrees(self.uwb_frame_offset_rad):.0f}도 돌아간 것으로 시뮬레이션")

    # ------------------------------------------------------------------
    def _phase(self, t):
        """t(경과초)에 따라 (선속도, 각속도) 반환 — 직진 -> 정지 -> 회전"""
        if t < self.forward_duration:
            return self.forward_speed, 0.0
        elif t < self.forward_duration + self.turn_duration:
            return 0.0, self.turn_speed
        else:
            return 0.0, 0.0

    def _pub_odom_imu(self):
        t = time.time() - self.start_time
        v, w = self._phase(t)
        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = 'imu'
        imu.angular_velocity.z = w
        imu.linear_acceleration.x = 0.0
        imu.linear_acceleration.y = 0.0
        imu.linear_acceleration.z = 9.81  # 정지 시 중력만 (imu0_remove_gravitational_acceleration:true 라 EKF가 처리)
        self.imu_pub.publish(imu)

    def _pub_uwb(self):
        now_t = time.time()
        dt = now_t - self._last_uwb_time
        self._last_uwb_time = now_t
        t = now_t - self.start_time
        v, w = self._phase(t)

        # map 프레임 기준 진짜 위치를 dead-reckoning으로 적분
        # (이 스크립트 안에서만 쓰는 참값 궤적)
        if not hasattr(self, '_map_x'):
            self._map_x, self._map_y, self._map_yaw = 0.0, 0.0, 0.0
        self._map_x += v * math.cos(self._map_yaw) * dt
        self._map_y += v * math.sin(self._map_yaw) * dt
        self._map_yaw += w * dt

        # uwb_frame은 map보다 offset만큼 돌아가 있음 -> 참값을 역회전해서
        # "UWB가 실제로 측정할 법한" uwb_frame 좌표를 만든다
        c, s = math.cos(-self.uwb_frame_offset_rad), math.sin(-self.uwb_frame_offset_rad)
        uwb_x = c * self._map_x - s * self._map_y
        uwb_y = s * self._map_x + c * self._map_y

        # 약간의 노이즈 추가 (실측 UWB의 수 cm 지터 흉내)
        import random
        uwb_x += random.uniform(-0.02, 0.02)
        uwb_y += random.uniform(-0.02, 0.02)

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'uwb_frame'
        msg.pose.pose.position.x = uwb_x
        msg.pose.pose.position.y = uwb_y
        msg.pose.pose.orientation.w = 1.0
        cov = [0.0] * 36
        cov[0] = 0.05
        cov[7] = 0.05
        cov[14] = 999999.0
        cov[21] = 999999.0
        cov[28] = 999999.0
        cov[35] = 999999.0
        msg.pose.covariance = cov
        self.uwb_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeSensorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
