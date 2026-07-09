#!/usr/bin/env python3
"""
fake_slam_publisher.py
------------------------
slam_map_alignment 노드를 하드웨어(LiDAR/slam_toolbox) 없이 검증하기 위한
가짜 SLAM pose 발행 노드.

동작 원리: 실제로 흐르고 있는 /uwb/pose(uwb_frame)를, uwb_map_calibration이
이미 발행해둔 map<-uwb_frame TF로 변환해 "참값 map 위치"를 얻는다. 그 다음
아래 미리 정해둔 GT(ground truth) slam_map<->map 변환의 "역변환"을 적용해
"SLAM이 관측했다면 이렇게 나왔을 법한" slam_map 프레임 좌표를 역산하여
/slam_toolbox/pose로 발행한다.

전제조건: uwb_map_calibration의 ~/calibrate가 먼저 완료되어 map<-uwb_frame
TF가 존재해야 한다 (fake_sensor_publisher 실행 + calibrate 서비스 호출을
먼저 수행할 것 — 오늘 로컬라이제이션 테스트와 동일한 순서).

검증 시나리오: GT_theta=25deg, GT_tx=2.0, GT_ty=-1.0로 고정.
~/align 서비스 호출 후 slam_map_alignment 노드가 이 값을 근사로 역산해내면
(RANSAC/SVD 강체변환 로직이 실제 노드 실행 환경에서도) 성공.

사용법 (계산 없이도 기존 로컬라이제이션 스택 4~5개 노드 + fake_sensor_publisher가
이미 떠 있고, ~/calibrate가 이미 한 번 성공한 상태라고 가정):
  cd ~/smart-shipyard/edge/ros2_ws
  source install/setup.bash
  ros2 run slam_map_alignment slam_map_alignment_node &
  python3 dev_tools/fake_slam_publisher.py &
  # 로봇이 좀 더 돌아다니게 몇 초 대기 (다양한 대응점 확보 위해)
  ros2 service call /slam_map_alignment/align std_srvs/srv/Trigger
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401  (do_transform_point 등록)


# ---- Ground Truth: slam_map <-> map 변환 (align 노드가 역산해내야 하는 정답) ----
GT_THETA_DEG = 25.0
GT_TX = 2.0
GT_TY = -1.0


class FakeSlamPublisher(Node):

    def __init__(self):
        super().__init__('fake_slam_publisher')

        self.map_frame_id = 'map'
        self.gt_theta = math.radians(GT_THETA_DEG)
        self.gt_tx = GT_TX
        self.gt_ty = GT_TY

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            PoseWithCovarianceStamped, '/uwb/pose', self._uwb_cb, 50)
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, '/slam_toolbox/pose', 50)

        self._warned = False
        self.get_logger().info(
            f"가짜 SLAM 발행 시작 — GT(정답): theta={GT_THETA_DEG}deg, "
            f"tx={GT_TX}, ty={GT_TY} (slam_map<->map). "
            "map<-uwb_frame TF가 이미 있어야 함 (캘리브레이션 선행 필수)."
        )

    def _uwb_cb(self, msg: PoseWithCovarianceStamped):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame_id, msg.header.frame_id, Time())
        except Exception:
            if not self._warned:
                self.get_logger().warn(
                    "map<-uwb_frame TF 없음 — uwb_map_calibration의 "
                    "~/calibrate를 먼저 완료하세요.")
                self._warned = True
            return
        self._warned = False

        p = PointStamped()
        p.header = msg.header
        p.point.x = msg.pose.pose.position.x
        p.point.y = msg.pose.pose.position.y
        p_map = tf2_geometry_msgs.do_transform_point(p, tf)

        # 참값 map 위치 -> GT 변환의 역변환으로 "SLAM이 관측했을 법한"
        # slam_map 좌표 역산.
        # 정의: map = R(gt_theta) * slam + T  =>  slam = R(-gt_theta) * (map - T)
        mx, my = p_map.point.x, p_map.point.y
        dx, dy = mx - self.gt_tx, my - self.gt_ty
        c, s = math.cos(-self.gt_theta), math.sin(-self.gt_theta)
        slam_x = c * dx - s * dy
        slam_y = s * dx + c * dy

        out = PoseWithCovarianceStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'slam_map'
        out.pose.pose.position.x = slam_x
        out.pose.pose.position.y = slam_y
        out.pose.pose.orientation.w = 1.0
        cov = [0.0] * 36
        cov[0] = 0.01
        cov[7] = 0.01
        out.pose.covariance = cov
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = FakeSlamPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
