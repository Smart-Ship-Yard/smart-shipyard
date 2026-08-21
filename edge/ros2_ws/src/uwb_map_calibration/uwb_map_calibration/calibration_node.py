#!/usr/bin/env python3
"""
UWB <-> Map Calibration Node (ROS2)
------------------------------------
앵커가 매일 재배치되는 운영 환경을 전제로 설계됨.

목적: uwb_frame(UWB 앵커 좌표계, 원점/축이 매일 바뀜) -> map(고정 세계 좌표계)
      의 회전 + 평행이동을 구해 정적 TF(map -> uwb_frame)로 발행한다.

동작 방식
---------
1. 노드는 시작 시 IDLE 상태로 대기한다. (재시작 없이 언제든 재캘리브레이션 가능해야 하므로
   노드 자체는 계속 떠 있고, 실제 캘리브레이션 로직만 서비스 호출로 트리거됨)
2. 사용자가 ~/calibrate (std_srvs/Trigger) 서비스를 호출하면:
   a. 로봇을 정지 상태에서 알고 있는 방향(예: map 좌표계 +x 방향)으로 직진시킨다는
      전제 하에, 그 구간 동안의 /uwb/pose 샘플을 수집한다 (COLLECTING 상태).
   b. 수집된 전체 샘플에 대해 SVD 기반 총최소제곱(total least squares) 직선
      피팅으로 uwb_frame 상에서의 진행 방향 벡터를 구하고, 이를 map 좌표계
      상의 알려진 직진 방향과 비교해 회전각(theta)을 역산한다. (시작/끝 두
      점만 쓰면 그 두 샘플에 낀 UWB 잔차가 그대로 각도 오차가 되므로, 수집된
      모든 샘플을 다 활용해 노이즈에 강건하게 만든다.)
   c. 시작점을 두 좌표계의 원점 오프셋 계산에 사용해 평행이동(tx, ty)을 구한다.
   d. map -> uwb_frame 정적 TF를 발행(갱신)한다.
   e. 결과를 회차 번호를 붙여 파일로 저장한다 (재현/디버깅용).
3. 서비스는 몇 번이고 다시 호출 가능 (앵커 재배치 후 노드 재시작 불필요).

주의: 로봇이 실제로 알려진 방향(관례상 map +x축)으로 "직선으로" 주행해야 한다.
이 노드는 주행 자체를 제어하지 않는다 - 사람이 조종하거나 별도 직진 액션이
이 서비스 호출과 동시에 실행되어야 한다.
"""

import json
import math
import os
import time
from enum import Enum, auto

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster


class CalibState(Enum):
    IDLE = auto()
    COLLECTING = auto()


def yaw_to_quaternion(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class UwbMapCalibration(Node):

    def __init__(self):
        super().__init__('uwb_map_calibration')

        # ---- 파라미터 ----
        self.declare_parameter('uwb_pose_topic', '/uwb/pose')
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('uwb_frame_id', 'uwb_frame')
        self.declare_parameter('collection_duration_s', 5.0)  # 더 이상 종료 조건으로 안 씀, 하위 호환용
        # 직선 피팅 신뢰를 위한 최소 이동거리.
        # ★ 1.4m (실측, 2026-08): 캘리브레이션 시 move_distance 1.5m로 주행하는데
        #   기존 1.0m 기준에서는 실제로 약 0.85m만 쓰인 채 계산이 끝나버렸음.
        #   1.4m로 올려 1.5m 주행을 거의 다 쓰게 하니 각도오차 5.58° -> 3.00°로 개선됨.
        self.declare_parameter('min_travel_distance_m', 1.4)
        # ★ 로봇 최고속도(예: 0.15m/s)로는 고정된 5초 안에 1.0m를 못 채우는 구조적
        #   모순이 있어, 시간이 아니라 "실제 이동거리"를 종료 조건으로 바꿈.
        #   이 값은 순수 안전장치(로봇이 안 움직이거나 UWB가 끊겼을 때 무한 대기 방지)로만 씀.
        self.declare_parameter('max_collection_timeout_s', 30.0)
        self.declare_parameter('known_heading_in_map_rad', 0.0)  # 로봇이 직진한 방향 (map 기준, 보통 +x = 0)
        self.declare_parameter('result_save_dir', '/tmp/uwb_calibration_results')
        # ★ 2026-08-20 신설. 지난 캘리브레이션 결과를 그대로 되살린다.
        #   map<-uwb_frame 은 **방에 고정된 값**이다. UWB 앵커가 벽에 붙어 있고
        #   map 원점도 한 번 정하면 안 움직이므로, 로봇이 어디 있든 같은 값이다.
        #   그런데 이 값이 노드 메모리에만 있어서, 배터리가 나가면(모터와 젯슨이
        #   배터리 하나를 같이 쓴다) 같이 날아갔다. 그러면 그 map 좌표계로 만든
        #   맵까지 통째로 못 쓰게 되어 매핑을 처음부터 다시 해야 했다.
        #   -> 저장해둔 json 을 불러오면 맵을 그대로 다시 쓸 수 있다.
        self.declare_parameter('load_calibration_file', '')

        self.map_frame_id = self.get_parameter('map_frame_id').value
        self.uwb_frame_id = self.get_parameter('uwb_frame_id').value
        self.collection_duration = self.get_parameter('collection_duration_s').value
        self.min_travel = self.get_parameter('min_travel_distance_m').value
        self.max_collection_timeout = self.get_parameter('max_collection_timeout_s').value
        self.known_heading = self.get_parameter('known_heading_in_map_rad').value
        self.save_dir = self.get_parameter('result_save_dir').value
        os.makedirs(self.save_dir, exist_ok=True)

        # ---- 상태 ----
        self.state = CalibState.IDLE
        self.samples = []          # [(t, x, y), ...] 수집 중 UWB 원시 샘플
        self.collect_start_time = None
        self.calibration_count = self._load_last_index() + 1

        # 현재 유효한 변환 (theta, tx, ty). 최초 기본값은 항등변환.
        self.theta = 0.0
        self.tx = 0.0
        self.ty = 0.0
        self.loaded_from = None
        self._load_calibration(self.get_parameter('load_calibration_file').value)

        # ---- 통신 ----
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter('uwb_pose_topic').value,
            self._uwb_cb, 20)

        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_tf()  # 시작 시 항등변환으로 일단 발행 (TF tree 끊김 방지)

        self.srv = self.create_service(Trigger, '~/calibrate', self._calibrate_cb)

        # 수집 종료 감시 타이머 (서비스 콜백 블로킹 금지 → 타이머 기반 상태머신)
        self.check_timer = self.create_timer(0.2, self._check_collection_done)

        self.get_logger().info(
            "uwb_map_calibration IDLE 상태로 대기 중. "
            "'~/calibrate' 서비스 호출 시 로봇을 known_heading_in_map_rad 방향으로 "
            f"직진시키세요 (최소 {self.min_travel}m 이동 시 자동 종료, "
            f"최대 {self.max_collection_timeout}초 타임아웃)."
        )

    # ------------------------------------------------------------------
    def _load_calibration(self, path: str) -> bool:
        """저장해둔 캘리브레이션 결과를 그대로 되살린다.

        되살릴 수 있는 이유는 map<-uwb_frame 이 방에 고정된 값이기 때문이다.
        로봇의 현재 위치와 무관하므로, 전원이 나간 뒤 아무 데서나 켜도 된다.
        불러오기가 실패하면 항등변환으로 두고 평소대로 서비스를 기다린다
        (여기서 노드를 죽이면 TF tree 가 끊겨 더 나쁘다).
        """
        if not path:
            return False
        if not os.path.exists(path):
            self.get_logger().error(
                f"load_calibration_file 이 가리키는 파일이 없다: {path} — "
                "평소대로 '~/calibrate' 서비스로 새로 재야 한다")
            return False
        try:
            with open(path) as f:
                d = json.load(f)
            self.theta = float(d['published_tf_theta_rad'])
            self.tx = float(d['published_tf_tx'])
            self.ty = float(d['published_tf_ty'])
        except (OSError, ValueError, KeyError) as e:
            self.get_logger().error(
                f"캘리브레이션 파일을 못 읽었다 ({path}): {e} — "
                "평소대로 '~/calibrate' 서비스로 새로 재야 한다")
            self.theta = self.tx = self.ty = 0.0
            return False

        # 상태는 IDLE 그대로 둔다. 이 노드는 캘리브레이션을 마쳐도 IDLE 로
        # 돌아가는 구조라 '완료' 상태가 따로 없다. 불러온 뒤에도 '~/calibrate'
        # 로 언제든 다시 잴 수 있어야 한다 (앵커를 옮겼을 때).
        self.loaded_from = path
        self.get_logger().info(
            f"저장된 캘리브레이션을 불러왔다: {os.path.basename(path)}  "
            f"tx={self.tx:.4f} ty={self.ty:.4f} "
            f"theta={math.degrees(self.theta):.2f}deg")
        self.get_logger().info(
            "   이 맵으로 바로 주행할 수 있다. 1.5m 주행 캘리브레이션은 안 해도 된다. "
            "(UWB 앵커를 옮겼거나 새로 매핑할 때만 다시 잰다)")
        return True

    # ------------------------------------------------------------------
    def _load_last_index(self) -> int:
        try:
            files = [f for f in os.listdir(self.save_dir) if f.startswith('calib_')]
            if not files:
                return 0
            indices = [int(f.split('_')[1].split('.')[0]) for f in files]
            return max(indices)
        except Exception:
            return 0

    # ------------------------------------------------------------------
    def _uwb_cb(self, msg: PoseWithCovarianceStamped):
        if self.state != CalibState.COLLECTING:
            return
        t = self._stamp_to_sec(msg.header.stamp)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.samples.append((t, x, y))

    # ------------------------------------------------------------------
    def _calibrate_cb(self, request, response):
        """
        주의: 서비스 콜백 안에서 rclpy.spin_once()로 블로킹 대기하면 안 된다.
        (이미 executor가 이 콜백을 실행 중이므로 데드락/예외 발생)
        따라서 이 서비스는 수집 '시작'만 트리거하고 즉시 리턴하며,
        실제 수집 종료와 계산은 타이머 콜백(_check_collection_done)이 담당한다.
        결과는 로그와 저장 파일로 확인한다.
        """
        if self.state == CalibState.COLLECTING:
            response.success = False
            response.message = "이미 캘리브레이션 수집 중입니다."
            return response

        self.get_logger().info(
            f"캘리브레이션 시작: 최소 {self.min_travel}m 이동할 때까지 "
            "/uwb/pose 샘플을 수집합니다. 지금부터 로봇을 직진시키세요. "
            "결과는 수집 종료 후 로그/저장 파일로 확인하세요."
        )
        self.samples = []
        self.state = CalibState.COLLECTING
        self.collect_start_time = time.time()

        response.success = True
        response.message = (
            f"수집 시작됨 (최소 {self.min_travel}m 이동 시 종료). "
            "종료 후 결과는 로그와 result_save_dir 파일로 확인."
        )
        return response

    def _check_collection_done(self):
        """주기 타이머: 시간이 아니라 '실제 이동거리(min_travel)'에 도달하면 계산 수행.
        (로봇 최고속도로는 고정된 5초 안에 min_travel을 못 채우는 구조적 모순이 있어
         거리 기반으로 변경. 안 움직이거나 UWB가 끊긴 경우를 대비해 안전 타임아웃도 둠.)"""
        if self.state != CalibState.COLLECTING:
            return

        elapsed = time.time() - self.collect_start_time

        travel = 0.0
        if len(self.samples) >= 2:
            start = self.samples[0]
            end = self.samples[-1]
            travel = math.hypot(end[1] - start[1], end[2] - start[2])

        if travel >= self.min_travel:
            self.state = CalibState.IDLE
            success, message = self._compute_calibration()
            if success:
                self.get_logger().info(f"[캘리브레이션 성공] {message}")
            else:
                self.get_logger().error(f"[캘리브레이션 실패] {message}")
            return

        if elapsed >= self.max_collection_timeout:
            self.state = CalibState.IDLE
            self.get_logger().error(
                f"[캘리브레이션 실패] 타임아웃({self.max_collection_timeout}초 경과). "
                f"이동거리 {travel:.2f}m < {self.min_travel}m. "
                "로봇이 실제로 움직이고 있는지, /uwb/pose가 정상 발행되는지 확인하세요."
            )

    # ------------------------------------------------------------------
    def _compute_calibration(self):
        if len(self.samples) < 10:
            return False, f"샘플 부족 ({len(self.samples)}개). 재시도하세요."

        start = self.samples[0]
        end = self.samples[-1]
        dx = end[1] - start[1]
        dy = end[2] - start[2]
        travel = math.hypot(dx, dy)

        if travel < self.min_travel:
            return False, (
                f"이동거리 부족 ({travel:.2f}m < {self.min_travel}m). "
                "더 길게, 더 곧게 직진 후 재시도하세요."
            )

        # uwb_frame 상에서 관측된 진행 방향: 시작/끝 두 점이 아니라 수집된
        # 전체 샘플에 대한 SVD 총최소제곱 직선 피팅으로 구한다 (양 끝 샘플만
        # 쓰면 그 두 점의 UWB 잔차가 그대로 각도 오차로 들어가기 때문).
        pts = np.array([(s[1], s[2]) for s in self.samples])
        centered = pts - pts.mean(axis=0)
        _, _, vt = np.linalg.svd(centered)
        direction = vt[0]  # 주성분 방향 (부호는 ±180도 모호함)
        if direction[0] * dx + direction[1] * dy < 0:
            # 실제 주행 방향(시작->끝)과 반대로 나왔으면 뒤집어 부호를 맞춘다
            direction = -direction
        uwb_heading = math.atan2(direction[1], direction[0])

        # map 상에서는 known_heading_in_map_rad 방향으로 움직였다고 알고 있으므로
        # 회전각 theta = (map에서의 방향) - (uwb_frame에서의 방향)
        # 이 theta는 "uwb 좌표 -> map 좌표" 변환의 회전각이다.
        theta_uwb_to_map = self._wrap(self.known_heading - uwb_heading)

        # 시작점을 이용한 평행이동 계산:
        # map 좌표 = R(theta) * uwb 좌표 + T  =>  T = map_start - R(theta) * uwb_start
        # 여기서는 시작점의 map 좌표를 별도로 알 수 없으므로, 시작점을 캘리브레이션
        # 기준 원점(0,0)으로 정의하는 실용적 규약을 사용한다. (조선소 현장에서
        # 직진 시작 지점을 map 원점 부근의 알려진 기준점에 두는 운영 절차 전제)
        cos_t, sin_t = math.cos(theta_uwb_to_map), math.sin(theta_uwb_to_map)
        rotated_start_x = cos_t * start[1] - sin_t * start[2]
        rotated_start_y = sin_t * start[1] + cos_t * start[2]
        tx_uwb_to_map = 0.0 - rotated_start_x
        ty_uwb_to_map = 0.0 - rotated_start_y

        # *** TF 의미론 주의 (2026-07-08 시뮬레이션 테스트로 잡은 부호 버그) ***
        # ROS TF에서 parent=map, child=uwb_frame으로 발행하는 변환값은
        # "child(uwb_frame) 좌표를 parent(map) 좌표로 옮기는 변환"
        # (= uwb_frame 원점/자세를 map 기준으로 표현한 것)이다.
        # 즉 위에서 구한 theta_uwb_to_map / T_uwb_to_map을 *그대로* 발행해야 한다.
        # 과거 버그: "map->uwb 방향이니 역변환을 넣어야 한다"고 오해해 한 번 더
        # 뒤집어(이중 반전) 발행했고, 그 결과 heading filter와 ekf_global이
        # TF lookup으로 얻는 회전이 정확히 반대 부호가 되어 map 변환이
        # 캘리브레이션 각도의 2배만큼 틀어졌다 (시뮬레이션: 기대 0도 -> 실측 -34도).
        # 절대 다시 역변환을 넣지 말 것.
        self.theta = theta_uwb_to_map
        self.tx = tx_uwb_to_map
        self.ty = ty_uwb_to_map

        self._publish_static_tf()
        self._save_result(travel, uwb_heading, theta_uwb_to_map)

        msg = (
            f"캘리브레이션 완료 #{self.calibration_count}: "
            f"theta={math.degrees(theta_uwb_to_map):.2f}deg (uwb->map), "
            f"travel={travel:.2f}m, samples={len(self.samples)}"
        )
        self.get_logger().info(msg)
        self.calibration_count += 1
        return True, msg

    # ------------------------------------------------------------------
    def _publish_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame_id
        t.child_frame_id = self.uwb_frame_id
        t.transform.translation.x = self.tx
        t.transform.translation.y = self.ty
        t.transform.translation.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(self.theta)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

    def _save_result(self, travel, uwb_heading, theta_uwb_to_map):
        path = os.path.join(self.save_dir, f'calib_{self.calibration_count:03d}.json')
        data = {
            'timestamp': time.time(),
            'travel_distance_m': travel,
            'uwb_heading_rad': uwb_heading,
            'theta_uwb_to_map_rad': theta_uwb_to_map,
            # TF(parent=map, child=uwb_frame)에 발행된 값 = uwb->map 좌표 변환
            'published_tf_theta_rad': self.theta,
            'published_tf_tx': self.tx,
            'published_tf_ty': self.ty,
            'num_samples': len(self.samples),
        }
        with open(path, 'w') as f:
            json.dump(data, f, indent=2)
        self.get_logger().info(f"캘리브레이션 결과 저장: {path}")

    @staticmethod
    def _wrap(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return stamp.sec + stamp.nanosec * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = UwbMapCalibration()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
