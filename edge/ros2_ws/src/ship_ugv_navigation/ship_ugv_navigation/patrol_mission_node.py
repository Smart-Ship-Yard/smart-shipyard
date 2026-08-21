#!/usr/bin/env python3
"""
patrol_mission_node.py — 원형 순찰 무한 순회 (Step 6)
======================================================
Nav2 는 "실행하면 돌아다니는 프로그램"이 아니다. "여기로 가"라고 지시하면
거기까지 가고 멈추는 엔진이며 순찰·반복 개념이 없다. 목표를 하나씩 계속
주는 주체가 이 노드다.

    patrol_mission_node ──navigate_to_pose──> Nav2 ──/cmd_vel──> 로봇
        (운전자)                              (자동차)

인터페이스 (nav2_작업_정리.md 7-2절 표 그대로)
-----------------------------------------------
    구독  /event/active     std_msgs/Bool    true=정지, false=주행
    액션  navigate_to_pose  Nav2             웨이포인트 하나씩
    발행  /cmd_vel          Twist            정지 시 0속도 (안전용 이중 확인)
    발행  /patrol/status    String(JSON)     상태·웨이포인트 번호

★ 경계 원칙: 이 노드는 /event/active 가 **어디서 왔는지 모른다.**
  욜로든 관제 버튼이든 수동 명령이든 true/false 만 본다.
  그래서 Step 7(event_gate_node) 없이도 완성·테스트할 수 있다.

★ BasicNavigator 를 쓰지 않은 이유
--------------------------------------
문서에는 nav2_simple_commander 의 BasicNavigator 를 쓰라고 적었지만,
그것은 **자기 자신이 Node** 이고 isTaskComplete() 안에서 자기 노드를
spin 한다. 우리 노드의 /event/active 구독은 그 사이에 처리되지 않아
**정지 명령이 늦게 먹는다.** 이벤트 정지는 안전 기능이라 지연을 허용할 수
없으므로, 노드 하나 + ActionClient + 타이머로 직접 구현했다.
동작은 동일하고 코드가 50줄쯤 늘어날 뿐이다.

상태 기계
---------
    WAIT_NAV2  Nav2 액션 서버를 기다린다
    RUNNING    웨이포인트로 이동 중
    STOPPED    /event/active=true. 목표를 취소하고 0속도를 계속 쏜다
    BLOCKED    연속 실패가 쌓였다. 같은 목표를 주기적으로 재시도한다

실패 처리 — 두 가지 실패를 구분한다
-------------------------------------
    A. 웨이포인트 지점만 막힘, 옆으로 지나갈 공간 있음
       -> 다음 목표로 건너뛰면 Nav2 가 알아서 우회한다
    B. 통로 자체가 막힘 (좁은 링)
       -> 다음 목표도 그 사람을 지나야 하므로 또 실패한다

B 에서 건너뛰기만 반복하면 로봇이 웨이포인트를 순서대로 계속 실패하며
헛돈다. 그래서 연속 실패가 max_consecutive_fails 에 이르면 BLOCKED 로
전환해 제자리에서 기다린다.

**BLOCKED 에서 빠져나오는 방법은 주기적 재시도뿐이다.** 길이 열렸는지
알려주는 신호가 없으므로 "해보고 되면 간다"가 유일한 방법이다.
간격을 5초로 둔 이유: 1초마다 재시도하면 Nav2 가 계속 경로 계획을 돌려
CPU 를 낭비하고 실패 로그가 폭주해 진짜 문제를 못 본다.

파라미터
--------
    center_x, center_y   순찰 원 중심 (check_patrol_space.py 출력값)
    radius               순찰 반지름
    num_waypoints        원 위의 목표 개수 (12 권장)
    direction            cw | ccw   (cw = 카메라가 오른쪽 90도로 배를 향함)
    start_from_nearest   현재 위치에서 가장 가까운 지점부터 시작할지
    goal_retry_count     한 웨이포인트를 몇 번 더 시도할지
    max_consecutive_fails 연속 실패가 이만큼이면 BLOCKED
    wait_retry_interval_s BLOCKED 에서 재시도 간격
    goal_timeout_s       한 목표에 이 시간을 넘기면 실패로 간주
"""

import json
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.msg import State as LifecycleState
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool, String

import tf2_ros


# 상태 이름 (문자열 그대로 /patrol/status 에 실려 나간다)
WAIT_NAV2 = 'WAIT_NAV2'
RUNNING = 'RUNNING'
STOPPED = 'STOPPED'
BLOCKED = 'BLOCKED'


def yaw_to_quat(yaw: float):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class PatrolMissionNode(Node):

    def __init__(self):
        super().__init__('patrol_mission_node')

        # ---- 파라미터 -------------------------------------------------
        p = self.declare_parameters('', [
            ('center_x', 0.0),
            ('center_y', 0.0),
            ('radius', 0.95),
            ('num_waypoints', 12),
            ('direction', 'cw'),
            ('start_from_nearest', True),
            ('resync_from_pose', True),
            ('goal_retry_count', 2),
            ('retry_delay_s', 2.0),
            ('max_consecutive_fails', 4),
            ('wait_retry_interval_s', 5.0),
            # ★ 60 -> 30 (2026-08-18 실측). 사람이 순찰 경로에 서 있을 때
            #   60초는 너무 길었다 — 로봇이 회전/대기/후진을 반복하는 동안
            #   웨이포인트가 하나도 안 넘어갔다.
            #   BT(navigate_with_spin.xml)의 재시도를 3회로 줄여 20~25초면
            #   FAILURE 가 돌아오므로, 이 값은 그게 안 돌아올 때를 위한
            #   **안전망**이다. BT 보다 살짝 길게 둬서 정상 실패 경로를
            #   가로채지 않도록 한다.
            ('goal_timeout_s', 30.0),
            ('map_frame', 'map'),
            ('base_frame', 'base_link'),
            ('nav2_lifecycle_node', 'bt_navigator'),
        ])
        g = self.get_parameter
        self.cx = g('center_x').value
        self.cy = g('center_y').value
        self.radius = g('radius').value
        self.n_wp = int(g('num_waypoints').value)
        self.cw = str(g('direction').value).lower() != 'ccw'
        self.start_nearest = bool(g('start_from_nearest').value)
        self.resync = bool(g('resync_from_pose').value)
        self.retry_count = int(g('goal_retry_count').value)
        self.retry_delay = float(g('retry_delay_s').value)
        self.max_fails = int(g('max_consecutive_fails').value)
        self.wait_interval = float(g('wait_retry_interval_s').value)
        self.goal_timeout = float(g('goal_timeout_s').value)
        self.map_frame = g('map_frame').value
        self.base_frame = g('base_frame').value
        self.lifecycle_node = g('nav2_lifecycle_node').value

        self.waypoints = self._build_waypoints()

        # ---- 통신 -----------------------------------------------------
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # ★ 액션 서버가 "존재"하는 것과 "목표를 받을 수 있는" 것은 다르다.
        #   bt_navigator 는 configure 에서 액션 서버를 만들고 activate 에서야
        #   활성화한다. 그 사이에 목표를 보내면 조용히 거부된다(실측 확인).
        #   그래서 lifecycle 상태가 ACTIVE 인지 직접 물어본 뒤 시작한다.
        self.state_cli = self.create_client(
            GetState, f'/{self.lifecycle_node}/get_state')
        self._state_future = None
        self._state_future_sent_at = None

        self.create_subscription(Bool, '/event/active', self._event_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/patrol/status', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- 상태 -----------------------------------------------------
        self.state = WAIT_NAV2
        self.idx = 0                 # 현재 목표 웨이포인트 번호
        self.attempt = 0             # 현재 웨이포인트 재시도 횟수
        self.consecutive_fails = 0   # 연속으로 실패한 웨이포인트 수
        self.laps = 0
        self.goal_handle = None
        self.result_future = None
        self.goal_sent_at = None
        self.next_retry_at = None
        self.event_active = False
        self._start_index_fixed = not self.start_nearest

        # 상태 기계는 5 Hz 면 충분하다. 목표 하나가 수 초~수십 초 걸린다.
        self.create_timer(0.2, self._tick)
        # 정지 중에는 0속도를 계속 쏜다 (아래 _stop_robot 주석 참고)
        self.create_timer(0.1, self._hold_stop)
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            f'순찰 준비: 중심({self.cx:.3f}, {self.cy:.3f}) 반지름 {self.radius:.2f} m, '
            f'웨이포인트 {self.n_wp}개, 방향 {"시계" if self.cw else "반시계"}')

    # ------------------------------------------------------------------
    def _build_waypoints(self):
        """원 위에 균등 분포. 각 지점의 목표 방향은 진행 접선으로 준다.

        접선 방향을 주는 이유: 목표 방향을 아무렇게나 주면 도착할 때마다
        제자리 회전이 필요해진다. 우리 로봇은 회전에 옆으로 0.259 m 를
        더 요구하므로 좁은 곳에서 그 회전이 실패한다.
        """
        sign = -1.0 if self.cw else 1.0
        wps = []
        for i in range(self.n_wp):
            th = sign * 2.0 * math.pi * i / self.n_wp
            x = self.cx + self.radius * math.cos(th)
            y = self.cy + self.radius * math.sin(th)
            yaw = th + sign * (math.pi / 2.0)   # 진행 접선
            wps.append((x, y, yaw))
        return wps

    def _robot_angle(self):
        """로봇이 원 중심에서 어느 각도에 있는지. 실패하면 None."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.0))
        except Exception:
            return None
        dx = tf.transform.translation.x - self.cx
        dy = tf.transform.translation.y - self.cy
        return math.atan2(dy, dx)

    def _index_ahead(self):
        """현재 위치의 각도를 웨이포인트 번호로 환산해 **진행 방향 한 칸 앞**을
        돌려준다. 위치를 모르면 None.

        가장 가까운 지점 자체를 목표로 주면, 로봇이 이미 그 옆을 지나쳤을 때
        되돌아가려고 크게 방향을 틀게 된다. 한 칸 앞을 주면 항상 앞으로만 간다.
        """
        ang = self._robot_angle()
        if ang is None:
            return None
        sign = -1.0 if self.cw else 1.0
        k = (ang / (sign * 2.0 * math.pi / self.n_wp)) % self.n_wp
        return int((math.floor(k) + 1) % self.n_wp)

    # ------------------------------------------------------------------
    def _event_cb(self, msg: Bool):
        if msg.data == self.event_active:
            return
        self.event_active = msg.data
        if self.event_active:
            self.get_logger().warn('이벤트 발생 — 순찰 정지')
            self._cancel_goal()
            self.state = STOPPED
        else:
            self.get_logger().info(f'이벤트 해제 — 순찰 재개 (웨이포인트 {self.idx})')
            # 취소된 목표를 다시 보내야 하므로 attempt 는 건드리지 않는다
            self.state = RUNNING
            self.goal_handle = None
            self.result_future = None

    def _cancel_goal(self):
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().warn(f'목표 취소 실패(무시): {e}')
        self.goal_handle = None
        self.result_future = None
        self._stop_robot()

    def _stop_robot(self):
        """0속도를 직접 쏜다.

        Nav2 를 취소하면 곧 멈추지만, 취소가 전달되는 사이 velocity_smoother
        에 남아 있던 명령이 나갈 수 있다. 이벤트 정지는 안전 기능이라
        Nav2 와 별개로 확실히 0을 보낸다. (7-2절 표에 명시된 동작)
        """
        self.cmd_pub.publish(Twist())

    def _hold_stop(self):
        if self.state == STOPPED:
            self._stop_robot()

    # ------------------------------------------------------------------
    def _nav2_is_active(self) -> bool:
        """bt_navigator 의 lifecycle 상태가 ACTIVE 인지 비동기로 확인한다.

        서비스 호출을 블로킹으로 하면 그 사이 /event/active 가 처리되지 않으므로
        future 를 들고 다니며 타이머마다 완료를 확인한다.

        ★ 2026-08-15 실측으로 잡은 버그: bt_navigator 가 Configuring 등으로
        바쁠 때 응답이 rmw 레벨에서 유실되면(`failed to send response ...
        (timeout)`) future 가 영원히 done() 이 안 된다. 재시도 로직이 없으면
        WAIT_NAV2 에서 영영 못 빠져나와 patrol_mission_node 를 수동으로
        재시작해야 했다. 일정 시간 지나도 안 끝나면 버리고 다음 틱에 새로
        요청한다.
        """
        if not self.nav.server_is_ready():
            return False
        if not self.state_cli.service_is_ready():
            return False
        if self._state_future is None:
            self._state_future = self.state_cli.call_async(GetState.Request())
            self._state_future_sent_at = self.get_clock().now()
            return False
        if not self._state_future.done():
            elapsed = (self.get_clock().now() - self._state_future_sent_at).nanoseconds / 1e9
            if elapsed > 3.0:
                self.get_logger().warn(
                    'get_state 응답 유실(3초 초과) — 재요청')
                self._state_future = None
            return False
        try:
            state_id = self._state_future.result().current_state.id
        except Exception:
            state_id = -1
        self._state_future = None      # 다음 틱에서 다시 물어본다
        return state_id == LifecycleState.PRIMARY_STATE_ACTIVE

    def _tick(self):
        if self.state == STOPPED:
            return

        if self.state == WAIT_NAV2:
            if self._nav2_is_active():
                self.get_logger().info('Nav2 활성 확인 — 순찰 시작')
                if not self._start_index_fixed:
                    self.idx = self._index_ahead() or 0
                    self._start_index_fixed = True
                    self.get_logger().info(f'현재 위치 기준 -> 웨이포인트 {self.idx} 부터')
                self.state = RUNNING
            return

        if self.state == BLOCKED:
            now = self.get_clock().now()
            if self.next_retry_at is not None and now < self.next_retry_at:
                return
            self.get_logger().info(f'BLOCKED 재시도 — 웨이포인트 {self.idx}')
            self.state = RUNNING
            # 아래로 흘러가 목표를 다시 보낸다

        # ---- RUNNING ---------------------------------------------------
        if self.goal_handle is None and self.result_future is None:
            # 실패 직후에는 잠깐 쉬었다 보낸다. 즉시 재시도하면 Nav2 가 상황을
            # 다시 판단할 틈이 없어 같은 실패를 순식간에 반복하고, 재시도
            # 횟수만 1초 안에 소진된다(실측 확인).
            if self.next_retry_at is not None and self.get_clock().now() < self.next_retry_at:
                return
            self._send_goal()
            return

        if self.result_future is not None and self.result_future.done():
            self._handle_result()
            return

        # 타임아웃 감시 — Nav2 가 응답 없이 붙들고 있는 경우
        if self.goal_sent_at is not None:
            elapsed = (self.get_clock().now() - self.goal_sent_at).nanoseconds / 1e9
            if elapsed > self.goal_timeout:
                self.get_logger().warn(
                    f'웨이포인트 {self.idx} 시간 초과 ({elapsed:.0f}초) — 취소')
                self._cancel_goal()
                self._on_failure()

    def _resync_index(self):
        """현재 위치를 기준으로 **진행 방향 앞쪽** 웨이포인트를 다시 고른다.

        왜 필요한가 (2026-08-12 실측으로 드러난 문제):
        도달 판정 반경이 0.15 m 라 로봇은 원을 정확히 밟지 않고 매번 조금씩
        어긋난 채 통과한다. 그 오차가 한 바퀴 동안 누적되면 원 바깥으로
        0.3 m 까지 밀려난다. 그 상태에서 "다음 번호" 웨이포인트를 고집하면
        **이미 지나친 지점으로 되돌아가라는 목표**가 되어 큰 방향 전환이
        필요해진다. 그러면 제자리 회전 -> 충돌 예측 -> 목표 실패 -> 복구가
        반복되며 로봇이 한 자리에서 굳는다(실제로 그렇게 됐다).

        현재 각도에서 앞쪽 지점을 다시 고르면 로봇은 항상 앞으로만 가고,
        밀려나든 건너뛰든 스스로 원으로 복귀한다.

        ⚠️ 단, **목표가 실제로 뒤에 있을 때만** 바꾼다.
        무조건 "앞쪽 한 칸"으로 덮어쓰면 무한 루프가 된다(실측):
        도달 판정 반경이 0.15 m 라 Nav2 는 로봇이 지점에 **닿기 직전**
        (반지름 0.95 에서 약 5도 앞)에 이미 "도달"로 처리한다. 그 상태에서
        다음 번호로 넘어가면 재동기화가 "너는 아직 그 지점 앞이다"라며
        도로 되돌려, 같은 지점을 무한히 반복한다.

        그래서 진행 방향으로 잰 목표까지의 각도가 반 바퀴를 넘을 때만
        (= 목표가 사실상 등 뒤일 때만) 앞쪽 지점으로 다시 잡는다.
        """
        if not self.resync:
            return
        ang = self._robot_angle()
        if ang is None:
            return
        sign = -1.0 if self.cw else 1.0
        target_ang = sign * 2.0 * math.pi * self.idx / self.n_wp
        # 진행 방향으로 잰 남은 각도를 [0, 2pi) 로 정규화
        delta = (sign * (target_ang - ang)) % (2.0 * math.pi)
        if delta <= math.pi:
            return                      # 정상 — 목표가 앞에 있다
        ahead = self._index_ahead()
        if ahead is not None and ahead != self.idx:
            self.get_logger().warn(
                f'목표가 등 뒤에 있다(진행방향 {math.degrees(delta):.0f}도) — '
                f'웨이포인트 {self.idx} -> {ahead} 로 재동기화')
            self.idx = ahead

    def _send_goal(self):
        if not self.nav.server_is_ready():
            self.state = WAIT_NAV2
            return
        self._resync_index()
        x, y, yaw = self.waypoints[self.idx]
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        qz, qw = yaw_to_quat(yaw)
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.goal_sent_at = self.get_clock().now()
        send_future = self.nav.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as e:
            self.get_logger().error(f'목표 전송 실패: {e}')
            self._on_failure()
            return
        if handle is None or not handle.accepted:
            self.get_logger().warn(f'웨이포인트 {self.idx} 거부됨')
            self._on_failure()
            return
        # 정지 명령이 전송과 응답 사이에 들어온 경우
        if self.state == STOPPED:
            handle.cancel_goal_async()
            return
        self.goal_handle = handle
        self.result_future = handle.get_result_async()

    def _handle_result(self):
        try:
            status = self.result_future.result().status
        except Exception as e:
            self.get_logger().error(f'결과 수신 실패: {e}')
            status = GoalStatus.STATUS_ABORTED
        self.goal_handle = None
        self.result_future = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            self._on_success()
        elif status == GoalStatus.STATUS_CANCELED:
            pass   # 이벤트 정지로 취소된 것. 재개 시 같은 목표를 다시 보낸다
        else:
            self._on_failure()

    # ------------------------------------------------------------------
    def _on_success(self):
        self.attempt = 0
        self.consecutive_fails = 0
        self.next_retry_at = None      # 성공했으니 대기 없이 다음 목표로
        prev = self.idx
        self.idx = (self.idx + 1) % self.n_wp
        if self.idx == 0:
            self.laps += 1
            self.get_logger().info(f'★ {self.laps}바퀴 완료')
        self.get_logger().info(f'웨이포인트 {prev} 도달 -> 다음 {self.idx}')

    def _on_failure(self):
        now = self.get_clock().now()
        self.attempt += 1
        if self.attempt <= self.retry_count:
            self.get_logger().warn(
                f'웨이포인트 {self.idx} 실패 — {self.retry_delay:.0f}초 후 재시도 '
                f'{self.attempt}/{self.retry_count}')
            self.next_retry_at = now + Duration(seconds=self.retry_delay)
            return

        # 이 웨이포인트는 포기하고 다음으로 건너뛴다
        self.attempt = 0
        self.consecutive_fails += 1
        self.get_logger().warn(
            f'웨이포인트 {self.idx} 포기 — 건너뜀 '
            f'(연속 실패 {self.consecutive_fails}/{self.max_fails})')
        self.idx = (self.idx + 1) % self.n_wp

        if self.consecutive_fails >= self.max_fails:
            self.get_logger().error(
                f'연속 {self.consecutive_fails}회 실패 — 길이 막힌 것으로 판단. '
                f'{self.wait_interval:.0f}초마다 재시도하며 대기')
            self.state = BLOCKED
            self.consecutive_fails = 0
            self._stop_robot()
            self.next_retry_at = now + Duration(seconds=self.wait_interval)
        else:
            self.next_retry_at = now + Duration(seconds=self.retry_delay)

    # ------------------------------------------------------------------
    def _publish_status(self):
        msg = String()
        msg.data = json.dumps({
            'state': self.state,
            'waypoint': self.idx,
            'total': self.n_wp,
            'laps': self.laps,
            'attempt': self.attempt,
            'consecutive_fails': self.consecutive_fails,
            'event_active': self.event_active,
        }, ensure_ascii=False)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PatrolMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 반드시 멈춘다. 안 그러면 마지막 속도로 계속 굴러간다.
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
