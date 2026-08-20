#!/usr/bin/env python3
"""로봇을 배 옆에 세워두고 뎁스로 배 중심좌표를 1회 측정한다.

    python3 scripts/measure_ship_center.py

왜 이 방식인가
--------------
ship_survey_node 는 주행하며 여러 각도에서 표면점을 모아 사각형을 맞췄다.
점 하나하나의 오차가 크기·방향으로 증폭돼서, 실측 0.80 x 0.17 m 인 배를
1.68 x 0.35 m 로 쟀다(2026-08-20). 원리상 안 되는 방식이었다.

이 스크립트는 **모으지 않는다.** 정지 상태에서 뎁스를 몇 초 읽어 중앙값만
쓴다. 깨졌던 것은 뎁스 센서가 아니라 누적·피팅 단계였다.

방향(yaw)은 아예 재지 않는다. "로봇을 배와 나란히 세운다"는 약속이 곧
yaw = 0 이다. 크기는 줄자 실측 상수(0.80 x 0.17)를 쓴다.

놓는 법
-------
  1. 배의 긴 변과 로봇을 **나란히** 둔다 (이게 yaw = 0 을 만든다)
  2. 카메라(로봇 오른쪽)가 **배 중심**을 보도록 앞뒤로 민다
  3. 이 스크립트를 돌린다
  4. 그대로 두고 캘리브레이션 (~/calibrate + 1.5m 직진)

  ★ 3 과 4 사이에 로봇을 움직이면 안 된다. 캘리브레이션 시작 위치가
    map 원점이라, 측정도 그 자리에서 해야 좌표가 맞는다.

캘리브레이션 뒤에 재려면 --use-tf 를 준다 (map<-base_link 로 변환한다.
대신 그 시점의 UWB 오차가 그대로 들어간다).
"""
import argparse
import json
import math
import os
import statistics
import sys
import time

# ---- 카메라 장착 실측값. ship_survey_node / change_point / websocket_client 와
#      반드시 같은 값을 유지할 것. 다르면 배 위치와 이벤트 좌표가 서로 어긋난다.
CAM_OFFSET_X = 0.135      # base_link 앞으로
CAM_OFFSET_Y = -0.089     # 오른쪽 (차체 반폭)
CAM_YAW_DEG = -90.0       # 카메라가 로봇 오른쪽을 본다
SHIP_THICKNESS = 0.17     # 줄자 실측. 짧은 변
DET_TOPIC = '/event_detection/uvd'

# 측정 결과를 여기에 남긴다. finalize_map.py 와 publish_ship_pose.py 가 읽는다.
# /tmp 가 아니라 레포 안에 두는 이유: /tmp 는 재부팅이나 청소로 비워진다.
# 오늘만 두 번 날아갔고, 그때마다 측량·정합 기록을 통째로 잃었다(2026-08-20).
MEASURED_PATH = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', 'config', 'ship_center_measured.json'))


def load_measured():
    """저장된 배 중심 측정값을 읽는다. 없거나 깨졌으면 None."""
    try:
        with open(MEASURED_PATH) as f:
            d = json.load(f)
        return float(d['x']), float(d['y']), d
    except (OSError, ValueError, KeyError, TypeError):
        return None


def measured_age_text(meta):
    """언제 잰 값인지 한 줄로. 재놓고 잊은 값을 그대로 쓰는 것을 막는다."""
    ts = meta.get('timestamp')
    if not ts:
        return '측정 시각 모름'
    age = time.time() - float(ts)
    when = time.strftime('%m-%d %H:%M', time.localtime(ts))
    if age < 90:
        return f'{when} — 방금 잰 값'
    if age < 3600:
        return f'{when} — {age / 60:.0f}분 전'
    if age < 86400:
        return f'{when} — {age / 3600:.1f}시간 전  ⚠️ 그 사이 배나 로봇을 옮겼나?'
    return f'{when} — {age / 86400:.0f}일 전  ⚠️ 오래된 값이다. 다시 재는 것을 권한다'


def save_measured(x, y, extra):
    d = {'x': x, 'y': y, 'yaw_deg': 0.0, 'frame': 'map',
         'source': 'measure_ship_center.py', 'timestamp': time.time()}
    d.update(extra)
    os.makedirs(os.path.dirname(MEASURED_PATH), exist_ok=True)
    with open(MEASURED_PATH, 'w') as f:
        json.dump(d, f, indent=2)
    return MEASURED_PATH


def surface_to_center(x_cam, z_cam, half_thick,
                      cam_x=CAM_OFFSET_X, cam_y=CAM_OFFSET_Y,
                      cam_yaw_deg=CAM_YAW_DEG):
    """카메라가 본 배 표면점 -> base_link 기준 배 **중심** (x, y).

    ship_survey_node._camera_xyz_to_map_xy 와 같은 순서를 따른다.
    거기에 "표면에서 두께 절반만큼 더 들어간 곳이 중심" 을 더한다.
    """
    # 카메라 기준 전방은 z_cam, 좌측은 -x_cam (OpenCV: x=우측)
    fwd, left = z_cam, -x_cam
    c, s = math.cos(math.radians(cam_yaw_deg)), math.sin(math.radians(cam_yaw_deg))
    bx = fwd * c - left * s + cam_x
    by = fwd * s + left * c + cam_y
    # 시선 방향으로 두께 절반만큼 더 들어간다
    return bx + half_thick * c, by + half_thick * s


def self_check():
    """카메라가 정면(오른쪽) 0.5 m 앞의 표면을 봤을 때를 손으로 계산해 맞춘다."""
    # x_cam=0(화면 중앙), z_cam=0.5(전방 0.5m) -> 카메라 시선은 -y
    x, y = surface_to_center(0.0, 0.5, SHIP_THICKNESS / 2.0)
    assert abs(x - 0.135) < 1e-9, x
    assert abs(y - (-0.089 - 0.5 - 0.085)) < 1e-9, y
    # 화면 오른쪽으로 0.1 m 치우친 점이면 base_link 기준 뒤쪽(-x)으로 간다
    x2, _ = surface_to_center(0.1, 0.5, 0.0)
    assert abs(x2 - (0.135 - 0.1)) < 1e-9, x2
    print('  self-check 통과')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--seconds', type=float, default=3.0, help='읽는 시간(기본 3초)')
    ap.add_argument('--thickness', type=float, default=SHIP_THICKNESS,
                    help=f'배 짧은 변 실측(m). 기본 {SHIP_THICKNESS}')
    ap.add_argument('--class-prefix', default='level', help="배 클래스 접두사")
    ap.add_argument('--use-tf', action='store_true',
                    help='캘리브레이션 뒤에 잴 때. map<-base_link 로 변환한다')
    ap.add_argument('--self-check', action='store_true', help='계산만 검증하고 끝')
    a = ap.parse_args()

    if a.self_check:
        self_check()
        return 0

    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    rclpy.init()
    node = Node('measure_ship_center')
    samples = []

    def cb(msg):
        try:
            d = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not str(d.get('class_id', '')).startswith(a.class_prefix):
            return
        xyz = d.get('depth_xyz')
        if xyz and len(xyz) == 3 and xyz[2] > 0.05:
            samples.append((float(xyz[0]), float(xyz[2]), str(d['class_id'])))

    node.create_subscription(String, DET_TOPIC, cb, 10)
    print(f'  {DET_TOPIC} 를 {a.seconds:.0f}초 읽는다 — 로봇을 움직이지 말 것')
    end = time.time() + a.seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    if not samples:
        print()
        print('  ❌ 배를 하나도 못 봤다.')
        print(f'     · YOLO 가 떠 있나:  systemctl is-active yolo-depth-publisher')
        print(f'     · 배가 카메라(로봇 오른쪽) 화면에 들어와 있나')
        print(f'     · 클래스 접두사가 맞나 (지금 "{a.class_prefix}")')
        node.destroy_node(); rclpy.shutdown()
        return 1

    x_cam = statistics.median(s[0] for s in samples)
    z_cam = statistics.median(s[1] for s in samples)
    spread = max(s[1] for s in samples) - min(s[1] for s in samples)
    cls = statistics.mode(s[2] for s in samples)

    bx, by = surface_to_center(x_cam, z_cam, a.thickness / 2.0)

    print(f'  검출 {len(samples)}개 ({cls}), 거리 중앙값 {z_cam:.3f} m '
          f'(퍼짐 {spread * 100:.1f} cm)')
    if spread > 0.05:
        print('  ⚠️ 거리 퍼짐이 5 cm 를 넘는다 — 로봇이나 배가 흔들렸을 수 있다')
    if abs(x_cam) > 0.10:
        print(f'  ⚠️ 배가 화면 중앙에서 {abs(x_cam) * 100:.0f} cm 치우쳐 있다 '
              f'— 앞뒤로 밀어 중앙에 맞추면 더 정확하다')

    if a.use_tf:
        import tf2_ros
        from geometry_msgs.msg import PointStamped
        import tf2_geometry_msgs  # noqa: F401  (PointStamped 변환 등록)
        buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, node)
        deadline = time.time() + 3.0
        while time.time() < deadline and not buf.can_transform(
                'map', 'base_link', rclpy.time.Time()):
            rclpy.spin_once(node, timeout_sec=0.1)
        p = PointStamped()
        p.header.frame_id = 'base_link'
        p.point.x, p.point.y = bx, by
        try:
            m = buf.transform(p, 'map', timeout=rclpy.duration.Duration(seconds=1.0))
        except Exception as e:
            print(f'  ❌ TF 변환 실패: {e}')
            node.destroy_node(); rclpy.shutdown()
            return 1
        X, Y = m.point.x, m.point.y
        print(f'  map<-base_link TF 적용 (그 시점 UWB 오차가 들어간다)')
    else:
        X, Y = bx, by
        print('  로봇이 캘리브레이션 시작 위치에 있다고 본다 '
              '(base_link 좌표 = map 좌표)')

    path = save_measured(X, Y, {
        'depth_m': z_cam, 'x_cam': x_cam, 'samples': len(samples),
        'class_id': cls, 'thickness_m': a.thickness,
        'via_tf': bool(a.use_tf),
    })

    print()
    print(f'  ▶ 배 중심   X = {X:+.3f}   Y = {Y:+.3f}   (yaw 는 0 으로 고정)')
    print()
    print(f'  저장했다: config/{os.path.basename(path)}')
    print('  아래 두 줄은 이 값을 알아서 읽는다 — 손으로 옮겨 적지 않아도 된다:')
    print('    python3 scripts/finalize_map.py <맵이름>')
    print('    python3 scripts/publish_ship_pose.py')
    node.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
