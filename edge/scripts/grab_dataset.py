#!/usr/bin/env python3
"""학습용 사진 수집 — 엔터 한 번에 한 장.

    python3 edge/scripts/grab_dataset.py level4
    python3 edge/scripts/grab_dataset.py level5 --out ~/datasets

엔터 = 촬영,  q + 엔터 = 종료.  이어서 하면 번호가 이어진다.

★ 왜 카메라를 직접 열지 않고 토픽에서 받나 (2026-09-02)

이 카메라는 한 프로세스만 열 수 있다. 그런데 yolo-depth-publisher.service
가 systemd 로 항상 떠 있어서 카메라를 물고 있다
(실측: uvc_open ... already opened, res:$-6).

서비스를 끄면 찍을 수는 있지만 **웹 카메라 화면도 같이 죽는다.**
젯슨 -> video-streamer -> mediamtx -> 웹 경로가 전부 이 노드의 발행에
얹혀 있기 때문이다. 화면을 못 보고 찍으면 200장을 헛수고할 수 있다.

그래서 토픽에서 받는다. yolo_depth_publisher 가
/camera/color/compressed_raw 로 **카메라 원본 MJPG 바이트를 그대로**
내보내므로(재압축도 축소도 안 한다), 여기서 저장한 파일은 카메라를 직접
열어 찍은 것과 바이트 단위로 같다. 학습 사진이 추론 프레임과 같아야
한다는 조건도 그대로 만족한다.

덤으로 systemd 를 건드릴 일이 없고, 찍는 동안 웹 화면으로 구도를 본다.

★ 왜 휴대폰 사진으로는 잘 안 되나

팀원들이 휴대폰으로 찍어 학습시켰더니 배를 잘 못 잡았다. 눈에는 비슷해
보여도 모델이 보는 것은 다르다 — 휴대폰은 HDR·노이즈 제거·선명화·채도
보정을 자동으로 걸고, 화각과 왜곡과 JPEG 압축 흔적도 다르다. 모델이
"배" 가 아니라 "그 후처리 흔적" 을 배워버린다.
"""
import argparse
import os
import sys
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage

TOPIC = '/camera/color/compressed_raw'


def next_index(out_dir, label):
    """이어서 찍을 때 기존 번호 뒤부터."""
    n = 0
    for name in os.listdir(out_dir):
        if name.startswith(label + '_') and name.endswith('.jpg'):
            try:
                n = max(n, int(name[len(label) + 1:-4]) + 1)
            except ValueError:
                pass
    return n


def is_jpeg(data):
    return len(data) > 2 and data[0] == 0xFF and data[1] == 0xD8


class Grabber(Node):

    def __init__(self):
        super().__init__('grab_dataset')
        self._latest = None
        self._lock = threading.Lock()
        # ★ 발행자가 BEST_EFFORT 다. 기본값(RELIABLE)으로 구독하면 QoS 가
        #   안 맞아 한 장도 안 들어온다.
        self.create_subscription(
            CompressedImage, TOPIC, self._cb,
            QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                       history=QoSHistoryPolicy.KEEP_LAST, depth=1))

    def _cb(self, msg):
        with self._lock:
            self._latest = bytes(msg.data)

    def latest(self):
        with self._lock:
            return self._latest


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('label', help='클래스 이름 (예: level4)')
    ap.add_argument('--out', default='~/datasets', help='저장 폴더')
    args = ap.parse_args()

    out_dir = os.path.join(os.path.expanduser(args.out), args.label)
    os.makedirs(out_dir, exist_ok=True)

    rclpy.init()
    node = Grabber()
    # ★ rclpy.spin 을 데몬 스레드로 돌리면 종료할 때 C++ 쪽이 정리되기 전에
    #   스레드가 죽어 "terminate called without an active exception" 으로
    #   코어를 뱉는다. 사진은 멀쩡하지만 놀랄 만하다. executor 를 직접 들고
    #   먼저 세운 뒤 내려간다.
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    # 첫 프레임을 기다린다. 안 오면 원인을 알려주고 끝낸다.
    for _ in range(100):
        if node.latest() is not None:
            break
        threading.Event().wait(0.1)
    else:
        print(f'{TOPIC} 에서 프레임이 안 온다.\n'
              '  systemctl status yolo-depth-publisher 로 노드가 떠 있는지 볼 것.',
              file=sys.stderr)
        executor.shutdown()
        spin.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    n = next_index(out_dir, args.label)
    if n:
        print(f'기존 {n}장 발견 — {n}번부터 이어서 찍는다')
    print(f'저장 위치: {out_dir}')
    print('엔터 = 촬영,  q + 엔터 = 종료')

    try:
        while True:
            if input().strip().lower() == 'q':
                break

            data = node.latest()
            if not is_jpeg(data):
                print('  JPEG 가 아니다 — 건너뛴다')
                continue

            path = os.path.join(out_dir, f'{args.label}_{n:03d}.jpg')
            with open(path, 'wb') as f:
                f.write(data)
            n += 1
            print(f'  {n}장  {os.path.basename(path)} '
                  f'({len(data) // 1024} KB)')
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        executor.shutdown()
        spin.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()

    print(f'\n{args.label}: 총 {n}장 — {out_dir}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
