#!/usr/bin/env python3
"""학습용 사진 수집 — 엔터 한 번에 한 장 (ROS 없이 카메라만 쓴다).

    python3 edge/scripts/grab_dataset.py level4
    python3 edge/scripts/grab_dataset.py level5 --out ~/datasets

엔터 = 촬영,  q + 엔터 = 종료.  이어서 하면 번호가 이어진다.

★ 왜 휴대폰 사진으로는 잘 안 되나 (2026-09-02)

같은 배를 찍어도 학습 사진과 추론 프레임이 **다른 카메라** 에서 나오면
YOLO 가 헤맨다. 눈에는 비슷해 보여도 모델이 보는 것은 다르다:

  · 휴대폰은 HDR·노이즈 제거·선명화·채도 보정을 자동으로 건다.
    이 카메라는 MJPG 를 거의 그대로 준다
  · 렌즈 화각·왜곡·해상도가 다르다
  · JPEG 압축 흔적이 다르다

그래서 이 스크립트는 yolo_depth_publisher 와 **똑같은 설정으로** 카메라를
연다. 뎁스 스트림과 정렬 모드까지 맞추는 이유는, 정렬이 컬러 프레임을
건드릴 여지를 남기지 않기 위해서다. 추론 때와 한 글자도 다르지 않은
프레임을 저장하는 것이 목적이다.

카메라가 MJPG 를 주면 **재압축 없이 원본 바이트를 그대로** 쓴다.
"""
import argparse
import os
import sys
import time

import numpy as np
from pyorbbecsdk import Pipeline, Config, OBSensorType, OBAlignMode


def open_camera():
    """yolo_depth_publisher 와 동일한 파이프라인. 순서·옵션을 바꾸지 말 것."""
    pipeline = Pipeline()
    config = Config()

    profiles = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
    config.enable_stream(profiles.get_default_video_stream_profile())

    profiles = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
    config.enable_stream(profiles.get_default_video_stream_profile())

    config.set_align_mode(OBAlignMode.SW_MODE)
    pipeline.start(config)
    return pipeline


def save(frame, path):
    """MJPG 면 원본 바이트 그대로, 아니면 품질 95 로 인코딩."""
    raw = np.frombuffer(frame.get_data(), dtype=np.uint8)
    if len(raw) > 2 and raw[0] == 0xFF and raw[1] == 0xD8:      # JPEG 매직
        with open(path, 'wb') as f:
            f.write(raw.tobytes())
        return 'raw'
    import cv2                    # MJPG 가 아닐 때만 필요하다
    img = cv2.imdecode(raw, cv2.IMREAD_COLOR)
    if img is None:
        return None
    cv2.imwrite(path, img, [cv2.IMWRITE_JPEG_QUALITY, 95])
    return 'encoded'


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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('label', help='클래스 이름 (예: level4)')
    ap.add_argument('--out', default='~/datasets', help='저장 폴더')
    ap.add_argument('--warmup', type=float, default=1.5,
                    help='자동노출이 안정될 때까지 버릴 시간(초)')
    args = ap.parse_args()

    out_dir = os.path.join(os.path.expanduser(args.out), args.label)
    os.makedirs(out_dir, exist_ok=True)

    try:
        pipeline = open_camera()
    except Exception as e:
        print(f'카메라를 못 열었다: {e}\n'
              '  이 카메라는 한 프로세스만 열 수 있다. yolo_depth_publisher 나\n'
              '  로컬라이제이션이 떠 있으면 먼저 끌 것.', file=sys.stderr)
        return 1

    # 자동노출이 잡히기 전 프레임은 색이 튄다. 잠깐 버린다.
    until = time.time() + args.warmup
    while time.time() < until:
        pipeline.wait_for_frames(100)

    n = next_index(out_dir, args.label)
    if n:
        print(f'기존 {n}장 발견 — {n}번부터 이어서 찍는다')
    print(f'저장 위치: {out_dir}')
    print('엔터 = 촬영,  q + 엔터 = 종료')

    try:
        while True:
            if input().strip().lower() == 'q':
                break

            frames = pipeline.wait_for_frames(1000)
            color = frames.get_color_frame() if frames else None
            if color is None:
                print('  프레임을 못 받았다 — 다시')
                continue

            path = os.path.join(out_dir, f'{args.label}_{n:03d}.jpg')
            how = save(color, path)
            if how is None:
                print('  디코딩 실패 — 다시')
                continue
            n += 1
            print(f'  {n}장  {os.path.basename(path)} '
                  f'({os.path.getsize(path) // 1024} KB, {how})')
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        pipeline.stop()

    print(f'\n{args.label}: 총 {n}장 — {out_dir}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
