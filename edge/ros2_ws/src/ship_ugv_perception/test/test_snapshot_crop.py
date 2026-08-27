#!/usr/bin/env python3
"""이벤트 스냅샷 crop 검증 (ROS 없이 돌아간다).

    python3 test/test_snapshot_crop.py

핵심은 **인코딩을 매 프레임 하지 않는다**는 것이다. _stash_crop 은 자르기만
하고, 인코딩은 change_point 가 새 이벤트를 확정했을 때 _snapshot_cb 에서만
한다. 순서가 뒤집히면 초당 수십 번 인코딩하게 되어 CPU 그림이 완전히
달라진다(실측: crop 인코딩 1.28 ms, 전체화면 4.80 ms).

여기서는 자르기가 화면 밖으로 안 나가는지, 여백이 붙는지, 그리고 실제
인코딩 비용이 예상 범위인지를 본다.
"""
import os
import sys
import time
import numpy as np
import cv2

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'ship_ugv_perception'))
from yolo_depth_publisher import YoloDepthPublisher, SNAPSHOT_CLASSES  # noqa: E402


class Fake:
    """_stash_crop 이 실제로 쓰는 것만 흉내 낸다."""
    _stash_crop = YoloDepthPublisher._stash_crop

    def __init__(self, margin):
        self.snapshot_margin = margin
        self._last_crop = {}
        import threading
        self._crop_lock = threading.Lock()


if __name__ == '__main__':
    img = np.zeros((480, 640, 3), np.uint8)
    f = Fake(margin=40)

    # 화면 한가운데 — 여백이 그대로 붙는다
    f._stash_crop('fire', img, 200, 150, 300, 250)
    c = f._last_crop['fire']
    assert c.shape[:2] == (100 + 80, 100 + 80), f'여백이 안 붙었다: {c.shape}'
    print('  가운데 검출 -> 사방 40px 여백 붙음  OK')

    # 화면 모서리 — 밖으로 안 나가야 한다
    f._stash_crop('fire', img, 5, 5, 60, 60)
    c = f._last_crop['fire']
    assert c.shape[0] <= 480 and c.shape[1] <= 640
    assert c.size > 0, '모서리에서 빈 crop 이 나왔다'
    print(f'  모서리 검출 -> 화면 안으로 잘림 {c.shape[1]}x{c.shape[0]}  OK')

    # 반대쪽 모서리
    f._stash_crop('fire', img, 600, 440, 700, 520)   # 화면 밖까지 걸침
    c = f._last_crop['fire']
    assert c.size > 0 and c.shape[0] <= 480 and c.shape[1] <= 640
    print(f'  화면 밖까지 걸친 검출 -> {c.shape[1]}x{c.shape[0]}  OK')

    # 너무 작은 것은 버린다 (쓸모없는 사진)
    f._last_crop.pop('fire')
    f2 = Fake(margin=0)
    f2._stash_crop('fire', img, 10, 10, 13, 13)
    assert 'fire' not in f2._last_crop, '너무 작은 crop 을 저장했다'
    print('  8px 미만 -> 저장 안 함  OK')

    # 위험 클래스만 대상인지 (상수 확인)
    assert set(SNAPSHOT_CLASSES) == {'fire', 'fallen_person', 'no_helmet'}, \
        f'스냅샷 대상이 위험 이벤트 밖으로 넓어졌다: {SNAPSHOT_CLASSES}'
    print('  스냅샷 대상은 위험 이벤트 3종뿐  OK')

    # 인코딩 비용이 예상 범위인가 (이벤트당 1회만 하므로 이 값이면 무시 가능)
    real = (np.random.default_rng(0)
            .integers(0, 255, (250, 300, 3), dtype=np.uint8))
    t = time.perf_counter()
    for _ in range(50):
        cv2.imencode('.jpg', real, [cv2.IMWRITE_JPEG_QUALITY, 70])
    ms = (time.perf_counter() - t) / 50 * 1000
    assert ms < 15.0, f'crop 인코딩이 너무 비싸다: {ms:.1f} ms'
    print(f'  crop 인코딩 {ms:.2f} ms / 이벤트 1건  OK')

    print('\n통과')
