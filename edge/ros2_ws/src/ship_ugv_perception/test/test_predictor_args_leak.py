#!/usr/bin/env python3
"""ultralytics predictor 인자 누수 회귀 테스트 (2026-08-27 사고).

    python3 test/test_predictor_args_leak.py

사고: 화면에 사람이 한 번 들어오면 그 뒤로 불을 영영 못 봤다.
원인: Model 하나가 predictor 하나를 공유하고 predict() kwargs 를 그
      predictor.args 에 **누적 병합**한다. person crop 재검출이
      self.model(crops, classes=[helmet, no_helmet]) 로 부르는 순간
      그 classes 필터가 메인 추론에 눌러붙어 fire/level 이 전부 걸러졌다.
      (같은 증상을 트래커 탓으로 두 번 오진했다 — track()/predict() 와 무관하다.)
방어: 메인 추론이 classes=None, conf 를 **매번 명시**한다.
"""
import os
import sys
import numpy as np

WEIGHTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..',
                       'weights', 'best.pt')

if __name__ == '__main__':
    if not os.path.exists(WEIGHTS):
        print(f'가중치 없음 - 건너뜀: {WEIGHTS}')
        sys.exit(0)
    from ultralytics import YOLO

    model = YOLO(WEIGHTS)
    helmet_ids = [i for i, n in model.names.items()
                  if n in ('helmet', 'no_helmet')]
    assert helmet_ids, '모델에 helmet/no_helmet 이 없다'

    # 검출이 실제로 나오는 프레임이 없어도 되도록, "predictor 에 남은 필터"
    # 자체를 본다. 이게 사고의 직접 원인이고 프레임에 의존하지 않는다.
    blank = np.zeros((160, 160, 3), np.uint8)

    model.predict(blank, verbose=False, classes=None, conf=0.1)
    model(blank, verbose=False, classes=helmet_ids)      # person crop 경로
    model.predict(blank, verbose=False, classes=None, conf=0.1)   # 메인 경로

    leaked = model.predictor.args.classes
    assert leaked is None, (
        f'person crop 의 classes 필터가 메인 추론에 남았다: {leaked} '
        '-> fire/level 이 통째로 걸러진다')
    print('  person crop 이후에도 classes 필터 안 남음  OK')

    assert abs(model.predictor.args.conf - 0.1) < 1e-9, (
        f'conf 가 명시값으로 안 잡혔다: {model.predictor.args.conf} '
        '(안 주면 ultralytics 기본 0.25 라 0.1/0.15 임계값이 무시된다)')
    print('  conf 가 명시값 0.1 로 잡힘  OK')

    print('\n통과')
