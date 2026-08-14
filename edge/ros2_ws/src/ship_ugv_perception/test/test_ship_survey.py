#!/usr/bin/env python3
"""
test_ship_survey.py
--------------------
ship_survey_node의 측량 기하 로직(fit_rect, coverage_bin_count) 자체 검증.

ROS 없이 도는 순수 파이썬 테스트다. 이 로직이 틀리면 Nav2 keepout 마스크가
엉뚱한 자리에 생기는데, 실물 주행으로만 확인하려면 배·로봇·매핑 랩이 전부
필요해서 확인 비용이 너무 크기 때문에 여기서 먼저 막는다.

실행:  python3 test_ship_survey.py
"""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from ship_ugv_perception.ship_survey_node import (  # noqa: E402
    fit_rect, coverage_bin_count, wrap_angle)


def rect_perimeter_points(cx, cy, length, width, yaw, n_per_edge=15):
    """중심(cx,cy)에 장축 length가 yaw 방향인 사각형 둘레 위의 점들을 만든다."""
    hl, hw = length / 2.0, width / 2.0
    corners = [(-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw)]

    local = []
    for i in range(4):
        x0, y0 = corners[i]
        x1, y1 = corners[(i + 1) % 4]
        for t in np.linspace(0.0, 1.0, n_per_edge, endpoint=False):
            local.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))

    c, s = math.cos(yaw), math.sin(yaw)
    return [(cx + lx * c - ly * s, cy + lx * s + ly * c) for lx, ly in local]


def test_basic_fit():
    """정답을 아는 사각형을 넣으면 중심·방향·크기가 그대로 복원돼야 한다."""
    true_yaw = math.radians(30.0)
    pts = rect_perimeter_points(5.0, 4.0, 0.80, 0.20, true_yaw)

    cx, cy, yaw, long_side, short_side, n = fit_rect(pts, yaw_hint=true_yaw)

    assert abs(cx - 5.0) < 1e-3, f"중심 x 틀림: {cx}"
    assert abs(cy - 4.0) < 1e-3, f"중심 y 틀림: {cy}"
    assert abs(wrap_angle(yaw - true_yaw)) < 1e-3, f"yaw 틀림: {math.degrees(yaw)}"
    assert abs(long_side - 0.80) < 1e-3, f"장축 틀림: {long_side}"
    assert abs(short_side - 0.20) < 1e-3, f"단축 틀림: {short_side}"
    assert n == len(pts)
    print(f"  기본 피팅 OK: 중심=({cx:.3f},{cy:.3f}) yaw={math.degrees(yaw):.1f}deg "
          f"크기=({long_side:.3f},{short_side:.3f})")


def test_yaw_hint_resolves_180_ambiguity():
    """
    최소외접사각형은 배의 앞뒤를 구분 못 한다(장축 방향만 앎).
    yaw_hint가 반대쪽을 가리키면 결과도 반대쪽으로 뒤집혀 나와야 한다.
    이게 안 되면 프론트 3D 배 모델이 뒤집혀 배치된다.
    """
    true_yaw = math.radians(30.0)
    pts = rect_perimeter_points(5.0, 4.0, 0.80, 0.20, true_yaw)

    _, _, yaw_fwd, _, _, _ = fit_rect(pts, yaw_hint=true_yaw)
    _, _, yaw_rev, _, _, _ = fit_rect(pts, yaw_hint=wrap_angle(true_yaw + math.pi))

    assert abs(wrap_angle(yaw_fwd - true_yaw)) < 1e-3
    assert abs(wrap_angle(yaw_rev - (true_yaw + math.pi))) < 1e-3
    assert abs(abs(wrap_angle(yaw_fwd - yaw_rev)) - math.pi) < 1e-3
    print(f"  180도 모호성 해소 OK: hint에 따라 "
          f"{math.degrees(yaw_fwd):.1f}deg / {math.degrees(yaw_rev):.1f}deg")


def test_outlier_is_rejected():
    """
    minAreaRect는 볼록껍질 기반이라 먼 이상치 하나가 사각형을 통째로 터뜨린다.
    YOLO가 배경을 잘못 물은 점 하나가 들어와도 결과가 안 변해야 한다.
    """
    true_yaw = math.radians(30.0)
    pts = rect_perimeter_points(5.0, 4.0, 0.80, 0.20, true_yaw)

    clean = fit_rect(pts, yaw_hint=true_yaw)
    dirty = fit_rect(pts + [(15.0, 15.0)], yaw_hint=true_yaw)

    assert abs(dirty[3] - clean[3]) < 1e-3, f"이상치가 장축을 오염시킴: {dirty[3]}"
    assert abs(dirty[0] - clean[0]) < 1e-3 and abs(dirty[1] - clean[1]) < 1e-3
    assert dirty[5] == clean[5], "이상치가 제거되지 않음"

    # 이상치를 안 걸렀다면 어떻게 됐을지 대조 (필터가 실제로 일을 하는지 확인)
    unfiltered = fit_rect(pts + [(15.0, 15.0)], yaw_hint=true_yaw,
                          outlier_radius_m=100.0)
    assert unfiltered[3] > 5.0, "대조군이 안 터짐 - 테스트 자체가 무의미"
    print(f"  이상치 제거 OK: 필터 있음 장축={dirty[3]:.3f}m / "
          f"없음 장축={unfiltered[3]:.3f}m")


def test_dwell_bias_immunity():
    """
    한 면 앞에 오래 서 있으면 그쪽 점만 잔뜩 쌓인다.
    minAreaRect는 볼록껍질만 보므로 같은 점이 몇 번 찍히든 결과가 같아야 한다.
    (단순 평균 대신 minAreaRect를 쓰는 이유 자체를 검증)
    """
    true_yaw = math.radians(30.0)
    pts = rect_perimeter_points(5.0, 4.0, 0.80, 0.20, true_yaw)

    balanced = fit_rect(pts, yaw_hint=true_yaw)
    # 한 변(처음 15점)을 100배로 뻥튀기 = 그 면 앞에서 오래 정지한 상황
    skewed = fit_rect(pts + pts[:15] * 100, yaw_hint=true_yaw)

    assert abs(skewed[0] - balanced[0]) < 1e-3, "중심이 체류시간에 끌려감"
    assert abs(skewed[1] - balanced[1]) < 1e-3, "중심이 체류시간에 끌려감"

    # 대조: 단순 평균은 실제로 끌려간다 (minAreaRect를 택한 근거)
    mean_skewed = np.mean(np.array(pts + pts[:15] * 100), axis=0)
    assert abs(mean_skewed[0] - 5.0) > 0.05 or abs(mean_skewed[1] - 4.0) > 0.05, \
        "대조군이 안 흔들림 - 테스트 자체가 무의미"
    print(f"  체류시간 편향 면역 OK: minAreaRect 중심 불변 / "
          f"단순평균은 ({mean_skewed[0]:.3f},{mean_skewed[1]:.3f})로 이탈")


def test_coverage_detects_partial_lap():
    """한 면만 봤을 때와 한 바퀴 돌았을 때의 방위 커버리지가 구분돼야 한다."""
    pts_full = rect_perimeter_points(5.0, 4.0, 0.80, 0.20, 0.0)
    one_side = pts_full[:15]   # 한 변만

    covered_full = coverage_bin_count(pts_full, 30.0)
    covered_side = coverage_bin_count(one_side, 30.0)

    assert covered_full >= 8, f"한 바퀴인데 커버가 부족함: {covered_full}/12"
    assert covered_side < 8, f"한 면인데 커버가 과함: {covered_side}/12"
    print(f"  한바퀴 판정 OK: 전체={covered_full}/12칸, 한 면만={covered_side}/12칸")


def test_too_few_points():
    """점이 모자라면 엉뚱한 값을 내지 말고 None을 돌려줘야 한다."""
    assert fit_rect([], 0.0) is None
    assert fit_rect([(1.0, 1.0), (2.0, 2.0)], 0.0) is None
    # 이상치 제거 후 3개 미만이 되는 경우도 None
    assert fit_rect([(0.0, 0.0), (50.0, 50.0), (51.0, 51.0), (52.0, 52.0)],
                    0.0, outlier_radius_m=0.1) is None
    print("  점 부족 방어 OK")


if __name__ == '__main__':
    print("ship_survey_node 측량 기하 테스트")
    test_basic_fit()
    test_yaw_hint_resolves_180_ambiguity()
    test_outlier_is_rejected()
    test_dwell_bias_immunity()
    test_coverage_detects_partial_lap()
    test_too_few_points()
    print("전부 통과")
