"""
rigid_transform_2d.py
----------------------
두 점 집합(대응점 쌍) 사이의 2D 강체변환(회전+평행이동, scale 없음)을
SVD 기반 Kabsch/Umeyama 방법으로 구한다. ROS에 의존하지 않아 순수 단위테스트가 가능하다.

이 모듈은 slam_map_alignment의 핵심 수학 로직이다:
  src_points: slam_map 프레임에서 관측된 (x, y) 점들
  dst_points: map 프레임(UWB, 이미 uwb_map_calibration으로 보정됨)에서 관측된 같은 순간의 (x, y) 점들
  구하는 것: slam_map -> map 변환 (theta, tx, ty)
    dst = R(theta) @ src + t

RANSAC을 추가한 이유: UWB 잔차 오차나 시간 보간 오류로 잘못 짝지어진 대응점이
섞였을 때, 단순 최소자승은 이상치에 크게 흔들린다. RANSAC은 무작위로 최소
샘플(2쌍)만 뽑아 후보 변환을 만들고, 전체 데이터 중 그 변환과 잘 맞는(inlier)
점이 가장 많은 후보를 채택한 뒤, 그 inlier들로만 다시 최소자승을 돌려 정밀화한다.
"""

import math
import random

import numpy as np


def estimate_rigid_transform_2d(src_points: np.ndarray, dst_points: np.ndarray):
    """
    Kabsch/Umeyama 알고리즘 (2D, scale=1 고정).

    src_points, dst_points: shape (N, 2), N >= 2, 대응 순서로 짝지어져 있어야 함.
    반환: (theta_rad, tx, ty) such that dst ≈ R(theta) @ src + [tx, ty]
    """
    src_points = np.asarray(src_points, dtype=float)
    dst_points = np.asarray(dst_points, dtype=float)

    if src_points.shape[0] < 2 or src_points.shape != dst_points.shape:
        raise ValueError("대응점은 최소 2쌍 이상, src/dst shape이 같아야 합니다.")

    src_centroid = src_points.mean(axis=0)
    dst_centroid = dst_points.mean(axis=0)

    src_centered = src_points - src_centroid
    dst_centered = dst_points - dst_centroid

    # H = sum(src_centered_i^T @ dst_centered_i), 2x2
    H = src_centered.T @ dst_centered

    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1.0, d])  # 반사(reflection) 방지용 보정
    R = Vt.T @ D @ U.T

    theta = math.atan2(R[1, 0], R[0, 0])
    t = dst_centroid - R @ src_centroid

    return theta, t[0], t[1]


def apply_transform(points: np.ndarray, theta: float, tx: float, ty: float) -> np.ndarray:
    points = np.asarray(points, dtype=float)
    c, s = math.cos(theta), math.sin(theta)
    R = np.array([[c, -s], [s, c]])
    return (R @ points.T).T + np.array([tx, ty])


def ransac_rigid_transform_2d(
    src_points: np.ndarray,
    dst_points: np.ndarray,
    inlier_threshold_m: float = 0.3,
    num_iterations: int = 500,
    min_inlier_ratio: float = 0.4,
    random_seed: int = 42,
):
    """
    RANSAC으로 이상치(잘못 짝지어진 대응점, UWB 튐 잔차 등)에 강건한 변환 추정.

    반환: (theta, tx, ty, inlier_mask)
      inlier_mask: shape (N,) bool 배열. 최종 inlier로 채택된 점들.
    실패 시(inlier가 min_inlier_ratio 미만) ValueError.
    """
    src_points = np.asarray(src_points, dtype=float)
    dst_points = np.asarray(dst_points, dtype=float)
    n = src_points.shape[0]

    if n < 2:
        raise ValueError("대응점이 최소 2쌍은 있어야 합니다.")

    rng = random.Random(random_seed)
    best_inliers = None
    best_count = -1

    for _ in range(num_iterations):
        idx = rng.sample(range(n), 2)
        try:
            theta, tx, ty = estimate_rigid_transform_2d(src_points[idx], dst_points[idx])
        except (ValueError, np.linalg.LinAlgError):
            continue

        transformed = apply_transform(src_points, theta, tx, ty)
        residuals = np.linalg.norm(transformed - dst_points, axis=1)
        inliers = residuals < inlier_threshold_m
        count = int(inliers.sum())

        if count > best_count:
            best_count = count
            best_inliers = inliers

    if best_inliers is None or best_count < max(2, int(min_inlier_ratio * n)):
        raise ValueError(
            f"RANSAC 실패: 충분한 inlier를 찾지 못함 (best_count={best_count}, n={n}). "
            "매핑 주행에 회전 구간이 부족하거나 UWB 잔차가 너무 큰지 확인하세요."
        )

    # inlier로만 다시 정밀 추정 (최종 정밀화)
    theta, tx, ty = estimate_rigid_transform_2d(
        src_points[best_inliers], dst_points[best_inliers]
    )
    return theta, tx, ty, best_inliers
