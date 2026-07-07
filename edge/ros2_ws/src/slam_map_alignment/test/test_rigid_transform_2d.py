import math
import unittest

import numpy as np

from slam_map_alignment.rigid_transform_2d import (
    estimate_rigid_transform_2d,
    apply_transform,
    ransac_rigid_transform_2d,
)


def make_synthetic(theta, tx, ty, n=30, noise_std=0.0, seed=0):
    rng = np.random.RandomState(seed)
    # 회전 구간을 포함하도록 원호+직선 섞인 궤적 생성 (일직선만 있으면 조건 나쁨)
    t = np.linspace(0, 4 * math.pi, n)
    src = np.stack([t, np.sin(t) * 2.0], axis=1)
    dst = apply_transform(src, theta, tx, ty)
    if noise_std > 0:
        dst = dst + rng.normal(0, noise_std, dst.shape)
    return src, dst


class TestEstimateRigidTransform(unittest.TestCase):

    def test_exact_recovery_no_noise(self):
        true_theta = math.radians(37.0)
        true_tx, true_ty = 2.5, -1.3
        src, dst = make_synthetic(true_theta, true_tx, true_ty, noise_std=0.0)

        theta, tx, ty = estimate_rigid_transform_2d(src, dst)

        self.assertAlmostEqual(theta, true_theta, places=4)
        self.assertAlmostEqual(tx, true_tx, places=4)
        self.assertAlmostEqual(ty, true_ty, places=4)

    def test_recovery_with_small_noise(self):
        true_theta = math.radians(-90.0)
        true_tx, true_ty = 10.0, 5.0
        src, dst = make_synthetic(true_theta, true_tx, true_ty, noise_std=0.01, seed=1)

        theta, tx, ty = estimate_rigid_transform_2d(src, dst)

        self.assertAlmostEqual(theta, true_theta, places=2)
        self.assertAlmostEqual(tx, true_tx, delta=0.1)
        self.assertAlmostEqual(ty, true_ty, delta=0.1)

    def test_raises_on_insufficient_points(self):
        src = np.array([[0.0, 0.0]])
        dst = np.array([[1.0, 1.0]])
        with self.assertRaises(ValueError):
            estimate_rigid_transform_2d(src, dst)

    def test_collinear_points_still_solve_but_may_be_ambiguous(self):
        # 일직선 점들은 이론상 반사 모호성이 생길 수 있음을 문서화하는 테스트.
        # 여기서는 최소한 예외 없이 동작하는지만 확인.
        src = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]])
        dst = np.array([[0.0, 0.0], [0.0, 1.0], [0.0, 2.0]])  # 90도 회전
        theta, tx, ty = estimate_rigid_transform_2d(src, dst)
        self.assertAlmostEqual(theta, math.radians(90.0), places=3)


class TestRansacRigidTransform(unittest.TestCase):

    def test_ransac_robust_to_outliers(self):
        true_theta = math.radians(15.0)
        true_tx, true_ty = 3.0, 1.0
        src, dst = make_synthetic(true_theta, true_tx, true_ty, noise_std=0.02, seed=2)

        # 대응점 중 20%를 크게 어긋난 이상치로 오염
        n = src.shape[0]
        num_outliers = max(1, n // 5)
        rng = np.random.RandomState(99)
        outlier_idx = rng.choice(n, num_outliers, replace=False)
        dst_contaminated = dst.copy()
        dst_contaminated[outlier_idx] += rng.uniform(3.0, 5.0, size=(num_outliers, 2))

        theta, tx, ty, inlier_mask = ransac_rigid_transform_2d(
            src, dst_contaminated, inlier_threshold_m=0.3, num_iterations=300
        )

        self.assertAlmostEqual(theta, true_theta, places=2)
        self.assertAlmostEqual(tx, true_tx, delta=0.15)
        self.assertAlmostEqual(ty, true_ty, delta=0.15)

        # 오염시킨 인덱스는 inlier에서 빠져야 함
        for idx in outlier_idx:
            self.assertFalse(inlier_mask[idx])

    def test_ransac_raises_when_mostly_outliers(self):
        src = np.random.RandomState(3).uniform(-5, 5, (20, 2))
        dst = np.random.RandomState(4).uniform(-5, 5, (20, 2))  # 완전 무관한 데이터
        with self.assertRaises(ValueError):
            ransac_rigid_transform_2d(src, dst, inlier_threshold_m=0.1, num_iterations=100)


if __name__ == '__main__':
    unittest.main()
