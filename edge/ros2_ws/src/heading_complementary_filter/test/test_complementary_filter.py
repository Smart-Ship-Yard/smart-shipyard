import math
import unittest

from heading_complementary_filter.complementary_filter_node import wrap_to_pi


class TestWrapToPi(unittest.TestCase):

    def test_no_wrap_needed(self):
        self.assertAlmostEqual(wrap_to_pi(0.5), 0.5, places=6)

    def test_wrap_positive_overflow(self):
        # pi + 0.1 -> 넘어가면 -pi + 0.1 근방으로 wrap
        val = wrap_to_pi(math.pi + 0.1)
        self.assertTrue(-math.pi <= val <= math.pi)
        self.assertAlmostEqual(val, -math.pi + 0.1, places=6)

    def test_wrap_negative_overflow(self):
        val = wrap_to_pi(-math.pi - 0.1)
        self.assertTrue(-math.pi <= val <= math.pi)
        self.assertAlmostEqual(val, math.pi - 0.1, places=6)

    def test_error_across_boundary_takes_short_path(self):
        # yaw_est가 +179도, course가 -179도인 경우
        # 실제 각도차는 2도여야지 358도가 되면 안 됨
        yaw_est = math.radians(179)
        course = math.radians(-179)
        error = wrap_to_pi(course - yaw_est)
        self.assertAlmostEqual(math.degrees(error), 2.0, places=3)


class TestReverseDetectionLogic(unittest.TestCase):
    """전진/후진 판별 데드밴드 로직 검증 (노드 인스턴스화 없이 조건식만 검증)"""

    def _classify(self, vx: float, prev_reverse: bool) -> bool:
        if vx > 0.02:
            return False
        elif vx < -0.02:
            return True
        return prev_reverse

    def test_forward(self):
        self.assertFalse(self._classify(0.5, True))

    def test_reverse(self):
        self.assertTrue(self._classify(-0.5, False))

    def test_deadband_holds_previous_state(self):
        self.assertTrue(self._classify(0.0, True))
        self.assertFalse(self._classify(0.0, False))


if __name__ == '__main__':
    unittest.main()
