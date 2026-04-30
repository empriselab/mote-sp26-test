import numpy as np
import os
import rosunit
import unittest

from gradescope_utils.autograder_utils.decorators import weight

from hw4_pkg.pure_pursuit import pure_pursuit


class TestPurePursuit(unittest.TestCase):
    def setUp(self):
        pass

    @weight(5)
    def test_curve_radius_straight_path(self):
        path = np.array([[1.0, 0.0], [2.0, 0.0]])

        pp = pure_pursuit(path)

        self.assertEqual(
            pp.compute_radius(),
            np.inf,
            msg="get_lookahead should return np.inf when the lookahead is directly in front.",
        )

    @weight(5)
    def test_curve_radius_zig_zag(self):
        path = np.array([[0.1, 0.7], [2.0, 1.0], [5.0, -4.0], [10.0, -2.0]])
        pp = pure_pursuit(path)
        self.assertEqual(
            pp.compute_radius(), 0.35714285714285715, msg="Incorrect radius"
        )

    @weight(10)
    def test_curve_radius_negative(self):
        path = np.array(
            [
                [0.1, -0.7],
                [2.0, -1.0],
            ]
        )
        pp = pure_pursuit(path)
        self.assertEqual(
            pp.compute_radius(), -0.35714285714285715, msg="Incorrect radius"
        )

    @weight(5)
    def test_control_straight_forward(self):
        path = np.array([[1.0, 0.0], [2.0, 0.0]])

        pp = pure_pursuit(path)
        lin_vel, ang_vel = pp.get_control()

        self.assertEqual(
            ang_vel,
            0.0,
            msg="Angular velocity should be zero when lookahead point is directly in front.",
        )

        self.assertEqual(
            lin_vel,
            pp.linear_vel,
            msg=f"Linear velocity should always be equal to {pp.linear_vel}.",
        )

    @weight(5)
    def test_control_zig_zag(self):
        path = np.array([[0.1, 0.7], [2.0, 1.0], [5.0, -4.0], [10.0, -2.0]])

        pp = pure_pursuit(path)
        lin_vel, ang_vel = pp.get_control()

        self.assertEqual(ang_vel, 0.7, msg="Incorrect angular velocity")

        self.assertEqual(
            lin_vel,
            pp.linear_vel,
            msg=f"Linear velocity should always be equal to {pp.linear_vel}.",
        )


if __name__ == "__main__":
    np.random.seed(111)
    rosunit.unitrun("planning", "test_rrt_r2", TestPurePursuit)
