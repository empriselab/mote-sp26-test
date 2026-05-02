import numpy as np
import os
import rosunit
import unittest

from hw4_pkg.planning import search
from hw4_pkg.planning.problems import R2Problem

# from gradescope_utils.autograder_utils.decorators import weight


class TestRRT(unittest.TestCase):
    def setUp(self):
        np.random.seed(111)
        fname = os.path.join(os.path.dirname(__file__), "share", "map1.txt")
        self.permissible_region = np.loadtxt(fname, dtype=bool)
        self.problem = R2Problem(self.permissible_region, check_resolution=0.5)

    #@weight(20)
    def test_rrt_r2problem(self):
        np.random.seed(111)
        rrt = search.RRTPlanner(self.problem, self.permissible_region)

        start = np.array([[1.0, 6.0]])
        goal = np.array([[9.0, 3.0]])
        correct_path = np.array(
            [[1.0, 6.0], [0.0, 3.0], [4.0, 0.0], [10.0, 4.0], [9.0, 3.0]]
        )

        path = rrt.Plan(start, goal)

        self.assertTrue(
            np.allclose(path[0], start),
            msg="Path must begin at start",
        )

        self.assertTrue(
            np.allclose(path[-1], goal),
            msg="Path must end at goal",
        )

        self.assertTrue(
            np.allclose(path, correct_path),
            msg="RRT implementation is incorrect",
        )


if __name__ == "__main__":
    np.random.seed(111)
    rosunit.unitrun("planning", "test_rrt_r2", TestRRT)
