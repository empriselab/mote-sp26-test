PKG = "hw3_pkg"
NAME = "pf_test"

import sys, unittest, time

import numpy as np
import math

from collections import Counter

# from gradescope_utils.autograder_utils.decorators import weight

import rospy, rostest

from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan

from hw3_pkg.particle_filter import particle_filter


class TestPF(unittest.TestCase):
    # this is called when initializing the test class
    @classmethod
    def setUpClass(cls):
        cls.pf = particle_filter(np.array([]), np.array([]))

        # create a publisher to send fake data
        cls.joint_states_publisher = rospy.Publisher("scan", LaserScan, queue_size=10)

        rospy.sleep(0.5)  # wait for publishers to init

    # this is called before each test fixture
    def setUp(self):
        self.pf = particle_filter(np.array([]), np.array([]))

    def __init__(self, *args):
        super(TestPF, self).__init__(*args)

    #@weight(5.0)
    def test_sensor_model_single_val(self):
        prob = self.pf.sensor_model(np.array([1.0]), np.array([1.0]), 0.1)
        self.assertAlmostEqual(
            prob, 3.989422804014327, msg="Incorrect unweighted probability"
        )

        prob = self.pf.sensor_model(np.array([1.0]), np.array([1.3]), 0.1)
        self.assertAlmostEqual(
            prob, 0.04431848411938, msg="Incorrect unweighted probability"
        )

    #@weight(5.0)
    def test_sensor_model_ignores_nan(self):
        prob = self.pf.sensor_model(
            np.array([1.0, math.nan]), np.array([1.0, 2.0]), 0.1
        )
        self.assertAlmostEqual(
            prob, 3.989422804014327, msg="Does not ignore NaN values in measurement"
        )

        prob = self.pf.sensor_model(
            np.array([1.0, 1.1]), np.array([1.0, math.nan]), 0.1
        )
        self.assertAlmostEqual(
            prob, 3.989422804014327, msg="Does not ignore NaN values in ground_truth"
        )

    #@weight(10.0)
    def test_sensor_model_many(self):
        prob = self.pf.sensor_model(
            np.array([1.0, 1.1, 1.2, 1.3]), np.array([1.0, 0.9, 0.8, 0.7]), 0.1
        )
        self.assertAlmostEqual(
            prob, 1.751438007529713e-10, msg="Does not ignore NaN values in measurement"
        )

    #@weight(10.0)
    def test_resampler_inplace(self):
        n_particles = 100  # number of particles

        particles = np.arange(n_particles)
        particles = particles[..., np.newaxis]
        weights = np.arange(n_particles, dtype=float)
        weights /= np.sum(weights)
        prev_particles = particles.copy()
        prev_weights = weights.copy()
        rs = particle_filter(particles, weights)
        rs.resample()
        self.assertTrue(
            np.allclose(rs.particles, particles),
            msg="Resampler should modify particles in-place",
        )
        self.assertFalse(
            np.allclose(prev_particles, particles),
            msg="Resampler should modify particles in-place",
        )
        self.assertTrue(
            np.allclose(rs.weights, weights),
            msg="Resampler should modify weights in-place",
        )
        self.assertFalse(
            np.allclose(prev_weights, weights),
            msg="Resampler should modify weights in-place",
        )

    #@weight(10.0)
    def test_resampler_is_fair(self):
        n_particles = 100  # number of particles
        k_val = 50  # number of particles that have non-zero weight
        trials = 10  # number of resamplings to do

        # Count how often each particle has been sampled across trials
        histogram = Counter()
        for i in range(trials):
            # Create a set of particles with weights proportional to their index
            # Particles with index greater than k_val are given zero weight
            particles = np.arange(n_particles)
            particles = particles[..., np.newaxis]
            weights = np.arange(n_particles, dtype=float)
            weights[k_val:] = 0.0
            weights /= np.sum(weights)

            rs = particle_filter(particles, weights)
            rs.resample()

            # Add the number of times each particle was sampled
            histogram.update(particles[:, 0])

        count_pairs = np.array(list(histogram.items()))
        counts = np.zeros((n_particles), dtype=int)
        counts[count_pairs[:, 0]] = count_pairs[:, 1]
        self.assertFalse(
            np.any(counts[k_val:] > 0),
            msg="Particles with 0 weight should never be sampled",
        )
        for i in range(4):
            # Compare mean number of samples for first half vs second half of particles.
            # Move up the the start index for the total range we're looking at
            # by powers of 2 (1/2, 1/4, 1/8 ...) so we're focusing on progressively
            # smaller slices of the end of the non-zero region.
            start = int(round(k_val * (1 - (2**-i))))
            end = k_val
            middle = (start + end) // 2
            self.assertLess(
                counts[start:middle].mean(),
                counts[middle:end].mean(),
                msg="Particles with less weight should be sampled less than those with more weight",
            )

    #@weight(10.0)
    def test_resampler_is_complete(self):
        n_particles = 100  # number of particles
        trials = 100

        for i in range(trials):
            histogram = Counter()

            # Create a set of particles with uniform weights
            particles = np.arange(n_particles)
            particles = particles[..., np.newaxis]
            weights = np.full([n_particles], 1.0 / n_particles)

            rs = particle_filter(particles, weights)
            rs.resample()

            # Add the number of times each particle was sampled
            histogram.update(particles[:, 0])
            count_pairs = np.array(list(histogram.items()), dtype=int)
            counts = np.zeros((n_particles))
            counts[count_pairs[:, 0]] = count_pairs[:, 1]
            self.assertFalse(
                np.any(counts == 0),
                msg="All particles should have been sampled at least once",
            )

    #@weight(10.0)
    def test_resampler_resets_weights(self):
        n_particles = 100  # number of particles

        # Create a set of particles with uniform weights
        particles = np.arange(n_particles)
        particles = particles[..., np.newaxis]
        weights = np.random.uniform(0, 1, n_particles)
        weights /= weights.sum()

        rs = particle_filter(particles, weights)
        rs.resample()

        np.testing.assert_equal(
            weights,
            np.full(n_particles, 1.0 / n_particles),
            err_msg="The resampler should set the weights to be uniform",
        )


if __name__ == "__main__":
    rospy.init_node(NAME, anonymous=True)

    rostest.rosrun(PKG, NAME, TestPF, sys.argv)
