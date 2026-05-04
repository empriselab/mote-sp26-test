from __future__ import annotations

import numpy as np
import numpy.typing

import scipy.stats as stats


from hw3_pkg.map_helpers import gen_lidar_scan

from tf.transformations import quaternion_from_euler


class particle_filter:
    def __init__(
        self, particles: numpy.ndarray[float], weights: numpy.ndarray[np.float64]
    ):
        # Setup some particle variables
        self.rng = np.random.default_rng(42)
        self.particles = particles
        self.n_particles = particles.shape[0]
        self.weights = weights

    def sensor_model(
        self,
        measurement: np.ndarray[np.float64],
        ground_truth: np.ndarray[np.float64],
        std_dev: float,
    ) -> float:
        """
        INPUTS
         * measurement and ground_truth
               360 LiDAR readings, stored as a numpy array.
               Invalid readings are set to NaN.
         * std_dev: the standard deviation of the error in each reading

        OUTPUTS
         * the unnormalized probability of reading measurement when the actual distances are
           ground_truth

        HINTS:
         * If a reading from either measurement or ground_truth is NaN, ignore it
           in both readings
         * Because we are assuming readings are IID, the joint probability of all
           readings is equal to their product.
         * I've imported scipy.stats as stats at the top of this file.
           stats.norm.pdf may be of interest
        """

        if np.all(np.isnan(measurement)) or np.all(np.isnan(ground_truth)):
            print("All measurements are NaN, returning 0 probability")
            return 0.0

        # QUESTION 1.1 BEGINS
        valid_idx = np.argwhere(~np.isnan(measurement) & ~np.isnan(ground_truth) & ~np.isinf(measurement) & ~np.isinf(ground_truth))
        # print("Valid measurements:")
        # print(measurement[valid_idx])
        # print("Valid ground truth:")
        # print(ground_truth[valid_idx])
        return np.product(
            stats.norm.pdf(measurement[valid_idx], ground_truth[valid_idx], std_dev)
        )
        # QUESTION 1.1 ENDS

    def resample(self):
        """
        Resample particles using the low-variance sampling scheme.

        Both self.particles and self.weights should be modified in-place.
        Aim for an efficient O(M) implementation!
        """

        # QUESTION 2.1 BEGINS

        # Sample solution taken from FOR HW 3.2 solutions
        self.step_array = np.arange(self.n_particles, dtype=np.float32)
        self.step_array /= self.n_particles
        self.indices = np.zeros(self.n_particles, dtype=int)

        # Choose an initial value from half open interval [0, 1/M)
        initval = self.rng.uniform(0, self.n_particles**-1)

        bin_parts = initval + self.step_array
        cum_weights = np.cumsum(self.weights)

        self.indices = np.searchsorted(cum_weights, bin_parts, side="left")

        assert np.all(self.indices < self.n_particles)
        self.particles[:] = self.particles[self.indices, :]

        # Uniformly weight new particles
        self.weights.fill(1.0 / self.n_particles)

        # QUESTION 2.1 ENDS
        pass

    def predict(self, dt):
        """
        Apply gausian noise to the particles
        No particular motion model because we're assuming the robot isn't moving
        """
        self.particles[:, 0] += self.rng.standard_normal(self.n_particles) * 0.05 * dt
        self.particles[:, 1] += self.rng.standard_normal(self.n_particles) * 0.05 * dt
        self.particles[:, 2] += self.rng.standard_normal(self.n_particles) * 0.15 * dt

    def update(self):
        mean = np.average(self.particles, weights=self.weights, axis=0)
        zero_mean = self.particles - mean
        cov = np.dot(zero_mean.T, zero_mean) / self.n_particles
        return mean, cov