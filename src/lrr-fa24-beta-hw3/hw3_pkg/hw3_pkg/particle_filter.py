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
               Evenly spaced LiDAR readings, stored as a numpy array.
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
            return 0.0

        # QUESTION 1.1 BEGINS

        # p(z|x) = p(x|z)p(z)/p(x)
        prob = 1
        for meas, grt in zip(measurement, ground_truth):
            if np.isnan(meas) or np.isnan(grt):
                continue
            prob *= stats.norm.pdf(meas, loc=grt, scale=std_dev)
        return prob

        # QUESTION 1.1 ENDS

    def resample(self):
        """
        Resample particles using the low-variance sampling scheme.

        Both self.particles and self.weights should be modified in-place.
        Aim for an efficient O(M) implementation!
        """

        # QUESTION 1.2 BEGINS
            # BEGIN SOLUTION "QUESTION 1.3"
        temp_particles = []
        r = self.rng.uniform(0, 1/self.n_particles)
        c = self.weights[0]
        i = 0
        for m in range(1, self.n_particles+1):
            U = r + (m-1) * 1/self.n_particles
            while U > c:
                i += 1
                c += self.weights[i]
            temp_particles.append(i)
        self.particles[:] =  self.particles[temp_particles]
        self.weights[:] = 1/self.n_particles
            # END SOLUTION

        # QUESTION 1.2 ENDS
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
