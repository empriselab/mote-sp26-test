from __future__ import annotations

import numpy as np


class pure_pursuit:
    def __init__(self, path):
        # path is list of points IN THE ROBOT'S BODY FRAME

        # Sample points every 10 cm along the path
        # This allows us to find the lookahead using a lookup instead of calculating circle / line intersections
        distances = np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))
        total_distance = distances[-1]
        sample_distances = np.linspace(0, total_distance, int(total_distance / 0.1))
        points_x = np.interp(sample_distances, np.insert(distances, 0, 0.0), path[:, 0])
        points_y = np.interp(sample_distances, np.insert(distances, 0, 0.0), path[:, 1])
        self.path = np.column_stack([points_x, points_y])

        self.lookahead = 0.1

        self.linear_vel = 0.02
        self.turn_around_speed = 1.5

    def get_lookahead(self, closest_idx):
        # Find points that are self.lookahead away from the robot, then return the point furthest along the path
        ahead_of_closest = self.path[closest_idx:]
        candidates = np.argwhere(
            np.abs(np.abs(np.linalg.norm(ahead_of_closest, axis=1)) - self.lookahead)
            < 0.1
        )
        if len(candidates) == 0:
            return self.path[
                np.argmin(
                    np.abs(np.linalg.norm(ahead_of_closest, axis=1) - self.lookahead)
                )
            ]

        return self.path[np.max(candidates) + closest_idx]

    def distance_to_path(self):
        return np.min(np.linalg.norm(self.path, axis=1))

    def compute_radius(self):
        """
        Compute the radius of curvature using geometric pure pursuit
        """

        closest_point_idx = np.argmin(np.linalg.norm(self.path, axis=1))
        lookahead_point = self.get_lookahead(closest_point_idx)

        # QUESTION 2.1 BEGINS
        # ignore divide by zero here, it'll get caught in get_control

        xH = lookahead_point[0]
        yH = lookahead_point[1] 
        radius = (xH ** 2 + yH ** 2) / (2 * yH)


        # QUESTION 2.1 ENDS

        return radius

    def get_control(self):
        """
        Compute a tuple (linear velocity, angular velocity) to send to the rover.
        """

        closest_point_idx = np.argmin(np.linalg.norm(self.path, axis=1))
        lookahead_point = self.get_lookahead(closest_point_idx)

        radius = self.compute_radius()

        linear_vel = self.linear_vel

        if np.abs(radius) == np.inf or radius == np.nan:
            return (linear_vel, 0.0)

        # QUESTION 2.2 BEGINS
        # While traveling forward at linear_vel, what angular velocity does the robot need
        # to achieve to follow a curve with radius "radius"?
        # Your answer should only be in terms of linear_vel and radius

        angular_vel = linear_vel / radius

        # QUESTION 2.2 ENDS

        if np.linalg.norm(self.path[-1]) < 0.1:
            # End of the road!
            return (0.0, 0.0)

        if np.abs(np.arctan2(lookahead_point[1], lookahead_point[0])) > np.pi / 2:
            # If we're facing the entirely wrong direction, turn in place
            angular_vel = self.turn_around_speed * np.sign(radius)
            linear_vel = 0

        return (linear_vel, angular_vel)
