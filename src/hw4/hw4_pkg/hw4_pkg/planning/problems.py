import numpy as np

import hw4_pkg.planning.utils as utils

import numpy as np
from matplotlib import pyplot as plt
import rospy
from sensor_msgs.msg import JointState


class PlanarProblem(object):
    def __init__(self, permissible_region, map_info=None, check_resolution=0.1):
        """Construct a planar planning problem.

        Args:
            permissible_region: Boolean np.array with shape map height x map width,
                where one indicates that the location is permissible
            map_info: map information, returned by get_map
            check_resolution: collision-checking resolution
        """
        self.permissible_region = permissible_region
        self.map_info = map_info
        self.check_resolution = check_resolution

        height, width = permissible_region.shape
        self.extents = np.zeros((3, 2))
        self.extents[0, 1] = width
        self.extents[1, 1] = height
        self.name = "CarSpace"

        if map_info is not None:
            map_angle = utils.quaternion_to_angle(map_info.origin.orientation)
            assert map_angle == 0
            utils.map_to_world(self.extents.T, map_info)
        self.extents = self.extents[:2, :]

    def check_state_validity(self, states):
        """Return whether states are valid.

        Valid states are within the extents of the map and collision-free.

        Args:
            states: np.array with shape N x D (where D may be 2 or 3)

        Returns:
            valid: np.array of Booleans with shape N
        """
        x = states[:, 0]
        y = states[:, 1]
        valid = np.ones_like(x, dtype=bool)  # feel free to delete this line

        # Check that x and y are within the extents of the map.
        xmin, xmax = self.extents[0, :]
        ymin, ymax = self.extents[1, :]
        within_x = (xmin <= x) & (x < xmax)
        within_y = (ymin <= y) & (y < ymax)
        valid = within_x & within_y

        # The units of the state are meters and radians. We need to convert the
        # meters to pixels, in order to index into the permissible region. This
        # function converts them in place.
        if self.map_info is not None:
            utils.world_to_map(states, self.map_info)

        yind = y.astype(int)
        xind = x.astype(int)
        coll_free = self.permissible_region[yind[valid], xind[valid]]
        valid[valid] = coll_free

        # Convert the units back from pixels to meters for the caller
        if self.map_info is not None:
            utils.map_to_world(states, self.map_info)

        return valid

    def check_edge_validity(self, q1, q2):
        """Return whether an edge is valid.

        Args:
            q1, q2: np.arrays with shape (1, D) (where D may be 2, 3 or 6)

        Returns:
            valid: True or False
        """

        path, length = self.steer(q1, q2)
        if length == 0:
            return False
        return self.check_state_validity(path).all()

    def cost_to_go(self, q1, q2):
        """Compute an admissible heuristic between two states.
        Args:
            q1, q2: np.arrays with shape (N, D) (where D may be 2, 3 or 6)
        Returns:
            heuristic: np.array with shape N of cost estimates between pairs of states
        """
        # Subclasses implement this.
        raise NotImplementedError

    def compute_distance(self, q1, q2):
        """Compute cost between two states.
        Args:
            q1, q2: np.arrays with shape (N, D) (where D may be 2, 3 or 6)
        Returns:
            heuristic: np.array with shape N of cost estimates between pairs of states
        """
        # Subclasses implement this.
        raise NotImplementedError

    def steer(self, q1, q2, **kwargs):
        """Return a local path connecting two states.
           Implemented by subclasses.

        Intermediate states are used for edge collision-checking.

        Args:
            q1, q2: np.arrays with shape (1, D) (where D may be 2, 3 or 6)

        Returns:
            path: sequence of states between q1 and q2
            length: length of local path
        """
        # Subclasses implement this.
        raise NotImplementedError


class R2Problem(PlanarProblem):
    def compute_distance(self, q1, q2):
        """Computes the Euclidean distance between corresponding elements of two arrays of states.

        Note that q1 and q2 may be of shape (2,) or (N,2) so make sure to use numpy broadcasting.

        Args:
            q1: np.array corresponding to (one or more) R2 states of dimension 2
            q2: np.array corresponding to (one or more) R2 states of dimension 2

        Returns:
            costs: np.array of shape (N,)
        """
        q1_2d = np.atleast_2d(q1)
        q2_2d = np.atleast_2d(q2)
        ### BEGIN QUESTION 1.1 #####################
        return np.linalg.norm(q1_2d - q2_2d, axis=1)
        ### END QUESTION 1.1 #######################

    def cost_to_go(self, q1, goal):
        """Computes the Euclidean distance between each element of an array of R2 states and a goal R2 state (used by AStar).

        Args:
            q1: np.array with shape (N, 2), N states where each row is a R2 state of dimension (1,2)
            goal: np.array with shape (1, 2), a goal R2 state

        Returns:
            heuristic: np.array with shape (N,), where element at index i is the Euclidean distance between
                       R2 state at index i of q1 and goal R2 state.
        """
        return self.compute_distance(q1, goal)

    def compute_distance_rrt(self, q1, q2):
        """Computes the Euclidean distance between two states (used by RRT).

        Args:
            q1: np.array with shape (1, 2), a R2 state
            q2: np.array with shape (1, 2), a R2 state

        Returns:
            heuristic: scalar float, Euclidean distance between R2 states q1 and q2
        """
        return np.ndarray.item(np.floor(self.compute_distance(q1, q2)))

    def steer(self, q1, q2, resolution=None, interpolate_line=True):
        """Return a straight-line path connecting two R2 states.

        Args:
            q1, q2: np.arrays with shape (1, 2)
            resolution: the space between waypoints in the resulting path
            interpolate_line: whether to provide fine waypoint discretization
             for line segments

        Returns:
            path: sequence of states between q1 and q2
            length: length of local path
        """
        if resolution is None:
            resolution = self.check_resolution
        q1 = q1.reshape((1, -1))
        q2 = q2.reshape((1, -1))
        dist = np.linalg.norm(q2 - q1)
        if not interpolate_line or dist < resolution:
            return np.vstack((q1, q2)), dist
        q1_toward_q2 = (q2 - q1) / dist
        steps = np.hstack((np.arange(0, dist, resolution), np.array([dist]))).reshape(
            (-1, 1)
        )
        return q1 + q1_toward_q2 * steps, dist
