from __future__ import annotations

import rospy

import numpy as np

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    PoseStamped,
)

from visualization_msgs.msg import Marker

from hw4_pkg.rrt_visualizer import rrt_visualizer as RRTVisualizer

from hw4_pkg.planning.problems import R2Problem
from hw4_pkg.planning.search import RRTPlanner
import hw4_pkg.planning.utils as util


class rrt_node:
    def __init__(self):
        # visualization publishers
        self.rrt_explore_viz_publisher = rospy.Publisher(
            "/rrt_explore_viz", Marker, queue_size=1
        )
        self.rrt_path_viz_publisher = rospy.Publisher(
            "/rrt_path_viz", Marker, queue_size=1
        )

        # wait for publisher to init
        rospy.sleep(0.5)

        self.visualizer = None

        # get map info
        self.map = None
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)

        # get initial position
        self.initial_pose = None
        rospy.Subscriber(
            "/initialpose", PoseWithCovarianceStamped, self.initial_pose_callback
        )

        # get target location
        self.goal = None
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)

        self.path = None

        # wait for publisher to init
        rospy.sleep(0.5)

    def map_callback(self, msg: OccupancyGrid):
        self.map = msg
        self.visualizer = RRTVisualizer(
            self.rrt_explore_viz_publisher, self.rrt_path_viz_publisher, self.map
        )

    def initial_pose_callback(self, msg: PoseWithCovarianceStamped):
        self.initial_pose = msg

    def goal_callback(self, msg: PoseStamped):
        self.goal = msg

        self.visualizer.clear()

        if self.map is None:
            rospy.logerr("Map failed to load.")
            return
        if self.initial_pose is None:
            rospy.logwarn("Please set initial pose before setting goal.")
            return

        plan = self.compute_plan()

        if plan is None:
            return

        self.visualizer.visualize_plan(plan)

    def compute_plan(self):
        # Compute plan
        permissible_region = np.array(self.map.data).reshape(
            (self.map.info.height, self.map.info.width)
        )

        obstructed = permissible_region != 0
        permissible_region[obstructed] = 0
        permissible_region[~obstructed] = 1

        planning_env = R2Problem(permissible_region)

        start = util.world_to_map(
            np.array(
                [
                    self.initial_pose.pose.pose.position.x,
                    self.initial_pose.pose.pose.position.y,
                ]
            ),
            self.map.info,
        )
        goal = util.world_to_map(
            np.array(
                [
                    self.goal.pose.position.x,
                    self.goal.pose.position.y,
                ]
            ),
            self.map.info,
        )

        planner = RRTPlanner(
            planning_env,
            permissible_region,
            bias=0.05,
            eta=1.0,
            show_tree=True,
            batch_size=1,
            shortcut=False,
        )

        return planner.Plan(start, goal, self.visualizer.visualize_edge)


def main(args=None):
    # init the node
    rospy.init_node("rrt", anonymous=True)

    _ = rrt_node()

    # let ROS handle ROS things
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
