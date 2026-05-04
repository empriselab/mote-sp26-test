from __future__ import annotations

import rospy

import numpy as np

import threading

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import (
    Twist,
    Vector3,
    PoseWithCovarianceStamped,
    PoseStamped,
)
from visualization_msgs.msg import Marker

import tf

from hw4_pkg.rrt_visualizer import rrt_visualizer as RRTVisualizer
from hw4_pkg.pure_pursuit_visualizer import pure_pursuit_visualizer as PPVisualizer

from hw4_pkg.pure_pursuit import pure_pursuit as PurePursuit
from hw4_pkg.planning.problems import R2Problem
from hw4_pkg.planning.search import RRTPlanner
import hw4_pkg.planning.utils as util


class pure_pursuit_node:
    def __init__(self):
        # visualization publishers
        self.rrt_explore_viz_publisher = rospy.Publisher(
            "/rrt_explore_viz", Marker, queue_size=1
        )
        self.rrt_path_viz_publisher = rospy.Publisher(
            "/rrt_path_viz", Marker, queue_size=1
        )
        self.pp_path_viz_publisher = rospy.Publisher(
            "/pure_pursuit_path_viz", Marker, queue_size=1
        )
        self.cmd_vel_publisher = rospy.Publisher(
            "/diff_drive_controller/cmd_vel", Twist, queue_size=1
        )

        # wait for publisher to init
        rospy.sleep(0.5)

        self.rrt_visualizer = None

        self.pp_visualizer = PPVisualizer(self.pp_path_viz_publisher)

        # get map info
        self.map = None
        rospy.Subscriber(
            "/costmap_node/costmap/costmap", OccupancyGrid, self.map_callback
        )

        # get target location
        self.goal = None
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)

        self.tf_listener = tf.TransformListener()

        self.plan_mutex = threading.Lock()
        self.plan = None

        thread = threading.Thread(target=self.thread_main)
        thread.start()

    def thread_main(self):
        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            if not self.plan_mutex.acquire(timeout=1.0):
                continue

            if self.plan is None:
                continue

            (trans, rot) = self.tf_listener.lookupTransform(
                "base_link", "map", rospy.Time(0)
            )
            map_to_base_link_tranform = tf.transformations.concatenate_matrices(
                tf.transformations.translation_matrix(trans),
                tf.transformations.quaternion_matrix(rot),
            )

            homogeneous_points = np.repeat(
                [[0.0, 0.0, 0.0, 1.0]], self.plan.shape[0], axis=0
            )

            util.map_to_world(self.plan, self.map.info, out=homogeneous_points[:, 0:2])

            robot_frame_plan = (map_to_base_link_tranform @ homogeneous_points.T).T

            self.pure_pursuit = PurePursuit(robot_frame_plan[:, 0:2])

            rad = self.pure_pursuit.compute_radius()
            lin, ang = self.pure_pursuit.get_control()
            self.plan_mutex.release()

            # rospy.loginfo(f"Radius: {rad}, Linear Velocity: {lin}, Angular Velocity: {ang}")

            self.pp_visualizer.show_path(rad)

            cmd = Twist(Vector3(lin, 0.0, 0.0), Vector3(0.0, 0.0, ang))
            self.cmd_vel_publisher.publish(cmd)

            r.sleep()

    def map_callback(self, msg: OccupancyGrid):
        if self.map == None:
            self.map = msg
            self.rrt_visualizer = RRTVisualizer(
                self.rrt_explore_viz_publisher, self.rrt_path_viz_publisher, self.map
            )

    def goal_callback(self, msg: PoseStamped):
        self.goal = msg

        self.rrt_visualizer.clear()

        if self.map is None:
            rospy.logerr("Map failed to load.")
            return

        self.plan_mutex.acquire(timeout=1.0)
        plan = self.compute_plan()

        if plan is None:
            return

        self.plan = plan
        self.plan_mutex.release()

        self.rrt_visualizer.visualize_plan(plan)

    def compute_plan(self):
        # Compute plan
        permissible_region = np.array(self.map.data).reshape(
            (self.map.info.height, self.map.info.width)
        )

        obstructed = (permissible_region > 25) | np.equal(permissible_region, -1)
        permissible_region[obstructed] = 0
        permissible_region[~obstructed] = 1

        planning_env = R2Problem(permissible_region)

        (trans, _) = self.tf_listener.lookupTransform("map", "base_link", rospy.Time(0))
        start = util.world_to_map(
            np.array([[trans[0], trans[1], 0, 0]]),
            self.map.info,
        )

        goal = util.world_to_map(
            np.array([[self.goal.pose.position.x, self.goal.pose.position.y, 0.0]]),
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

        return planner.Plan(
            start[0, 0:2], goal[0, 0:2], self.rrt_visualizer.visualize_edge
        )


def main(args=None):
    # init the node
    rospy.init_node("pure_pursuit", anonymous=True)

    _ = pure_pursuit_node()

    # let ROS handle ROS things
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
