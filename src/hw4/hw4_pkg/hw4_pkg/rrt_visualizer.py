from __future__ import annotations

import rospy

import numpy as np

from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import (
    Pose,
    Point,
    Quaternion,
    Vector3,
)
from visualization_msgs.msg import Marker

import copy

import hw4_pkg.planning.utils as util


class rrt_visualizer:
    def __init__(self, explore_publisher, path_publisher, map):
        # Setup graph
        self.exploration_graph = Marker()
        self.exploration_graph.header.frame_id = "map"
        self.exploration_graph.header.stamp = rospy.Time.now()
        self.exploration_graph.ns = "rrt_viz"
        self.exploration_graph.type = Marker.LINE_LIST
        self.exploration_graph.action = Marker.ADD
        self.exploration_graph.pose = Pose()
        self.exploration_graph.pose.position = Point(0.0, 0.0, 0.0)
        self.exploration_graph.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        self.exploration_graph.scale = Vector3(0.02, 0.00, 0.0)
        self.exploration_graph.lifetime.set(0, 0)
        self.path_graph = copy.deepcopy(self.exploration_graph)
        self.exploration_graph.id = 0
        self.path_graph.id = 1

        # self.path_graph.header.frame_id = "base_link"
        # self.path_graph.frame_locked = True

        # publishers
        self.rrt_explore_viz_publisher = explore_publisher
        self.rrt_path_viz_publisher = path_publisher

        self.map = map

    def visualize_plan(self, plan):
        self.visualize_graph(self.path_graph, plan, [0.0, 1.0, 0.0, 1.0], z=0.1)
        self.rrt_path_viz_publisher.publish(self.path_graph)

    def visualize_edge(self, start, end):
        self.visualize_graph(
            self.exploration_graph, np.array([start, end]), [1.0, 0.0, 0.0, 1.0]
        )
        self.rrt_explore_viz_publisher.publish(self.exploration_graph)

    def visualize_graph(self, graph, plan, color, z=0.0):
        for i in range(1, len(plan)):
            s = util.map_to_world(np.append(plan[i - 1], 0.0), self.map.info).flatten()
            e = util.map_to_world(np.append(plan[i], 0.0), self.map.info).flatten()
            graph.points.append(Point(s[0], s[1], z))
            graph.points.append(Point(e[0], e[1], z))
            graph.colors.append(ColorRGBA(*color))

    def clear(self):
        self.exploration_graph.points = []
        self.exploration_graph.colors = []
        self.path_graph.points = []
        self.path_graph.colors = []
        self.exploration_graph.action = Marker.DELETEALL
        self.path_graph.action = Marker.DELETEALL
        self.rrt_path_viz_publisher.publish(self.path_graph)
        self.rrt_explore_viz_publisher.publish(self.exploration_graph)
        self.exploration_graph.action = Marker.ADD
        self.path_graph.action = Marker.ADD