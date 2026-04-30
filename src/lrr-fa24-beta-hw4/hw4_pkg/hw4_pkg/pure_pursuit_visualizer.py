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


class pure_pursuit_visualizer:
    def __init__(self, path_publisher):
        # Setup graph
        self.radius_graph = Marker()
        self.radius_graph.header.frame_id = "base_link"
        self.radius_graph.header.stamp = rospy.Time.now()
        self.radius_graph.ns = "pure_pursuit_viz"
        self.radius_graph.type = Marker.LINE_STRIP
        self.radius_graph.action = Marker.ADD
        self.radius_graph.pose = Pose()
        self.radius_graph.pose.position = Point(0.0, 0.0, 0.0)
        self.radius_graph.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        self.radius_graph.scale = Vector3(0.02, 0.00, 0.00)
        self.radius_graph.lifetime.set(0, 0)
        self.radius_graph.id = 0
        self.radius_graph.frame_locked = True

        # publishers
        self.viz_publisher = path_publisher

    def show_path(self, radius):
        theta = np.linspace(0, 2 * np.pi, 100)
        x = radius * np.cos(theta)
        y = radius * np.sin(theta) + radius
        padded = np.concatenate(
            (np.column_stack((x, y)), np.repeat([[0, 1]], x.shape[0], axis=0)), axis=1
        )
        self.radius_graph.points = list(
            map(lambda p: Point(p[0], p[1], 0.1), padded.tolist())
        )
        self.viz_publisher.publish(self.radius_graph)

    def clear(self):
        self.radius_graph.points = []
        self.radius_graph.colors = []
        self.viz_publisher.publish(self.radius_graph)
