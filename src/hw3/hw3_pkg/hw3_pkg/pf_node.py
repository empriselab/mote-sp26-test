from __future__ import annotations

import rospy

import numpy as np

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Time
from nav_msgs.msg import OccupancyGrid

from hw3_pkg.map_helpers import gen_lidar_scan
from hw3_pkg.particle_filter import particle_filter

import tf.transformations
from tf.transformations import quaternion_from_euler

import threading


class pf_node:
    def __init__(self):
        # initial pose
        self.initial_pose = None
        self.reset_distribution = False

        # tunable parameters
        self.num_particles = 150
        self.lidar_std = 0.10
        self.lidar_subsample_count = 60

        self.pf = self.get_initial_distribution()

        # subscribe to messages of type LaserScan on the /scan topic
        self.laser_scan_subscription = rospy.Subscriber(
            "scan", LaserScan, self.scan_callback
        )

        # subscribe to messages of type PoseWithCovariance on the /initialpose topic
        self.laser_scan_subscription = rospy.Subscriber(
            "initialpose", PoseWithCovarianceStamped, self.initial_pose_callback
        )

        # publish messages of type on the /scan topic
        self.pose_array_publisher = rospy.Publisher(
            "particlecloud", PoseArray, queue_size=10
        )

        # publish ground_truth
        self.gt_publisher = rospy.Publisher(
            "scan_ground_truth", LaserScan, queue_size=10
        )

        # initialize the tf tree
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster.sendTransform(
            (0.0, 0.0, 0.0),
            quaternion_from_euler(0.0, 0.0, 0.0),
            rospy.Time.now(),
            "odom",
            "map",
        )

        # map info
        self.map = None
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)

        # latest scan message
        self.mutex = threading.Lock()
        self.scan = None

        # wait for publisher to init
        rospy.sleep(0.5)

        thread = threading.Thread(target=self.thread_main)
        thread.start()

    def thread_main(self):
        while not rospy.is_shutdown():
            if (self.map is None) or (self.scan is None):
                # We haven't recieved map or scan info yet, wait for a bit
                rospy.sleep(0.1)
                continue

            if self.reset_distribution:
                self.pf = self.get_initial_distribution()

            # If this is the first message, set the last time and wait for a second message
            try:
                self.last_stamp
            except AttributeError:
                self.last_stamp = rospy.Time.now()
                continue

            # Update the last timestamp
            self.last_stamp = rospy.Time.now()

            # Recalculate the weights for each particle
            self.calculate_weights()

            # Resample the cloud
            self.pf.resample()

            # Update map -> odom tf
            (mean, _) = self.pf.update()

            # Update transforms between frames
            self.update_transforms(mean)

            # Redistribute a percentage of the particles to prevent converging on an incorrect solution
            # self.pf.particles[
            #     np.linspace(
            #         0,
            #         self.pf.n_particles - 1,
            #         int(self.pf.n_particles * 0.05),
            #         dtype=int,
            #     )
            # ] = self.pf.rng.uniform(
            #     [-2.0, -2.0, 0.0],
            #     [2.0, 2.0, 3.14 * 2.0],
            #     size=(int(self.pf.n_particles * 0.05), 3),
            # )

            # Publish the synthetic scan predicted from the map
            self.publish_gt_scan(mean)

            # Predict the motion of the rover
            self.pf.predict(rospy.Time.now().to_sec() - self.last_stamp.to_sec())

            # Publish our particles for visualization
            self.publish_cloud()

    def calculate_weights(self):
        self.mutex.acquire(blocking=True)
        ranges = np.array(self.scan.ranges)
        self.mutex.release()

        subsampled_ranges = ranges[
            np.linspace(0, 359, self.lidar_subsample_count, dtype=int)
        ]

        for idx, particle in enumerate(self.pf.particles):
            # Extract ground truth from the map
            map_ranges = gen_lidar_scan(particle, self.map, self.lidar_subsample_count)

            # Set a new weight for the particle
            self.pf.weights[idx] = self.pf.sensor_model(
                subsampled_ranges, np.array(map_ranges), self.lidar_std
            )

        if np.all(self.pf.weights == 0.0):
            self.pf.weights = np.full(self.pf.weights.shape, 1.0)

        self.pf.weights /= np.sum(self.pf.weights)

    def publish_cloud(self):
        # Turn particles into a ROS PoseArray message that Foxglove can display
        particle_poses = PoseArray()
        for particle in self.pf.particles:
            pose = Pose()
            pose.position = Point(particle[0], particle[1], 0.05)
            pose.orientation = Quaternion(*quaternion_from_euler(0.0, 0.0, particle[2]))
            particle_poses.poses.append(pose)
        particle_poses.header.stamp = rospy.Time.now()
        particle_poses.header.frame_id = "map"

        # Publish the particles
        self.pose_array_publisher.publish(particle_poses)

    def update_transforms(self, mean):
        # Calculate and publish map -> odom tranformation
        # In accordance with REP-105 https://www.ros.org/reps/rep-0105.html

        mean_to_map_transform = tf.transformations.inverse_matrix(
            tf.transformations.concatenate_matrices(
                tf.transformations.translation_matrix([mean[0], mean[1], 0.0]),
                tf.transformations.quaternion_matrix(
                    quaternion_from_euler(0.0, 0.0, mean[2])
                ),
            )
        )

        (trans, rot) = self.tf_listener.lookupTransform("odom", "lidar", rospy.Time(0))
        trans[2] = 0.0
        odom_to_mean_tranform = tf.transformations.concatenate_matrices(
            tf.transformations.translation_matrix(trans),
            tf.transformations.quaternion_matrix(rot),
        )

        inverse_tranform = tf.transformations.inverse_matrix(
            odom_to_mean_tranform @ mean_to_map_transform
        )

        self.tf_broadcaster.sendTransform(
            tf.transformations.translation_from_matrix(inverse_tranform),
            tf.transformations.quaternion_from_matrix(inverse_tranform),
            rospy.Time.now(),
            "odom",
            "map",
        )

    def publish_gt_scan(self, mean):
        # Publish the scan the model expects to be getting from its prediction
        fake_scan = LaserScan()
        fake_scan.header.frame_id = "lidar"
        fake_scan.header.stamp = rospy.Time.now()
        fake_scan.angle_min = -3.14
        fake_scan.angle_max = 3.14
        fake_scan.angle_increment = (3.14 * 2.0) / 360.0
        fake_scan.range_min = 0.1
        fake_scan.range_max = 8.0
        fake_scan.ranges = gen_lidar_scan(mean, self.map, 360)
        fake_scan.intensities = np.full(360, 1.0)
        self.gt_publisher.publish(fake_scan)

    def get_initial_distribution(self):
        rng = np.random.default_rng()
        if self.initial_pose is None:
            particles = rng.uniform(
                [-2.0, -2.0, 0.0], [2.0, 2.0, 3.14 * 2.0], size=(self.num_particles, 3)
            )
        else:
            particles = rng.normal(
                [
                    self.initial_pose.pose.pose.position.x,
                    self.initial_pose.pose.pose.position.y,
                    tf.transformations.euler_from_quaternion(
                        [
                            self.initial_pose.pose.pose.orientation.x,
                            self.initial_pose.pose.pose.orientation.y,
                            self.initial_pose.pose.pose.orientation.z,
                            self.initial_pose.pose.pose.orientation.w,
                        ],
                    )[2],
                ],
                [0.05, 0.05, 3.14 / 12],
                size=(self.num_particles, 3),
            )
            self.reset_distribution = False
        return particle_filter(
            particles, np.full(self.num_particles, 1 / self.num_particles)
        )

    def map_callback(self, msg: OccupancyGrid):
        self.map = msg

    def scan_callback(self, msg: LaserScan):
        self.mutex.acquire(blocking=True)
        self.scan = msg
        self.mutex.release()

    def initial_pose_callback(self, msg: PoseWithCovarianceStamped):
        self.initial_pose = msg
        self.reset_distribution = True


def main(args=None):
    # init the node
    rospy.init_node("particle_filter", anonymous=True)

    _ = pf_node()

    # let ROS handle ROS things
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
