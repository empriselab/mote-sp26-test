import rospy

import tf

import math

from std_msgs.msg import Header
from geometry_msgs.msg import TwistStamped, Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf.transformations


class odometry:
    def __init__(self):
        self.odometry_msg = Odometry()
        self.odometry_msg.header = Header(stamp=rospy.Time.now(), frame_id="odom")
        # base_footprint is base_link but on the ground
        self.odometry_msg.child_frame_id = "base_footprint"

        # publish messages of type Odometry on the /odom topic
        self.odometry_publisher = rospy.Publisher("odom", Odometry, queue_size=10)

        self.last_twist_msg_time = 0

        self.last_twist = Twist()

        # subscribe to TwistStamped messages on the /cmd_vel_ground_truth topic, and register twist_callback as the callback
        self.twist_subscription = rospy.Subscriber(
            "cmd_vel_ground_truth", TwistStamped, self.twist_callback
        )

    def twist_callback(self, msg: TwistStamped):
        if self.last_twist_msg_time == 0:
            self.last_twist_msg_time = msg.header.stamp
            self.last_twist = msg.twist
            return

        dt = (msg.header.stamp - self.last_twist_msg_time).to_sec()

        # we do our calculations based on the last twist, not the one we just recieved
        linear_velocity = self.last_twist.linear.x
        angular_velocity = self.last_twist.angular.z

        """
        QUESTION 2.1 BEGINS
        """
        # the Odometry message type is a bit more complicated than Twist or JointState
        # read through the specification carefully: https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
        # you can click on subtypes (PoseWithCovariance, TwistWithCovariance, ect) to read their spec

        # you don't need to assign anything to the covariance values (for now)

        # pose orientation is stored as a quaternion
        # working with quaternions can be difficult, read:
        # https://wiki.ros.org/tf2/Tutorials/Quaternions

        # set the twist in the self.odometry_msg, then ...
        self.odometry_msg.twist.twist = msg.twist
        orientation_list = [
            self.odometry_msg.pose.pose.orientation.x,
            self.odometry_msg.pose.pose.orientation.y,
            self.odometry_msg.pose.pose.orientation.z,
            self.odometry_msg.pose.pose.orientation.w
        ]

        # ... update the pose orientation, then ...
        angles = tf.transformations.euler_from_quaternion(orientation_list)
        yaw = angles[2] + angular_velocity * dt
        q_rot = tf.transformations.quaternion_from_euler(angles[0], angles[1], yaw)

        # ... update the pose position, then ...
        dx = linear_velocity * dt * math.cos(yaw)
        dy = linear_velocity * dt * math.sin(yaw)
        self.odometry_msg.pose.pose.position.x += dx
        self.odometry_msg.pose.pose.position.y += dy
        self.odometry_msg.pose.pose.orientation = Quaternion(*q_rot)

        # ... update the message header, then ...
        self.odometry_msg.header = Header(stamp=rospy.Time.now(), frame_id="odom")
        # ... publish the message
        self.odometry_publisher.publish(self.odometry_msg)

        """ 
        QUESTION 2.1 ENDS
        """

        """
        QUESTION 2.2 BEGINS
        """
        # you may find this tutorial useful: https://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28Python%29

        # create a TransformBrodcaster, then ...
        br = tf.TransformBroadcaster()

        # ... brodcast a transform from "odom" to "base_link"
        br.sendTransform(
            (self.odometry_msg.pose.pose.position.x, self.odometry_msg.pose.pose.position.y, 0),
            (self.odometry_msg.pose.pose.orientation.x, self.odometry_msg.pose.pose.orientation.y, self.odometry_msg.pose.pose.orientation.z, self.odometry_msg.pose.pose.orientation.w),
            rospy.Time.now(),
            "base_link",
            "odom"
        )

        """
        QUESTION 2.2 ENDS
        """

        self.last_twist_msg_time = msg.header.stamp
        self.last_twist = msg.twist


def main(args=None):
    # init the node
    rospy.init_node("odometry", anonymous=True)
    _ = odometry()

    # let ROS handle ROS things
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
