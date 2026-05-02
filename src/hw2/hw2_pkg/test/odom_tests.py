PKG = "hw2_pkg"
NAME = "odom_tests"

import sys, unittest, time

# from gradescope_utils.autograder_utils.decorators import weight

import tf

import rospy, rostest

from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, Twist
import tf.transformations

from hw2_pkg.odometry import odometry as odom_node
from hw2_pkg.forward_kinematics import forward_kin


class TestOdom(unittest.TestCase):
    # this is called when initializing the test class
    @classmethod
    def setUpClass(cls):
        cls.odometry = odom_node()
        cls.forward_kin = forward_kin()

        # create a publisher to send fake data
        cls.twist_publisher = rospy.Publisher(
            "cmd_vel_ground_truth", TwistStamped, queue_size=10
        )

        rospy.sleep(0.5)  # wait for publishers to init

    # this is called before each test fixture
    def setUp(self):
        self.odometry_recieved = None
        self.tf_listener = tf.TransformListener()

        self.odometry = odom_node()
        self.forward_kin = forward_kin()

    def __init__(self, *args):
        # create a subscriber to check results
        rospy.Subscriber("odom", Odometry, self.callback)

        super(TestOdom, self).__init__(*args)

    def wait_for_data(self):
        # 2 second timeout
        timeout_t = time.time() + 2.0

        # Wait for the callback to get hit
        while (
            self.odometry_recieved == None
            and not rospy.is_shutdown()
            and time.time() < timeout_t
        ):
            rospy.sleep(0.1)

        # Check for timeout
        self.assertNotEqual(
            self.odometry_recieved, None, msg="Odometry not recieved before timeout"
        )

        data = self.odometry_recieved
        self.odometry_recieved = None

        return data

    def check_odom_msg(
        self, odom_msg: Odometry, twist_msg: Twist, x: float, y: float, theta: float
    ):
        self.assertAlmostEquals(
            odom_msg.pose.pose.position.x, x, msg="X position is incorrect"
        )
        self.assertAlmostEquals(
            odom_msg.pose.pose.position.y, y, msg="Y position is incorrect"
        )
        self.assertAlmostEquals(
            odom_msg.pose.pose.position.z, 0.0, msg="Z should == 0.0"
        )

        quat_out = tf.transformations.euler_from_quaternion(
            [
                odom_msg.pose.pose.orientation.x,
                odom_msg.pose.pose.orientation.y,
                odom_msg.pose.pose.orientation.z,
                odom_msg.pose.pose.orientation.w,
            ]
        )

        self.assertAlmostEquals(quat_out[2], theta, msg="Computed yaw is incorrect")

        quat_in = tf.transformations.quaternion_from_euler(0, 0, theta)

        self.assertAlmostEquals(odom_msg.pose.pose.orientation.x, quat_in[0])
        self.assertAlmostEquals(odom_msg.pose.pose.orientation.y, quat_in[1])
        self.assertAlmostEquals(odom_msg.pose.pose.orientation.z, quat_in[2])
        self.assertAlmostEquals(odom_msg.pose.pose.orientation.w, quat_in[3])

        self.assertAlmostEquals(
            odom_msg.twist.twist.angular.x,
            twist_msg.angular.x,
            msg="Twist from odometry message is not correct. Make sure you're using the most recently recieved message!",
        )
        self.assertAlmostEquals(
            odom_msg.twist.twist.angular.y,
            twist_msg.angular.y,
            msg="Twist from odometry message is not correct. Make sure you're using the most recently recieved message!",
        )
        self.assertAlmostEquals(
            odom_msg.twist.twist.angular.z,
            twist_msg.angular.z,
            msg="Twist from odometry message is not correct. Make sure you're using the most recently recieved message!",
        )
        pass

    #@weight(5)
    def test_no_motion(self):
        twist = TwistStamped()
        twist.header = Header(stamp=rospy.Time(0), frame_id="base_link")
        twist.twist.linear.x = 0
        twist.twist.angular.z = 0

        self.twist_publisher.publish(twist)

        twist.header = Header(stamp=rospy.Time(1), frame_id="base_link")

        self.twist_publisher.publish(twist)
        odom = self.wait_for_data()

        self.check_odom_msg(odom, twist.twist, 0.0, 0.0, 0.0)  # x, y, theta

    #@weight(5)
    def test_straight(self):
        twist = TwistStamped()
        twist.header = Header(stamp=rospy.Time(0), frame_id="base_link")
        twist.twist.linear.x = 1
        twist.twist.angular.z = 0

        self.twist_publisher.publish(twist)

        twist.header = Header(stamp=rospy.Time(1), frame_id="base_link")

        self.twist_publisher.publish(twist)
        odom = self.wait_for_data()

        self.check_odom_msg(odom, twist.twist, 1.0, 0.0, 0.0)  # x, y, theta

    #@weight(5)
    def test_turn_on_spot(self):
        twist = TwistStamped()
        twist.header = Header(stamp=rospy.Time(0), frame_id="base_link")
        twist.twist.linear.x = 0.0
        twist.twist.angular.z = 0.5

        self.twist_publisher.publish(twist)

        twist.header = Header(stamp=rospy.Time(1), frame_id="base_link")

        self.twist_publisher.publish(twist)
        odom = self.wait_for_data()

        self.check_odom_msg(odom, twist.twist, 0.0, 0.0, 0.5)  # x, y, theta

    #@weight(5)
    def test_left_turn(self):
        twist = TwistStamped()
        twist.header = Header(stamp=rospy.Time(0), frame_id="base_link")
        twist.twist.linear.x = 1.0
        twist.twist.angular.z = 2.0

        self.twist_publisher.publish(twist)

        twist.header = Header(stamp=rospy.Time(1), frame_id="base_link")

        self.twist_publisher.publish(twist)
        odom = self.wait_for_data()

        self.check_odom_msg(
            odom,
            twist.twist,
            -0.4161468365471424,
            0.9092974268256817,
            2.0,
        )  # x, y, theta

    #@weight(5)
    def test_transform(self):
        twist = TwistStamped()
        twist.header = Header(stamp=rospy.Time(0), frame_id="base_link")
        twist.twist.linear.x = 1.0
        twist.twist.angular.z = 2.0

        self.twist_publisher.publish(twist)

        twist.header = Header(stamp=rospy.Time(1), frame_id="base_link")

        self.twist_publisher.publish(twist)
        odom = self.wait_for_data()

        rospy.sleep(1.0)

        (trans, rot) = self.tf_listener.lookupTransform(
            "/odom", "/base_link", rospy.Time(0)
        )

        rot = tf.transformations.euler_from_quaternion(rot)

        self.assertAlmostEqual(
            trans[0],
            -0.4161468365471424,
            msg=f"X position of published transform is incorrect",
        )
        self.assertAlmostEqual(
            trans[1],
            0.9092974268256817,
            msg="Y position of published transform is incorrect",
        )
        self.assertAlmostEqual(
            trans[2],
            0.0,
            msg="Z position of published transform should be 0, but it's not",
        )

        self.assertAlmostEqual(
            rot[0],
            0.0,
            msg="Pitch should be 0, but it's not",
        )
        self.assertAlmostEqual(
            rot[1],
            0.0,
            msg="Roll should be 0, but it's not",
        )
        self.assertAlmostEqual(
            rot[2],
            2.0,
            msg="Yaw of published transform is incorrect",
        )

    def callback(self, msg: Odometry):
        self.odometry_recieved = msg


if __name__ == "__main__":
    rospy.init_node(NAME, anonymous=True)

    rostest.rosrun(PKG, NAME, TestOdom, sys.argv)
