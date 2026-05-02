PKG = "hw2_pkg"
NAME = "ik_tests"

import sys, unittest, time

# from gradescope_utils.autograder_utils.decorators import weight

import rospy, rostest
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

from hw2_pkg.inverse_kinematics import inv_kin


class TestIK(unittest.TestCase):
    # this is called when initializing the test class
    @classmethod
    def setUpClass(cls):
        cls.ik = inv_kin()

        # create a publisher to send fake data
        cls.twist_publisher = rospy.Publisher("diff_drive_controller/cmd_vel", Twist, queue_size=10)

        rospy.sleep(0.5)  # wait for publishers to init

    # this is called before each test fixture
    def setUp(self):
        self.ik = inv_kin()

        self.joint_state_recieved = None

    def __init__(self, *args):
        # create a subscriber to check results
        rospy.Subscriber("joint_states_predicted", JointState, self.callback)

        super(TestIK, self).__init__(*args)

    def wait_for_data(self):
        # 2 second timeout
        timeout_t = time.time() + 2.0

        # Wait for the callback to get hit
        while (
            self.joint_state_recieved == None
            and not rospy.is_shutdown()
            and time.time() < timeout_t
        ):
            rospy.sleep(0.1)

        # Check for timeout
        self.assertNotEqual(
            self.joint_state_recieved,
            None,
            msg="JointState not recieved before timeout",
        )

        data = self.joint_state_recieved
        self.joint_state_recieved = None

        return data

    # @weight(2.5)
    def test_no_motion(self):
        twist = Twist()
        twist.angular.z = 0
        twist.linear.x = 0

        self.twist_publisher.publish(twist)

        joint_state = self.wait_for_data()

        self.assertAlmostEquals(
            joint_state.velocity[0], 0.0, msg="Left wheel velocity incorrect"
        )
        self.assertAlmostEquals(
            joint_state.velocity[1], 0.0, msg="Right wheel velocity incorrect"
        )

    # @weight(2.5)
    def test_angular_only(self):
        twist = Twist()
        twist.angular.z = 1.0
        twist.linear.x = 0.0

        self.twist_publisher.publish(twist)

        joint_state = self.wait_for_data()

        self.assertAlmostEquals(
            joint_state.velocity[0],
            -2.2965879265091864,
            msg="Left wheel velocity incorrect",
        )
        self.assertAlmostEquals(
            joint_state.velocity[0],
            joint_state.velocity[1],
            msg="Right wheel velocity incorrect",
        )

    # @weight(2.5)
    def test_linear_only(self):
        twist = Twist()
        twist.angular.z = 0.0
        twist.linear.x = 1.0

        self.twist_publisher.publish(twist)

        joint_state = self.wait_for_data()

        self.assertAlmostEquals(
            joint_state.velocity[0],
            32.808398950131235,
            msg="Left wheel velocity incorrect",
        )
        self.assertAlmostEquals(
            joint_state.velocity[0],
            -joint_state.velocity[1],
            msg="Right wheel velocity incorrect",
        )

    # @weight(2.5)
    def test_linear_and_angular(self):
        twist = Twist()
        twist.angular.z = 3.0
        twist.linear.x = 1.0

        self.twist_publisher.publish(twist)

        joint_state = self.wait_for_data()

        self.assertAlmostEquals(
            joint_state.velocity[0],
            25.918635170603675,
            msg="Left wheel velocity incorrect",
        )
        self.assertAlmostEquals(
            joint_state.velocity[1],
            -39.69816272965879,
            msg="Right wheel velocity incorrect",
        )

    def callback(self, msg: JointState):
        self.joint_state_recieved = msg


if __name__ == "__main__":
    rospy.init_node(NAME, anonymous=True)

    rostest.rosrun(PKG, NAME, TestIK, sys.argv)
