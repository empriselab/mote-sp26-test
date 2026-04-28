import rospy

from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

# this is b in your equations
WHEEL_SEPARATION = 0.14  # meters

# you'll need this to convert linear velocity into angular velocity
WHEEL_DIAMETER = 0.060960  # meters


class inv_kin:
    def __init__(self):
        # subscribe to Twist messages on the /cmd_vel topic, and register twist_callback as the callback
        self.twist_subscription = rospy.Subscriber(
            "cmd_vel", Twist, self.twist_callback
        )

        # initialize our joint states
        # what is the JointState message type?
        # read: https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html
        self.joint_states = JointState()
        self.joint_states.header = Header(stamp=rospy.Time.now(), frame_id="base_link")
        self.joint_states.name = ["wheel_left", "wheel_right"]
        self.joint_states.velocity = [0.0, 0.0]  # rad / s

        self.joint_states.position = []  # you'll leave this one empty
        self.joint_states.effort = []  # this one too

        # publish messages of type JointState on the /joint_state_predicted topic
        self.joint_state_publisher = rospy.Publisher(
            "joint_states_predicted", JointState, queue_size=10
        )

    def twist_callback(self, msg: Twist):
        # why linear.x and angular.z?
        # read: https://www.ros.org/reps/rep-0103.html#axis-orientation
        # and: https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        """
        QUESTION 2.1 BEGINS
        """
        # calculate v_left and v_right (in rad/s, not m/s!), then ...
        v_left  = (linear_velocity - angular_velocity * WHEEL_SEPARATION / 2)/(WHEEL_DIAMETER/2)
        v_right = -1*(linear_velocity + angular_velocity * WHEEL_SEPARATION / 2)/(WHEEL_DIAMETER/2)

        # ... update self.joint_states.velocity, then ...
        self.joint_states.velocity = [v_left, v_right]

        # ... update the header (see __init__), then ...
        self.joint_states.header = Header(stamp=rospy.Time.now(), frame_id="base_link")

        # ... publish the message using self.joint_state_publisher
        self.joint_state_publisher.publish(self.joint_states)
        """ 
        QUESTION 2.1 ENDS
        """


def main(args=None):
    # init the node
    rospy.init_node("inv_kin", anonymous=True)
    _ = inv_kin()

    # let ROS handle ROS things
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
