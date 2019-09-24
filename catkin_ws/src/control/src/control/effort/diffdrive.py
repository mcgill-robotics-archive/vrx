import rospy
import numpy as np

from math import pi

import message_filters

from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32


class DifferentialDrive(object):
    def __init__(self, linear_cmd_topic, angular_cmd_topic, left_thrust_topic,
                 right_thrust_topic):
        """
        Initializes a differential driver.
        Args:
            linear_cmd_topic: Topic to velocity commands on as TwistStamped
                              messages.
            angular_cmd_topic: Topic to recieve angular rate(yaw_rate) commands
                                on as TwistStamped messages.
            left_thrust_topic: Topic to publish left thruster outputs to.
            right_thrust_topic: Topic to publish right thruster outputs to.
        """
        self.linfac = rospy.get_param('~linear_scaling', 0.1)
        self.angfac = rospy.get_param('~angular_scaling', 10)
        self.max_thrust = rospy.get_param('~max_thrust', 50)

        linear_cmd_sub = message_filters.Subscriber(
            linear_cmd_topic, TwistStamped, queue_size=1)
        angular_cmd_sub = message_filters.Subscriber(
            angular_cmd_topic, TwistStamped, queue_size=1)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [linear_cmd_sub, angular_cmd_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.twist_cb)

        self.pub_left_thruster = rospy.Publisher(
            left_thrust_topic, Float32, queue_size=1)
        self.pub_right_thruster = rospy.Publisher(
            right_thrust_topic, Float32, queue_size=1)

    def twist_cb(self, linear, angular):
        port = self.linfac * linear.twist.linear.x - \
                self.angfac * angular.twist.angular.z

        star = self.linfac * linear.twist.linear.x + \
                self.angfac * angular.twist.angular.z

        port = np.clip(port, -self.max_thrust, self.max_thrust)
        star = np.clip(star, -self.max_thrust, self.max_thrust)

        self.pub_left_thruster.publish(port)
        self.pub_right_thruster.publish(star)
