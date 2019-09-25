#!/usr/bin/python
import rospy

from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

import tf2_ros as tf2
from tf.transformations import euler_from_quaternion


def build_twist_msg():
    twist_msg = TwistStamped()
    twist_msg.header.frame_id = "base_link"
    twist_msg.header.stamp = rospy.Time().now()
    return twist_msg


def translate_yaw_cb(msg, angular_twist_pub):
    twist_msg = build_twist_msg()
    twist_msg.twist.angular.z = msg.data
    angular_twist_pub.publish(twist_msg)


def translate_velocity_cb(msg, linear_twist_pub):
    twist_msg = build_twist_msg()
    twist_msg.twist.linear.x = msg.data
    linear_twist_pub.publish(twist_msg)


def odom_cb(msg, publishers):
    vel_state_pub, angle_state_pub = publishers
    rotation = msg.pose.pose.orientation

    quat = [rotation.x, rotation.y, rotation.z, rotation.w]

    _, _, yaw = euler_from_quaternion(quat)

    angle_state_pub.publish(yaw)

    speed = msg.twist.twist.linear.x
    vel_state_pub.publish(speed)


if __name__ == '__main__':
    rospy.init_node('translator')

    linear_translate_topic = rospy.get_param('~linear_ctl_topic',
                                             'control/translate_velocity')
    angular_translate_topic = rospy.get_param('~angular_ctl_topic',
                                              'control/translate_angle')

    linear_driver_topic = rospy.get_param('~linear_driver_topic',
                                          'control/linear_cmd')
    angular_driver_topic = rospy.get_param('~angular_driver_topic',
                                           'control/angular_cmd')

    odom_topic = rospy.get_param('~odom_topic',
                                 '/wamv/robot_localization/odometry/filtered')
    angle_state_pub_topic = rospy.get_param('~angle_state_pub_topic',
                                            'control/angle_measurements')
    vel_state_pub_topic = rospy.get_param('~vel_state_pub_topic',
                                          'control/vel_measurements')

    state_pub_rate = rospy.get_param('~state_pub_rate', 60.)

    linear_twist_pub = rospy.Publisher(
        linear_driver_topic, TwistStamped, queue_size=1)
    linear_sub = rospy.Subscriber(
        linear_translate_topic,
        Float64,
        callback=translate_velocity_cb,
        callback_args=(linear_twist_pub))

    angular_twist_pub = rospy.Publisher(
        angular_driver_topic, TwistStamped, queue_size=1)
    angle_sub = rospy.Subscriber(
        angular_translate_topic,
        Float64,
        callback=translate_yaw_cb,
        callback_args=(angular_twist_pub))

    vel_state_pub = rospy.Publisher(vel_state_pub_topic, Float64, queue_size=1)
    angle_state_pub = rospy.Publisher(
        angle_state_pub_topic, Float64, queue_size=1)

    odom_sub = rospy.Subscriber(
        odom_topic,
        Odometry,
        callback=odom_cb,
        callback_args=(vel_state_pub, angle_state_pub))

    rospy.spin()
