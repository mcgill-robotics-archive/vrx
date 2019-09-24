#!/usr/bin/python

import rospy

from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import PoseStamped
from control.srv import Waypoint

import tf2_ros as tf2
from tf.transformations import euler_from_quaternion

from math import atan2, pi

from threading import Lock


def wrap_angle(angle):
    return (angle + pi) % (2 * pi) - pi


def yaw_from_quat(quat):
    quat = [quat.x, quat.y, quat.z, quat.w]

    _, _, yaw = euler_from_quaternion(quat)
    return yaw


class WaypointController(object):
    def __init__(self, acceptance_radius, default_speed, publish_rate,
                 speed_ctl_topic, angle_ctl_topic, waypoint_topic):
        """
        Creates a WaypointController that will take waypoints and translate
        them to yaw commands for the thrust controllers. Should be used by
        a ROS node that has called init_node.

        Args:
            acceptance_radius(float) : Max acceptance radius for the waypoint.a
            default_speed(float): Default cruise speed.
            publish_rate(float) : Rate to publish setpoints at.
            speed_ctl_topic(string) : Topic to send forward velocity setpoints
                                      to.
            angle_ctl_topic(string) : Topic to send yaw setpoints to.
            waypoint_topic(string) : Topic to accept waypoint commands at.
        """
        self.acceptance_radius = acceptance_radius
        self.default_speed = default_speed

        self.speed_ctl_pub = rospy.Publisher(
            speed_ctl_topic, Float64, queue_size=1)
        self.angle_ctl_pub = rospy.Publisher(
            angle_ctl_topic, Float64, queue_size=1)
        self.wp_srv = rospy.Subscriber(
            waypoint_topic, PoseStamped, self.set_waypoint, queue_size=1)

        self.current_waypoint = None
        self._wp_lock = Lock()

        self.buffer = tf2.Buffer()
        self.listener = tf2.TransformListener(self.buffer)

        self.publish_timer = rospy.Timer(
            rospy.Duration(1. / publish_rate), self.publish)

        self.publish_timer.run()

    def publish(self, stamp):
        with self._wp_lock:
            if self.current_waypoint:
                self.pub_cmd()

    def pub_cmd(self):
        try:
            tr = self.buffer.lookup_transform('wamv/odom', 'wamv/base_link',
                                              rospy.Time())
        except (tf2.LookupException, tf2.ConnectivityException,
                tf2.ExtrapolationException) as e:
            rospy.logwarn_throttle(1, str(e))
            return

        position = tr.transform.translation

        dist_to_current_wp = ((self.current_waypoint.x - position.x)**2 +
                              (self.current_waypoint.y - position.y)**2)

        if dist_to_current_wp < self.acceptance_radius**2:
            speed_target = 0.
            angle_to_pnt = self.wp_yaw  # Align with the waypoint
        else:
            speed_target = self.default_speed

            angle_to_pnt = yaw_from_quat(tr.transform.rotation)
            if self.current_waypoint:
                wp_pos = self.current_waypoint
                angle_to_pnt = atan2(wp_pos.y - position.y,
                                     wp_pos.x - position.x)

        self.speed_ctl_pub.publish(speed_target)
        self.angle_ctl_pub.publish(angle_to_pnt)

    def set_waypoint(self, msg):
        with self._wp_lock:
            self.current_waypoint = msg.pose.position
            self.wp_yaw = yaw_from_quat(msg.pose.orientation)

        return True


if __name__ == '__main__':
    rospy.init_node('waypoint_controller')

    acceptance_radius = rospy.get_param('~acceptance_radius', 5)
    cruise_speed = rospy.get_param('~cruise_speed', 10)

    angle_ctl_topic = rospy.get_param('~ctl_topic', '/angular_pid/setpoint')
    speed_ctl_topic = rospy.get_param('~ctl_topic', '/linear_pid/setpoint')
    set_waypoint_topic = rospy.get_param('~set_waypoint_topic',
                                         '/control/set_waypoint')
    publish_rate = rospy.get_param('~setpoint_pub_rate', 20)

    waypoint_ctl = WaypointController(acceptance_radius, cruise_speed,
                                      publish_rate, speed_ctl_topic,
                                      angle_ctl_topic, set_waypoint_topic)
    rospy.spin()
