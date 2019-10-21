#!/usr/bin/env python3

import rospy
import actionlib

from std_msgs.msg import Bool, Float64
from planner.msg import PivotAction, PivotFeedback, PivotResult


class PivotServer(object):
	def __init__(self, angle_thresh, linear_enable_topic,
		angular_enable_topic, angular_setpoint_topic):

		self._action_name = 'pivot'
		self._feedback = PivotFeedback()
		self._result = PivotResult()

		self.angle_thresh = angle_thresh

		self.linear_enable_pub = rospy.Publisher(
			linear_enable_topic, Bool, queue_size=0)
		self.angular_enable_pub = rospy.Publisher(
			angular_enable_topic, Bool, queue_size=0)
		self.angular_setpoint_pub = rospy.Publisher(
			angular_setpoint_topic, Float64, queue_size=0)

		self._server = actionlib.SimpleActionServer(
			'pivot', PivotAction, self.execute, auto_start=False)

		self.server.start()

	def execute(self, goal):
		self.server.set_succeeded()


if __name__ == "__main__":
	rospy.init_node('pivot_action_server')

	angle_thresh = rospy.get_param(
		'~angle_thresh', 0.175)
	linear_enable_topic = rospy.get_param(
		'~linear_enable_topic', '/linear_pid/pid_enable')
	angular_enable_topic = rospy.get_param(
		'~angular_enable_topic', '/angular_pid/pid_enable')
	angular_setpoint_topic = rospy.get_param(
		'~angular_setpoint_topic', '/angular_pid/setpoint')

	server = PivotServer(angle_thresh, linear_enable_topic,
		angular_enable_topic, angular_setpoint_topic)

	rospy.spin()