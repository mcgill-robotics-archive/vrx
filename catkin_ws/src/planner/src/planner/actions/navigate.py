#!/usr/bin/env python

import actionlib
import math
import rospy
import tf

from geometry_msgs.msg import PoseStamped
from navigation.srv import GeoToUTM
from planner.msg import NavigateAction, NavigateFeedback, NavigateResult
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion


class NavigateServer(object):
	NAME = 'navigate_action'
	RATE = 1000

	def __init__(self, add_wp_topic, clr_wp_topic):
		self._feedback = NavigateFeedback()
		self._result = NavigateResult()
		self._name = self.NAME

		self._listener = tf.TransformListener()

		self.add_pub = rospy.Publisher(add_wp_topic, PoseStamped, queue_size=1)
		self.clr_pub = rospy.Publisher(clr_wp_topic, Bool, queue_size=1)

		self.geo_to_utm = rospy.ServiceProxy('/geo_to_utm', GeoToUTM)

		self.server = actionlib.SimpleActionServer(
			self._name, NavigateAction, self.execute, False)

		self.final_pose = PoseStamped()
		self.rate = rospy.Rate(self.RATE)
		self.success = True

		self.server.start()

	def execute(self, goal):
		# Clear current waypoints
		self.clr_pub.publish(True)

		# Set the new goal
		for pose in goal.poses:
			# Convert GeoPose to Pose
			try:
				pose_utm = self.geo_to_utm(pose)
				self.add_pub.publish(pose_utm.pose)
				self.final_pose = pose_utm.pose # only last will be kept

			except rospy.ServiceException, e:
				rospy.logerr('Service call to /geo_to_utm failed: {}'.format(e))

		stable_counts = 0
		while stable_counts < goal.stable_counts_ms:
			if self.server.is_preempt_requested():
				rospy.loginfo('%s was preempted' % self._name)
				self.clr_pub(True)
				self.server.set_preempted()
				self.success = False
				break

			try:
				trans, rot = self._listener.lookupTransform(
					'utm', 'wamv/base_link', rospy.Time(0))

				(x, y, z) = trans
				(roll, pitch, yaw) = euler_from_quaternion(rot)

				diff_x = self.final_pose.pose.position.x - x
				diff_y = self.final_pose.pose.position.y - y

				lin_err = math.sqrt((diff_x*diff_x) + (diff_y*diff_y))
				# TODO calculate angular error

				self._feedback.lin_error = lin_err
				self._feedback.ang_error = 0
				self.server.publish_feedback(self._feedback)

				if (lin_err < goal.acceptance_radius):
					stable_counts = stable_counts + 1

			except (tf.LookupException,
					tf.ConnectivityException,
					tf.ExtrapolationException) as e:
				rospy.logdebug(e.message)

			self.rate.sleep()

		self._result.success = self.success
		rospy.loginfo('{} result: {}'.format(self._name, self.success))
		self.server.set_succeeded(self._result)


# class NavigateState(smach.SimpleActionState):
# 	"""
# 	Smach state that maintains course towards waypoint
# 	"""
# 	def __init__(self):
# 		outcomes = ['succeeded', 'failed', 'drifted', 'preempted']
# 		smach.SimpleActionState.__init__(self, outcomes=outcomes)

# 	def execute(self, userdata):
# 		rospy.logdebug('[Station Keeping] Executing navigate state')
# 		return 'succeeded'


if __name__ == '__main__':
	rospy.init_node(NavigateServer.NAME)

	add_wp_topic = rospy.get_param('~/add_wp_topic', '/control/push_waypoint')
	clr_wp_topic = rospy.get_param('~/clr_wp_topic', '/control/clear_waypoints')

	server = NavigateServer(add_wp_topic, clr_wp_topic)
	rospy.spin()
