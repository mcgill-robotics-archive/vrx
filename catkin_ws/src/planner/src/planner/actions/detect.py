#!/usr/bin/env python

import actionlib
import rospy
import tf

from geodesy.utm import UTMPoint
from geographic_msgs.msg import GeoPoseStamped
from perception.srv import DetectObjects
from planner.msg import DetectAction, DetectFeedback, DetectResult
from std_msgs.msg import Header


class DetectServer(object):
	NAME = 'detect_action'
	RATE = 0.4

	def __init__(self, landmark_topic):
		self._feedback = DetectFeedback()
		self._result = DetectResult()
		self._name = self.NAME

		self.zone = 0
		self.band = ''

		self._listener = tf.TransformListener()

		self.detect_srv = rospy.ServiceProxy(
			'/perception/object_detection_server/detect_objects', DetectObjects)

		self.landmark_pub = rospy.Publisher(
			landmark_topic, GeoPoseStamped, queue_size=1)

		self.server = actionlib.SimpleActionServer(
			self._name, DetectAction, self.execute, False)

		self.rate = rospy.Rate(self.RATE)
		self.success = True

		self.server.start()

	def execute(self, goal):
		self.zone = goal.zone
		self.band = goal.band
		rospy.loginfo(
			'Updated zone ({}) and band ({})'.format(self.zone, self.band))

		trial_count = 0
		while trial_count < goal.attempted_trials:
			if self.server.is_preempt_requested():
				rospy.loginfo('%s was preempted' % self._name)
				self.clr_pub(True)
				self.server.set_preempted()
				self.success = False
				break

			try:
				landmarks = self.detect_srv()

				try:
					trans, rot = self._listener.lookupTransform(
						'wamv/lidar_wamv_link', 'utm', rospy.Time(0))

					(x, y, z) = trans
					(roll, pitch, yaw) = euler_from_quaternion(rot)

					for lankmark in landmarks:
						point = UTMPoint(landmark.pose.position.x + x,
										 landmark.pose.position.y + y,
										 zone=self.zone,
										 band=self.band).toMsg()

						msg = GeoPoseStamped()
						msg.header = Header()
						msg.pose.position = point
						self.landmark_pub.publish(msg)

					trial_count = trial_count + 1

				except (tf.LookupException,
						tf.ConnectivityException,
						tf.ExtrapolationException) as e:
					rospy.logdebug(e.message)

			except rospy.ServiceException, e:
				rospy.logerr('Service call to /geo_to_utm failed: {}'.format(e))

			self.rate.sleep()

		self._result.success = self.success
		rospy.loginfo('{} result: {}'.format(self._name, self.success))
		self.server.set_succeeded(self._result)


if __name__ == '__main__':
	rospy.init_node(DetectServer.NAME)

	landmark_topic = rospy.get_param(
		'~/landmark_topic', '/vrx/perception/landmark')

	server = DetectServer(landmark_topic)
	rospy.spin()
