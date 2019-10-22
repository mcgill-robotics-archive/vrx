import rospy
import numpy as np
import tf2_ros as tf2

from geodesy.utm import fromLatLong
from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import PoseStamped
from math import atan2, pi
from std_msgs.msg import Header, Float64
from tf.transformations import euler_from_quaternion
from threading import Lock
from uuid_msgs.msg import UniqueID


class WaypointManager(object):
	def __init__(self, waypoint_topic, distance_topic,
		angle_topic, publish_rate):
		
		self.target_sub = rospy.Subscriber(
			target_topic, GeoPoseStamped, self.target_cb)
		self.distance_pub = rospy.Publiser(
			distance_topic, Float64, queue_size=0)
		self.angle_pub = rospy.Publisher(
			angle_topic, Float64, queue_size=0)

		self.target_dist
		self.target_angle

	def target_cb(self, msg):
		point = fromLatLong(msg.pose.position.latitude,
							msg.pose.position.longitude,
							msg.pose.position.altitude)

		try:
			tr = self.buffer.lookup_transform(
				'earth', 'wamv/odom', rospy.Time(0))
		except (tf2.LookupException,
				tf2.ConnectivityException,
				tf2.ExtrapolationException) as e:
			rospy.logwarn_throttle(1, str(e))
			return
