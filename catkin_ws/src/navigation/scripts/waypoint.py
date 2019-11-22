#!/usr/bin/env python

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

from navigation.srv import GeoToUTM, GeoToUTMResponse

def geo_to_utm_handler(req):
	point = fromLatLong(req.geo.position.latitude,
						req.geo.position.longitude,
						req.geo.position.altitude).toPoint()

	# try:
	# 	tr = self.buffer.lookup_transform(
	# 		'earth', 'wamv/odom', rospy.Time(0))
	# except (tf2.LookupException,
	# 		tf2.ConnectivityException,
	# 		tf2.ExtrapolationException) as e:
	# 	rospy.logwarn_throttle(1, str(e))
	# 	return

	res = GeoToUTMResponse()
	res.pose.position.x = point.x
	res.pose.position.y = point.y
	res.pose.position.z = point.z
	res.pose.orientation = req.geo.orientation
	return res

if __name__ == '__main__':
	rospy.init_node('geo_to_utm_server')
	srv = rospy.Service('geo_to_utm', GeoToUTM, geo_to_utm_handler)
	rospy.loginfo('GeoPose to Pose Service is up')
	rospy.spin()
