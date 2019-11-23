#!/usr/bin/env python

import rospy

from geodesy.utm import fromLatLong
from geographic_msgs.msg import GeoPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

from navigation.srv import GeoToUTM, GeoToUTMResponse

def geo_to_utm_handler(req):
	point = fromLatLong(req.geo.position.latitude,
						req.geo.position.longitude,
						req.geo.position.altitude).toPoint()

	res = GeoToUTMResponse()
	res.pose.header = Header()
	res.pose.header.frame_id = 'utm'
	res.pose.pose.position.x = point.x
	res.pose.pose.position.y = point.y
	res.pose.pose.position.z = point.z
	res.pose.pose.orientation = req.geo.orientation
	return res

if __name__ == '__main__':
	rospy.init_node('geo_to_utm_server')
	srv = rospy.Service('geo_to_utm', GeoToUTM, geo_to_utm_handler)
	rospy.loginfo('GeoPose to Pose Service is up')
	rospy.spin()
