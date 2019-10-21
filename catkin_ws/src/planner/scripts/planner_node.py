#!/usr/bin/env python3

import rospy

from planner import Planner

if __name__ == '__main__':
	rospy.init_node('planner_node')
	mp = Planner()
	mp.start()
	rospy.spin()
