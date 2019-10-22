#!/usr/bin/env python3

import rospy

from planner import Planner

if __name__ == '__main__':
	rospy.init_node('planner_node')

	task_topic = rospy.get_param('~task_topic', '/vrx/task/info')
	attempted_tasks = rospy.get_param('~attempted_tasks')
	init_timeout = rospy.get_param('~init_timeout', 30)
	choose_task_rate = rospy.get_param('~choose_task_rate', 10)
	give_up_rate = rospy.get_param('~give_up_rate', 10)

	mp = Planner(task_topic, attempted_tasks, init_timeout,
		choose_task_rate, give_up_rate)
	mp.start()
	rospy.spin()
