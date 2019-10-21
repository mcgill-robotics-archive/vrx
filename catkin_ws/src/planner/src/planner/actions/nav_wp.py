import rospy
import actionlib

from planner.msg import NavWayPointAction


class NavWayPointServer(object):
	def __init__(self):
		self.server = actionlib.SimpleActionServer(
			'nav_wp', NavWayPointAction, self.execute, False)

		self.server.start()

	def execute(self, goal):
		self.server.set_succeeded()
