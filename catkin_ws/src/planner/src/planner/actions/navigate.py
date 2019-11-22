import rospy
import actionlib

from planner.msg import NavPathAction


class NavigateServer(object):
	def __init__(self):
		self.server = actionlib.SimpleActionServer(
			'nav_path', NavPathAction, self.execute, False)

		self.server.start()

	def execute(self, goal):
		self.server.set_succeeded()
