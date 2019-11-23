import rospy
import actionlib

from planner.msg import HoldAction


class HoldServer(object):
	def __init__(self):
		self.server = actionlib.SimpleActionServer(
			'hold', HoldAction, self.execute, False)

		self.server.start()

	def execute(self, goal):
		self.server.set_succeeded()
