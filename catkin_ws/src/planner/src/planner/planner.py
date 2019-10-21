import rospy
import smach
import smach_ros

from tasks import *


class Planner(object):
	def __init__(self):
		outcomes = ['succeeded', 'failed', 'preempted']
		self.sm = smach.StateMachine(outcomes=outcomes)

		self.station_keeping = StationKeeping().get_state_machine()

		with self.sm:
			smach.StateMachine.add(
				'STATION_KEEPING',
				self.station_keeping,
				{'succeeded':'succeeded',
				'failed':'failed',
				'preempted':'preempted'})

	def start(self):
		outcome = self.sm.execute()
