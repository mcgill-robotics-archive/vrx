import rospy
import smach
import smach_ros

from planner.utils import wait_for_message
from planner.msg import NavigateAction, NavigateGoal
from geographic_msgs.msg import GeoPath


class Wayfinding(object):
	WAYFINDING_GOAL_TOPIC = '/vrx/wayfinding/waypoints'
	WAYFINDING_TIMEOUT = 5

	def __init__(self):
		self.goal = NavigateGoal()

		outcomes = ['succeeded', 'failed', 'preempted']
		self.sm = smach.StateMachine(outcomes=outcomes)

		try:
			rospy.loginfo('Wayfinding: Waiting for goal...')
			res = wait_for_message(self.WAYFINDING_GOAL_TOPIC,
								   GeoPath,
								   self.WAYFINDING_TIMEOUT)

			for pose in res.poses:
				self.goal.poses.append(pose.pose)

			self.goal.acceptance_radius = 0.01
			self.goal.stable_counts_ms = 30000

		except Exception as e:
			rospy.logwarn('Timed out waiting for goal... {}'.format(e))
			return


		with self.sm:
			smach.StateMachine.add(
				'NAVIGATE',
				smach_ros.SimpleActionState('navigate_action', 
											NavigateAction,
											goal=self.goal),
				{'succeeded':'succeeded',
				 'aborted':'failed',
				 'preempted':'preempted'})

	def get_state_machine(self):
		return self.sm
