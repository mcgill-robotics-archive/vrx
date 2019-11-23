import rospy
import smach
import smach_ros

from planner.utils import wait_for_message
from planner.msg import NavigateAction, NavigateGoal
from geographic_msgs.msg import GeoPoseStamped


class StationKeeping(object):
	STATION_KEEPING_GOAL_TOPIC = '/vrx/station_keeping/goal'
	STATION_KEEPING_TIMEOUT = 5

	def __init__(self):
		self.goal = NavigateGoal()

		outcomes = ['succeeded', 'failed', 'preempted']
		self.sm = smach.StateMachine(outcomes=outcomes)

		try:
			rospy.loginfo('Station Keeping: Waiting for goal...')
			res = wait_for_message(self.STATION_KEEPING_GOAL_TOPIC,
								   GeoPoseStamped,
								   self.STATION_KEEPING_TIMEOUT)
			
			self.goal.poses.append(res.pose)
			self.goal.acceptance_radius = 0.01
			self.goal.stable_counts_ms = 30000

		except Exception as e:
			rospy.logwarn('Timed out waiting for goal... {}'.format(e))
			return

		with self.sm:
			# smach.StateMachine.add(
			# 	'DIRECT', 
			# 	Direct(),
			# 	{'succeeded':'NAVIGATE',
			# 	 'aborted':'failed',
			# 	 'preempted':'preempted'})
			smach.StateMachine.add(
				'NAVIGATE',
				smach_ros.SimpleActionState('navigate_action', 
											NavigateAction,
											goal=self.goal),
				{'succeeded':'succeeded',
				 'aborted':'failed',
				 'preempted':'preempted'})
			# smach.StateMachine.add(
			# 	'HOLD',
			# 	Hold(),
			# 	{'succeeded':'succeeded',
			# 	 'aborted':'failed',
			# 	 'preempted':'preempted'})

	def get_state_machine(self):
		return self.sm
