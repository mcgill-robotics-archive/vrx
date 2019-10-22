import rospy
import smach
import smach_ros


class Placeholder(smach.State):
	"""
	Placeholder smach state that automatically succeeds
	"""
	def __init__(self):
		outcomes = ['succeeded']
		smach.State.__init__(self, outcomes=outcomes)

	def execute(self, userdata):
		rospy.logdebug('[Scan and Dock] Executing placeholder state')
		return 'succeeded'

class ScanAndDock(object):
	def __init__(self):
		outcomes = ['succeeded', 'failed', 'preempted']
		self.sm = smach.StateMachine(outcomes=outcomes)

		with self.sm:
			smach.StateMachine.add(
				'PLACEHOLDER', 
				Placeholder(),
				{'succeeded':'succeeded'})

	def get_state_machine(self):
		return self.sm
