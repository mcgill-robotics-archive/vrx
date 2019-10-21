import rospy
import smach
import smach_ros

class Direct(smach.State):
	"""
	Smach state for redirecting the boat towards a waypoint
	"""
	def __init__(self):
		outcomes = ['succeeded', 'failed', 'preempted']
		smach.State.__init__(self, outcomes=outcomes)

	def execute(self, userdata):
		rospy.logdebug('[Station Keeping] Executing direct state')
		return 'succeeded'

class Navigate(smach.State):
	"""
	Smach state that maintains course towards waypoint
	"""
	def __init__(self):
		outcomes = ['succeeded', 'failed', 'drifted', 'preempted']
		smach.State.__init__(self, outcomes=outcomes)

	def execute(self, userdata):
		rospy.logdebug('[Station Keeping] Executing navigate state')
		return 'succeeded'

class Hold(smach.State):
	"""
	Smach state for holding the station-keeping pose
	"""
	def __init__(self):
		outcomes = ['succeeded', 'failed', 'drifted', 'preempted']
		smach.State.__init__(self, outcomes=outcomes)

	def execute(self, userdata):
		rospy.logdebug('[Station Keeping] Executing hold state')
		return 'succeeded'

class StationKeeping(object):
	def __init__(self):
		outcomes = ['succeeded', 'failed', 'preempted']
		self.sm = smach.StateMachine(outcomes=outcomes)

		with self.sm:
			smach.StateMachine.add(
				'DIRECT', 
				Direct(),
				{'succeeded':'NAVIGATE',
				 'failed':'failed',
				 'preempted':'preempted'})
			smach.StateMachine.add(
				'NAVIGATE',
				Navigate(),
				{'succeeded':'HOLD',
				 'failed':'failed',
				 'drifted':'DIRECT',
				 'preempted':'preempted'})
			smach.StateMachine.add(
				'HOLD',
				Hold(),
				{'succeeded':'succeeded',
				 'failed':'failed',
				 'drifted':'DIRECT',
				 'preempted':'preempted'})

	def get_state_machine(self):
		return self.sm
