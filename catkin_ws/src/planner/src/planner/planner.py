import rospy
import smach
import smach_ros

from tasks import *
from wait_for_message import wait_for_message
from vrx_gazebo.msg import Task


class Initialize(smach.State):
	def __init__(self, task_topic, timeout):
		outcomes = ['succeeded', 'timed_out', 'preempted']
		smach.State.__init__(self, outcomes=outcomes)

		self.task_topic = task_topic
		self.timeout = timeout

	def execute(self, userdata):
		rospy.loginfo('[Global] Executing initialize state')
	
		try:
			wait_for_message(self.task_topic, Task, self.timeout)
		except Exception:
			rospy.logwarn('Initialize timed out...')
			return 'timed_out'

		return 'succeeded'


class Wait(smach.State):
	"""
	Smach state that ignores the current task until it times out
	"""

	DEFAULT_TIME = 999

	def __init__(self, task_topic, give_up_rate):
		outcomes = ['succeeded', 'failed', 'preempted']
		smach.State.__init__(self, outcomes=outcomes)

		self.task_sub = rospy.Subscriber(task_topic, Task, self.task_cb)
		self.t_remain = rospy.Duration(self.DEFAULT_TIME)
		self.rate = rospy.Rate(give_up_rate)
		self.zero = rospy.Duration(0)

	def execute(self, userdata):
		rospy.loginfo('[Global] Executing wait state')
		
		# Wait until time runs out for current task
		while (self.t_remain > self.zero):
			rospy.loginfo_throttle(
				1, 'Remaining: {}s'.format(self.t_remain / 1000))
			
			self.rate.sleep()
		
		# Once time reaches 0, return success
		return 'succeeded'

	def task_cb(self, msg):
		self.t_remain = msg.remaining_time


class ChooseTask(smach.State):
	def __init__(self, task_topic, target_tasks, choose_task_rate):
		outcomes = ['station_keeping',
					'wayfinding',
					'perception',
					'navigation_course',
					'scan',
					'scan_and_dock',
					'wait',
					'succeeded',
					'failed',
					'preempted']
		smach.State.__init__(self, outcomes=outcomes)

		self.task_sub = rospy.Subscriber(task_topic, Task, self.task_cb)

		self.target_tasks = target_tasks		
		self.rate = rospy.Rate(choose_task_rate)

		self.task = None
		self.timed_out = False

	def execute(self, userdata):
		rospy.loginfo('[Global] Executing choose task state')
		
		# Wait until viable task is published
		while ((not self.task) or (self.timed_out)):
			rospy.loginfo_throttle(2, '{} | {}'.format(self.task, self.timed_out))
			rospy.loginfo_throttle(2, 'Waiting for next task assignment...')
			self.rate.sleep()
		
		# If target tasks is empty, we've completed everything we want to try
		# Else if task is in attempted tasks, return that task
		# Otherwise return the give up state
		# UNCOMMENT THE BELOW =================================================
		# if not self.target_tasks:
		# 	return 'succeeded'
		# =====================================================================
		if self.task in self.target_tasks:
			self.target_tasks.remove(self.task)
			return self.task
		else:
			return 'wait'

	def task_cb(self, msg):
		self.task = msg.name
		self.timed_out = msg.timed_out


class Planner(object):
	def __init__(self, task_topic, attempted_tasks, init_timeout,
		choose_task_rate, give_up_rate):
		
		outcomes = ['succeeded', 'failed', 'preempted']
		self.sm = smach.StateMachine(outcomes=outcomes)

		self.station_keeping = StationKeeping().get_state_machine()
		self.wayfinding = Wayfinding().get_state_machine()
		self.perception = Perception().get_state_machine()
		self.navigation_course = NavigationCourse().get_state_machine()
		self.scan = Scan().get_state_machine()
		self.scan_and_dock = ScanAndDock().get_state_machine()

		with self.sm:
			smach.StateMachine.add(
				'INITIALIZE',
				Initialize(task_topic, init_timeout),
				{'succeeded':'CHOOSE_TASK',
				'timed_out':'failed',
				'preempted':'preempted'})
			smach.StateMachine.add(
				'WAIT',
				Wait(task_topic, give_up_rate),
				{'succeeded':'CHOOSE_TASK',
				'failed':'failed',
				'preempted':'preempted'})
			smach.StateMachine.add(
				'STATION_KEEPING',
				self.station_keeping,
				{'succeeded':'CHOOSE_TASK',
				'failed':'failed',
				'preempted':'preempted'})
			smach.StateMachine.add(
				'WAYFINDING',
				self.wayfinding,
				{'succeeded':'CHOOSE_TASK',
				'failed':'failed',
				'preempted':'preempted'})
			smach.StateMachine.add(
				'PERCEPTION',
				self.perception,
				{'succeeded':'CHOOSE_TASK',
				'failed':'failed',
				'preempted':'preempted'})
			smach.StateMachine.add(
				'NAVIGATION_COURSE',
				self.navigation_course,
				{'succeeded':'CHOOSE_TASK',
				'failed':'failed',
				'preempted':'preempted'})
			smach.StateMachine.add(
				'SCAN',
				self.scan,
				{'succeeded':'CHOOSE_TASK',
				'failed':'failed',
				'preempted':'preempted'})
			smach.StateMachine.add(
				'SCAN_AND_DOCK',
				self.scan_and_dock,
				{'succeeded':'CHOOSE_TASK',
				'failed':'failed',
				'preempted':'preempted'})
			smach.StateMachine.add(
				'CHOOSE_TASK',
				ChooseTask(task_topic, attempted_tasks, choose_task_rate),
				{'station_keeping':'STATION_KEEPING',
				'wayfinding':'WAYFINDING',
				'perception':'PERCEPTION',
				'navigation_course':'NAVIGATION_COURSE',
				'scan':'SCAN',
				'scan_and_dock':'SCAN_AND_DOCK',
				'wait':'WAIT',
				'succeeded':'succeeded',
				'failed':'failed',
				'preempted':'preempted'})

	def start(self):
		outcome = self.sm.execute()
		rospy.loginfo('State Machine exited. Result: {}'.format(outcome))
