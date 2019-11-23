import rospy
import smach
import smach_ros

from geodesy.utm import gridZone
from planner.utils import wait_for_message
from planner.msg import DetectAction, DetectGoal
from geographic_msgs.msg import GeoPath
from sensor_msgs.msg import NavSatFix


class Perception(object):
	GPS_SAMPLE_TOPIC = '/wamv/sensors/gps/gps/fix'
	GPS_SAMPLE_TIMEOUT = 5

	PERCEPTION_ATTEMPED_TRIALS = 999 # Placeholder because whatever...

	def __init__(self):
		self.goal = DetectGoal()
		self.goal.detect = True

		self.zone = 0
		self.band = ''

		outcomes = ['succeeded', 'failed', 'preempted']
		self.sm = smach.StateMachine(outcomes=outcomes)

		try:
			rospy.loginfo('Perception: Waiting for gps sample...')
			res = wait_for_message(self.GPS_SAMPLE_TOPIC,
								   NavSatFix,
								   self.GPS_SAMPLE_TIMEOUT)
			
			self.zone, self.band = gridZone(res.latitude, res.longitude)
			self.goal.zone = self.zone
			self.goal.band = self.band
			self.goal.attempted_trials = self.PERCEPTION_ATTEMPED_TRIALS

		except Exception as e:
			rospy.logwarn('Timed out waiting for goal... {}'.format(e))
			return

		with self.sm:
			smach.StateMachine.add(
				'DETECT',
				smach_ros.SimpleActionState('detect_action', 
											DetectAction,
											goal=self.goal),
				{'succeeded':'succeeded',
				 'aborted':'failed',
				 'preempted':'preempted'})

	def get_state_machine(self):
		return self.sm
