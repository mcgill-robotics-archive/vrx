import rospy
from numpy import interp

from geometry_msgs.msg import TwistStamped
from control.msg import ThrusterCmd


class DifferentialDrive(object):
    def __init__(self, sub_topic, pub_topic):
        self.min_pwm = rospy.get_param('~min_pwm', 1100)
        self.max_pwm = rospy.get_param('~max_pwm', 1800)
        self.zero_pwm = rospy.get_param('~neutral_pwm', 1500)
        self.linfac = rospy.get_param('~linear_scaling', 20)
        self.angfac = rospy.get_param('~angular_scaling', 8)

        self.min = (-0.3 * self.linfac) + (-0.2 * self.angfac)
        self.max = (0.3 * self.linfac) + (0.2 * self.angfac)

        self.sub_range = [self.min, self.max]
        self.pub_range = [self.min_pwm, self.max_pwm]

        self.sub = rospy.Subscriber(sub_topic,
                                    TwistStamped,
                                    self.twist_cb,
                                    queue_size=1)

        self.pub = rospy.Publisher(pub_topic,
                                   ThrusterCmd,
                                   queue_size=1)

        self.msg = ThrusterCmd()

        rospy.loginfo('\nDiff. drive controller started')

    def twist_cb(self, data):
        rospy.logdebug('DiffDrive called ' + rospy.get_caller_id())
        rospy.logdebug('\tlinear:\t%'.format(data.twist.linear[0]))
        rospy.logdebug('\tangular:\t%'.format(data.twist.angular[2]))

        port = self.linfac * data.twist.linear[0] + \
               self.angfac * data.twist.angular[2]
        star = self.linfac * data.twist.linear[0] - \
               self.angfac * data.twist.angular[2]

        port = interp(port, self.sub_range, self.pub_range)
        star = interp(star, self.pub_range, self.pub_range)

        self.msg.port = max(self.min_pwm, min(self.max_pwm, port))
        self.msg.starboard = max(self.min_pwm, min(self.max_pwm, star))

        self.pub.publish(self.msg)

