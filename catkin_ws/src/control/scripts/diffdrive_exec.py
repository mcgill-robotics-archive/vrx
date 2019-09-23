#!/usr/bin/env python

import rospy
from control import DifferentialDrive

if __name__ == '__main__':
    rospy.init_node('differential_drive')

    sub_topic = rospy.get_param('~sub_topic', 'control/twist')
    pub_topic = rospy.get_param('~pub_topic', 'control/thrusters')

    diff_drive = DifferentialDrive(sub_topic, pub_topic)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass