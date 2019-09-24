#!/usr/bin/env python

import rospy
from control import DifferentialDrive

if __name__ == '__main__':
    rospy.init_node('differential_drive')

    linear_cmd_topic = rospy.get_param('~linear_cmd_topic',
                                       'control/linear_cmd')
    angular_cmd_topic = rospy.get_param('~angular_cmd_topic',
                                        'control/angular_cmd')
    left_thruster_topic = rospy.get_param('~left_thruster_topic',
                                          '/wamv/thrusters/left_thrust_cmd')
    right_thruster_topic = rospy.get_param('~right_thruster_topic',
                                           '/wamv/thrusters/right_thrust_cmd')

    diff_drive = DifferentialDrive(linear_cmd_topic, angular_cmd_topic,
                                   left_thruster_topic, right_thruster_topic)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
