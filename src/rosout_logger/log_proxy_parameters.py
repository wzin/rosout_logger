#!/usr/bin/env python

import rospy

class LogProxyParameters():
    def __init__(self):
        rospy.loginfo("Initializing %s" % self.__class__.__name__)
        self.syslog_facility = rospy.get_param('~syslog_facility', 'local7')
        self.ros_log_source = rospy.get_param('~ros_log_source', '/rosout_agg')
        self.respect_severity = rospy.get_param('~respect_severity', True)
        self.log_port = rospy.get_param('~log_port', 514)
        self.log_host = rospy.get_param('~respect_severity', 'localhost')
        self.include_seq = rospy.get_param('~include_seq', True)
        self.include_ros_node_name = rospy.get_param('~include_ros_node_name', True)
        self.include_ros_file_name = rospy.get_param('~include_ros_file_name', True)
        self.include_function_name = rospy.get_param('~include_function_name', True)
        self.include_line = rospy.get_param('~include_line', True)
        self.include_topic = rospy.get_param('~include_topic', True)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4