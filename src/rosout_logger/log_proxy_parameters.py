#!/usr/bin/env python

import rospy
import syslog

class LogProxyParameters():
    def __init__(self):
        rospy.loginfo("Initializing %s" % self.__class__.__name__)
        self.syslog_facility = self._initialize_facility()
        self.ros_log_source = rospy.get_param('~ros_log_source', '/rosout_agg')
        self.respect_severity = rospy.get_param('~respect_severity', True)
        self.log_port = rospy.get_param('~log_port', 514)
        self.log_host = rospy.get_param('~respect_severity', 'localhost')
        self.include_seq = rospy.get_param('~include_seq', True)
        self.include_ros_node_name = rospy.get_param('~include_ros_node_name', True)
        self.include_file_name = rospy.get_param('~include_ros_file_name', True)
        self.include_function_name = rospy.get_param('~include_function_name', True)
        self.include_line = rospy.get_param('~include_line', True)
        self.include_topic = rospy.get_param('~include_topic', True)
    
    def _initialize_facility(self):
        syslog_facility_string = rospy.get_param('~syslog_facility', 'local7')
        return self._get_facility_by_string(syslog_facility_string)
    
    def _get_facility_by_string(self, facility_string):
        """
        Translates human readable syslog facility name to syslog facility int understood by syslog
        e.g. str(local7) to int(184)
        defaults to "LOG_INFO"
        """
        facilities_ints = {k: v for k, v in syslog.__dict__.iteritems() if "LOG_" in k}
        canonical_facility = "LOG_%s" % facility_string.upper()
        try:
            facility_int = facilities_ints[canonical_facility]
        except KeyError:
            rospy.logwarn("Could not get provided facility for syslog - using LOGINFO")
            facility_int = 6
        
        return facility_int

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4