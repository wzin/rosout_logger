#!/usr/bin/env python

import rospy
import syslog
from rosgraph_msgs.msg import Log
from rosout_logger import log_proxy_parameters
from rosout_logger import syslog_handler


class LogProxy():
    def __init__(self, configuration, syslog_writer):
        rospy.loginfo("Initializing %s" % self.__class__.__name__)
        self.configuration = configuration
        self.syslog_writer = syslog_writer
    
    def _process_ros_message(self, msg):
        self._submit_to_syslog(self._deserialize_ros_log_msg(msg), msg.level)
    
    def _submit_to_syslog(self, log_entry, log_severity):
        self.syslog_writer.submit(log_entry, log_severity)
        
    def _deserialize_ros_log_msg(self, msg):
        """
        Add all the fields to syslog message according to the configuration
        """
        header = ''
        if self.configuration.include_seq:
            header = "%s %s" % (header, msg.header.seq) 
        if self.configuration.include_ros_node_name:
            header = "%s %s" % (header, msg.name)
        if self.configuration.include_file_name:
            header = "%s %s" % (header, msg.file)
        if self.configuration.include_function_name:
            header = "%s %s" % (header, msg.function)
        if self.configuration.include_line:
            header = "%s %s" % (header, msg.line)
        if self.configuration.include_topic:
            header = "%s (%s)" % (header, (',').join(msg.topics))
        
        log_entry = "%s %s" % (header, msg.msg)
        
        return log_entry
        
    def _get_syslog_severity(self, severity):
        ros_syslog_severity_map = {
                                    1: syslog.LOG_DEBUG,
                                    2: syslog.LOG_INFO,
                                    4: syslog.LOG_WARNING,
                                    8: syslog.LOG_ERR,
                                    16: syslog.LOG_CRIT
                                    }
        return ros_syslog_severity_map[severity]

def main():
    rospy.loginfo("Starting ROS rosout_logger")
    configuration = log_proxy_parameters.LogProxyParameters()
    syslog_writer = syslog_handler.SyslogHandler()
    
    try:
        log_proxy = LogProxy(configuration, syslog_writer)
        rospy.Subscriber(configuration.ros_log_source,
                        Log,
                        log_proxy._process_message)

    except rospy.ROSInterruptException, e:
        print "Exiting due to rospy.ROSInterruptException: %s" % e

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
