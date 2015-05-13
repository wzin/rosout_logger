#!/usr/bin/env python

import syslog
import rospy

class SyslogHandler():
    def __init__(self, configuration):
        rospy.loginfo("Initializing %s" % self.__class__.__name__)
        self.facility = configuration.syslog_facility
        self.respect_severity = configuration.respect_severity
        # notimplemented yet start
        self.port = configuration.log_port
        self.host = configuration.log_host
        # notimplemented yet end
        
        syslog.openlog(facility=self.facility)
    
    def submit(self, message, severity):
        if self.respect_severity:
            self.syslog(severity, message)
        else:
            self.syslog(syslog.LOG_INFO, message)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
