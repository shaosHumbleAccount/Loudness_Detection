#!/usr/bin/env python
import roslib; #roslib.load_manifest('add_two_ints_server')

import sys

import rospy
from audio_interface.srv import *

def add_two_ints_client(x, y):
    rospy.wait_for_service('add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', test)
        resp1 = add_two_ints(x, y)
        return resp1.Sum
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":
    x = 12
    y = 5
    print "Requesting %s+%s"%(x, y)
    print "%s + %s = %s"%(x, y, add_two_ints_client(x, y))