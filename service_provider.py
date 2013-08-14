#!/usr/bin/env python
import rospy; 
import roslib; roslib.load_manifest('audio_interface')
from audio_interface.srv import *


def handle_add_two_ints(req):
    print req
    print "Returning [%s + %s = %s]"%(req.A, req.B, (req.A + req.B))
    return testResponse(req.A + req.B)

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('add_two_ints', test, handle_add_two_ints)
    print "Ready to add two ints."
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()