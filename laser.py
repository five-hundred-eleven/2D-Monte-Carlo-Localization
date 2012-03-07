#!/usr/bin/env python
import roslib; roslib.load_manifest('uml_hmm')
import rospy
import random

from sensor_msgs.msg import LaserScan

pub = None

def got_scan(msg):
    global pub
    
    tmp = list(msg.ranges)

    for ii in range(len(tmp)):
        if tmp[ii] < 5.0:
            tmp[ii] = random.gauss(tmp[ii], 0.5)

    msg.ranges = tmp
    pub.publish(msg)


def crappy_laser():
    global reading
    global pub

    pub = rospy.Publisher('/robot/base_scan', LaserScan)
    rospy.init_node('laser')
    
    rospy.Subscriber('/stage/base_scan', LaserScan, got_scan)

    rospy.spin()

if __name__ == '__main__':
    try:
        crappy_laser()
    except rospy.ROSInterruptException: pass
