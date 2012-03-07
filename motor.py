#!/usr/bin/env python
import roslib; roslib.load_manifest('uml_hmm')
import rospy

from random import gauss
from math import sqrt

from geometry_msgs.msg import Twist

cmd_vel = rospy.Publisher('/stage/cmd_vel', Twist)

def got_cmd_vel(msg):
    global cmd_vel

    sp = msg.linear.x
    sp = sp if sp < 4.0 else 4.0

    tu = msg.angular.z
    tu = tu if tu < 6.28 else 6.28

    msg.linear.x = gauss(sp, sqrt(tu*tu + sp*sp) / 6.0)
    msg.linear.y = 0
    msg.linear.z = 0

    msg.angular.x = 0
    msg.angular.y = 0
    msg.angular.z = gauss(tu, sqrt(tu*tu + sp*sp) / 6.0)

    cmd_vel.publish(msg)

rospy.init_node('motor')

rospy.Subscriber('/robot/cmd_vel', Twist, got_cmd_vel)

rospy.spin()
