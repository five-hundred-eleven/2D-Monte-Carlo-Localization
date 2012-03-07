#!/usr/bin/env python

import sys, os

import roslib; roslib.load_manifest('uml_mcl')
import rospy
rospy.init_node('mcl_localizer')

try:
    sys.path.append('/home/cowley/project-files/ros_workspace/sample_hw7/src')
    sys.path.append('/home/cowley/project-files/ros_workspace/sample_hw7/src/raycaster')
    import mcl_tools
except ImportError:
    print "error importing mcl tools"
    sys.exit(1)

import numpy as np
from scipy.cluster.vq import vq, kmeans, kmeans2, whiten
from scipy.spatial.distance import cdist, pdist
from scipy.sparse import coo_matrix
import random

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from time import *
import random
from math import *
import angles

import threading

PAR_COUNT = 1200
NOISE = PAR_COUNT*.10 
cmd_vel = None

dt = 0.1

# State
parset   = [mcl_tools.random_particle() for ii in range(PAR_COUNT)]

# range thetas, or angles
range_ts = [radians(t) for t in xrange(-90, 91, 45)]

centroid = (0, 0, 0) 
var_threshold = 0.70
threshold_cnt = 0
mode = 'wander'

laserscan = np.array([0.0 for i in xrange(5)])

global gaussian_base
gaussian_base = 1./(sqrt(2*pi)*0.5)

def gaussian_p(s_reading, p_reading):
    global gaussian_base
    mu = p_reading - s_reading
    exp = -(mu*mu) 
    return gaussian_base * e**exp 

def particle_weight(particle, scan):
    global range_ts
    return sum([gaussian_p(scan[i], mcl_tools.map_range(particle, range_ts[i]))
                for i in xrange(5)])

def low_var_sample(ps, sample_count, ws):
    p_prime = []
    sc_inverse = 1./len(ps)
    r = random.random()*sc_inverse
    c = ws[0]
    i = 0
    for m in xrange(sample_count):
        threshold = r + i*sc_inverse
        while threshold > c:
            i += 1
            c += ws[i]
        p_prime.append(ps[i])
    return p_prime

weights = np.array([1.0 for ii in xrange(PAR_COUNT)])
def particle_filter(ps, control, scan):
    global weights
    v, w = control
    v_dt = v*dt
    w_dt = w*dt
    sigma = sqrt(v*v + w*w)/6.0 * dt

    def motion_update(p):
        x, y, t = p
        new_p = (x + random.gauss(v_dt*cos(t), sigma),
                 y + random.gauss(v_dt*sin(t), sigma), 
                 t + random.gauss(w_dt, sigma))
        if not mcl_tools.map_hit(new_p[0], new_p[1]):
            return new_p
        else:
            return mcl_tools.random_particle()


    '''
    p_prime = [motion_update(p) if not mcl_tools.map_hit(p[0], p[1]) else
               mcl_tools.random_particle()
               for p in ps]
    '''
    
    weights = np.array([particle_weight(p, scan) for p in ps])
    weights /= weights.sum()
    wvar = 1./sum([w*w for w in weights])
    if wvar < random.gauss(PAR_COUNT*.80, 50):
        print 'resampled', wvar
        ps = mcl_tools.random_sample(ps, PAR_COUNT - NOISE, weights) + [mcl_tools.random_particle() for ii in xrange(NOISE)]
    else:
        print 'no resample', wvar

    return [motion_update(p) for p in ps] 


def got_scan(msg):
    global parset
    global control
    global centroid
    global laserscan

    laserscan = msg.ranges

    ''' 
    enum_ls = [(s, i) for i, s in enumerate(laserscan)]
    max_ls_index = max(enum_ls)[1]
    min_ls_val, min_ls_index = min(enum_ls)
    print max_ls_index
    control = (min_ls_val*0.9 - 1, (max_ls_index-(min_ls_index*0.55))*.85)
    '''

    # Calculate command to robot. 
    # if front sensor reading is greater than 3, take the difference between
    # left and right diagnal sensors.
    # Otherwise, slow down and go in whichever direction has more room.

    parset = particle_filter(parset, control, laserscan)
    mcl_tools.show_particles(parset)
    mcl_tools.show_best_pose(centroid)

    return



# find a path to goal tuple(x, y) from current(x, y, theta)
# specific to this scenario. Go to the middle of the hallway, go to correct x
# coordinate, go to correct y coordinate
def pathfind(goal, current):
    x, y, t = current
    gx, gy = goal
    dx = x - gx 
    dy = y - 6.0 # middle of the hallway 

    def set_heading(new_t):
        new_t = angles.r2r(new_t)
        clkwise = new_t - t
        ct_clkwise = 2*pi + t - new_t
        return clkwise if abs(clkwise) < abs(ct_clkwise) else ct_clkwise

    # are we at the right place on the x-axis?
    if abs(dx) < 0.4:
        print "correct longitude"
        # are we at the goal??
        dy = y - gy 
        if abs(dy) < 0.3:
            print ""
            print "Great success!"
            print ""
            sys.exit(1)
            control = (0.0, 0.0)

        # are we facing the right direction?
        dt = set_heading(3*pi/2 - dx)
        v = 1.5 if abs(dt) < pi/8 else 0.0
        control = (v, dt/3)

    # if not, are we at the right place on the y-axis?
    elif abs(dy) < 0.5:
        # are we facing the right difference?
        print "correct latitude"
        dt = set_heading(0.0 if dx > 0 else pi) 
        v = 1.5 if abs(dt) < pi/8 else 0.0 
        control = (v, dt/3)
    # if neither, get to the right latitude
    else:
        print "Neither latitude nor longitude are correct"
        # are we facing the right difference?
        dt = set_heading(3*pi/2 if dy > 0 else pi/2)
        v = 1.0 if abs(dt) < pi/8 else 0.0
        control = (v, dt/3)

    return control

control = (0.0, 0.0)
def controller(msg):
    global cmd_vel
    global mode
    global centroid
    global control
    global NOISE

    x, y, t = centroid 
    laserscan = msg.ranges

    if mode == 'wander' or abs(x-19.5) > 0.4:
        if min(laserscan[1:4]) < 1.9:
            t = ((laserscan[4]+laserscan[3]) - (laserscan[1]+laserscan[0]))
            if abs(t) < 1.00: 
                t = copysign(0.3, t) 
            else:
                t *= 0.15
            control = ((min(laserscan) - 1.0)*0.6, t) 
        else:
            l, d, t = (0, 1, 1) if laserscan[1] < laserscan[3] else (4, 3, -1)
            w = sqrt(l**2 + d**2 - 2*l*d*cos(pi/4)) 
            t = (l-w)*t if l > 0.5 else (w-l)*t
            control = (1.0, t*.5)

    else: # mode == 'goal'
        v, t = pathfind((19.5, 1.0), centroid)
        if min(laserscan[1:4]) < 0.75: 
            t += (laserscan[3] - laserscan[1])*0.35
        control = (v, t)

    cmd = Twist()
    (cmd.linear.x, cmd.angular.z) = control
    cmd_vel.publish(cmd)

    return




def clustering():
    global parset
    global centroid
    global var_threshold
    global threshold_cnt
    global mode
    global control

    def motion_update(p):
        v, w = control

        v_dt = v*dt
        w_dt = w*dt
        x, y, t = p
        return (x + v_dt*cos(t),
                y + v_dt*sin(t),
                angles.r2r(t+dt))

    while True:
        #ps = np.array([(p[0], p[1]) for p in parset])
        #whitened = whiten(ps)
        c, var = kmeans(np.array(parset), 1) 
        x, y, t = c[0]
        centroid = (x, y, angles.r2r(t))
        #print centroid, var, mode

        if var < var_threshold:
            mode = 'goal'
            threshold_cnt += 1
        else:
            mode = 'wander'
            threshold_cnt = 0

        rospy.sleep(0.08)
        for i in xrange(4):
            centroid = motion_update(centroid)
            rospy.sleep(0.1)



if __name__ == '__main__':
    # Uncomment for debugging if nesssary, recomment before turning in.
    #rospy.Subscriber('/stage/base_pose_ground_truth', Odometry, mcl_debug.got_odom)
    try:
        import psyco
        psyco.full()
    except IOError:
        pass

    rospy.Subscriber('/robot/base_scan', LaserScan, callback=got_scan,
            queue_size = 10, tcp_nodelay = True)
    rospy.Subscriber('/robot/base_scan', LaserScan, callback=controller,
            queue_size = 10, tcp_nodelay = True)
    cmd_vel = rospy.Publisher('/robot/cmd_vel', Twist)

    clustering_thread = threading.Thread(target=clustering)
    clustering_thread.start()

    mcl_tools.mcl_init('sample_hw7')
    mcl_tools.mcl_run_viz()
