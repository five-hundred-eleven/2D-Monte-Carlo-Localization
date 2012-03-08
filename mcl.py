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
NOISE = int(PAR_COUNT*.10)
cmd_vel = None

dt = 0.1
control = (0.0, 0.0)

# State
parset   = [mcl_tools.random_particle() for ii in range(PAR_COUNT)]

# range thetas, or angles
range_ts = [radians(t) for t in xrange(-90, 91, 45)]

centroid = (0, 0, 0) 
var_threshold = 0.70
threshold_cnt = 0
mode = 'wander'
gaussian_base = 1./(sqrt(2*pi)*0.5)

Wait, Found_Goal = False, False

# Gaussian probability
def gaussian_p(s_reading, p_reading):
    global gaussian_base
    mu = p_reading - s_reading
    exp = -(mu*mu)/0.5 
    return gaussian_base * e**exp 

# Particle weight assignment
def particle_weight(particle, scan):
    global range_ts
    return sum([gaussian_p(scan[i], mcl_tools.map_range(particle, range_ts[i]))
                for i in xrange(5)])

# Low variance sampling algorithm from the book.
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
    
    new_weights = np.array([particle_weight(p, scan) for p in ps])
    weights *= new_weights
    weights /= weights.sum()
    wvar = 1./sum([w*w for w in weights])
    if wvar < random.gauss(PAR_COUNT*.81, 60):
        ps = mcl_tools.random_sample(ps, PAR_COUNT - NOISE, weights) + [mcl_tools.random_particle() for ii in xrange(NOISE)]
        weights = [1.0 for ii in xrange(PAR_COUNT)]
    else:
        pass

    return [motion_update(p) for p in ps] 


def got_scan(msg):
    global Wait
    global Found_Goal
    if Wait or Found_Goal:
        return

    global parset
    global control
    global centroid

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
            # We're there! No movement signals success.
            return (0.0, 0.0)

        # are we facing the right direction?
        dt = set_heading(3*pi/2 - dx)
        v = 0.9 if abs(dt) < pi/8 else 0.0
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

def controller(msg):
    global Wait
    global Found_Goal
    if Wait or Found_Goal:
        control = (0.0, 0.0)
        cmd = Twist()
        (cmd.linear.x, cmd.angular.z) = control
        cmd_vel.publish(cmd)
        return

    global cmd_vel
    global mode
    global centroid
    global control
    global NOISE

    x, y, t = centroid 
    laserscan = msg.ranges

    # If mode is wander, just avoid hitting the walls. 
    if mode == 'wander' or abs(x-19.5) > 0.4:
        # Is there a wall in front of us? If so, slow down and spin.
        if min(laserscan[1:4]) < 1.9:
            t = ((laserscan[4]+laserscan[3]) - (laserscan[1]+laserscan[0]))
            if abs(t) < 1.00: 
                t = copysign(0.4, t) 
            else:
                t *= 0.20
            control = ((min(laserscan) - 1.0)*0.6, t) 
        # Otherwise, move forward and try to align with the nearest wall.
        else:
            l, d, t = (0, 1, 1) if laserscan[1] < laserscan[3] else (4, 3, -1)
            w = sqrt(l**2 + d**2 - 2*l*d*cos(pi/4)) 
            t = (l-w)*t if l > 0.5 else (w-l)*t
            control = (1.5, t*.5)

    # If mode is goal, try to get to the goal using pathfind()
    else: # mode == 'goal'
        v, t = pathfind((19.5, 1.5), centroid)
        if (v, t) == (0.0, 0.0):
            print ''
            print 'Great Success!'
            print ''
            Found_Goal = True
            return
        if min(laserscan[1:4]) < 0.75: 
            t += (laserscan[3] - laserscan[1])*0.35
        control = (v, t)

    cmd = Twist()
    (cmd.linear.x, cmd.angular.z) = control
    cmd_vel.publish(cmd)

    return



def dist(p0, p1):
    return sqrt((p0[0]-p1[0])**2 + (p0[1]-p1[1])**2)

def clustering():
    global parset
    global centroid
    global var_threshold
    global threshold_cnt
    global mode
    global control
    global Wait

    def motion_update(p):
        v, w = control 
        v_dt = v*dt
        w_dt = w*dt
        x, y, t = p
        return (x + v_dt*cos(t),
                y + v_dt*sin(t),
                angles.r2r(t))
    
    # Clustering algorithm: DBSCAN
    # Not important for the primary purpose of this assignment.
    while True:
        #ps = np.array([(p[0], p[1]) for p in parset])
        #whitened = whiten(ps)

        Wait = True

        ps = parset
        ps.sort()
        visited = set()
        clusters = {}
        cluster_cnt = 0
        for i in xrange(PAR_COUNT):
            if i in visited: 
                continue
            frontier = [i]
            cluster_cnt += 1
            clusters[cluster_cnt] = []
            while len(frontier):
                ci = frontier.pop()
                current = ps[ci]
                clusters[cluster_cnt].append(current)
                end_i = ci + 100
                if end_i > PAR_COUNT: 
                    end_i = PAR_COUNT
                region_query = [(dist(current, ps[j]), j) for j in xrange(ci, end_i) 
                                if j not in visited]
                region_query.sort(lambda a, b: cmp(a[0], b[0]))
                for d, qi in region_query: 
                    if d > 0.2:
                        break
                    else:
                        visited.add(qi)
                        frontier.append(qi)

        sz, best_cluster = max([(len(c), c) for c in clusters.values()])
        centroids, var = kmeans(np.array([(x, y) for x, y, t in best_cluster]), 1)
        x, y = centroids[0] 
        centroid = (x, y, best_cluster[random.randint(0, sz-1)][2])

        mode = 'wander' if sz < 800 else 'goal'
        print mode, centroid

        Wait = False

        rospy.sleep(0.1)
        for i in xrange(20):
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

    cmd_vel = rospy.Publisher('/robot/cmd_vel', Twist)
    rospy.Subscriber('/robot/base_scan', LaserScan, callback=got_scan,
            queue_size = 10, tcp_nodelay = True)
    rospy.Subscriber('/robot/base_scan', LaserScan, callback=controller,
            queue_size = 10, tcp_nodelay = True)

    clustering_thread = threading.Thread(target=clustering)
    clustering_thread.start()

    mcl_tools.mcl_init('sample_hw7')
    mcl_tools.mcl_run_viz()
