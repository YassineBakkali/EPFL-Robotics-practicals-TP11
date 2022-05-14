#!/usr/bin/env python3

# program structure
# 

from re import I
import rospy
import math
import numpy as np
from ros_basics_msgs.msg import SimplePoseStamped
from ros_basics_msgs.msg import SimpleVelocities
from ros_basics_msgs.msg import ProximitySensors

from ros_basics_msgs.srv import CheckWaypointReached
from ros_basics_msgs.srv import CurrentWaypoint
from ros_basics_msgs.srv import RemoveWaypoint
from ros_basics_msgs.srv import GetWaypoints

#USE_SIMULATION = True
USE_SIMULATION = False

KP_LIN = .25
KD_LIN = 1.5
KI_LIN = .1

KP_ANG = 7
KD_ANG = 1
KI_ANG = .05

MAX_LIN_VEL = 11e-2
MAX_ANG_VEL = MAX_LIN_VEL/(4.75e-2)

WEIGHTS_ANG = [-10, -5, -5, 5, 10, 10, -10]
WEIGHTS_LIN = [-2, -5, -10, -5, -2, 10, 10]
PROX_CONV_COEFF = -0.0023
PROX_CONV_OFFSET = 12.281
OUTPUT_COEFF_ANG = 0.05
OUTPUT_COEFF_LIN = 0.005

NUMBER_WAYPOINTS = 1 # optional, if number of waypoints is unknown set to 1



cur_pos = SimplePoseStamped()
cur_pos.pose.xyz.x = 0.0
cur_pos.pose.xyz.y = 0.0

cur_sens = ProximitySensors()
wpts = GetWaypoints()

def poseCallback(_data):
    cur_pos.pose = _data.pose
    

def sensor_to_m(data):
    out = data*PROX_CONV_COEFF + PROX_CONV_OFFSET
    out[data==0] = math.inf
    return out*1e-2

def sensorsCallback(_data):
    cur_sens.values = _data.values

def spin():
    global file
    global I_ang, I_lin
    global err_ang_prev, err_lin_prev
    global start_recording
    # 1) call the corresponding service to check if the waypoint was reached

    wpt_reached = False
    rospy.wait_for_service('check_waypoint_reached')
    try:
        wpt_reached = chk_wpt_reached(cur_pos, True)
    except rospy.ServiceException as e:
        print("CheckWaypointReached service failed: %s"%e)


    # 2) subscribe to the robot_pose topic to get the robot's current pose

    # 3) call the corresponding service to get the current waypoint or waypoint list

    rospy.wait_for_service('current_waypoint')
    try:
        current_wpt = get_cur_wpt()

    except rospy.ServiceException as e:
        print("CurrentWaypoint service failed: %s"%e)

    # 4) implement your PID/PD/P logic

    # PID Controller

    ang_vel = 0
    lin_vel = 0

    err_lin = 0
    err_ang = 0

    err_lin = math.sqrt((current_wpt.goal.x - cur_pos.pose.xyz.x)**2 + (current_wpt.goal.y - cur_pos.pose.xyz.y)**2)
    err_ang = math.atan2(current_wpt.goal.y - cur_pos.pose.xyz.y, current_wpt.goal.x - cur_pos.pose.xyz.x) - cur_pos.pose.rpy.yaw
    if (err_ang > math.pi):
        err_ang -= 2*math.pi
    elif (err_ang < -math.pi):
        err_ang += 2*math.pi

    dt = 0.1

    P_ang = KP_ANG*err_ang
    I_ang += KI_ANG*err_ang*dt
    if (I_ang > 2*MAX_ANG_VEL):
        I_ang = 2*MAX_ANG_VEL
    elif (I_ang < -2*MAX_ANG_VEL):
        I_ang = -2*MAX_ANG_VEL
    D_ang = KD_ANG*(err_ang-err_ang_prev)/dt

    P_lin = KP_LIN*err_lin
    I_lin += KI_LIN*err_lin*dt
    if (I_lin > 2*MAX_LIN_VEL):
        I_lin = 2*MAX_LIN_VEL
    elif (I_lin < -2*MAX_LIN_VEL):
        I_lin = -2*MAX_LIN_VEL
    D_lin = KD_LIN*(err_lin-err_lin_prev)/dt

    ang_vel = P_ang + I_ang + D_ang

    if (abs(err_ang) > math.pi/2):
        lin_vel = 0
    else:
        lin_vel = P_lin + I_lin + D_lin


    dist = np.array(cur_sens.values)

    if USE_SIMULATION == False:
        dist = sensor_to_m(dist)

    for i in range(len(dist)):
        if (dist[i] == 0):
            dist[i] = 1e-12
        ang_vel += 1/dist[i]*WEIGHTS_ANG[i]*OUTPUT_COEFF_ANG
        lin_vel += 1/dist[i]*WEIGHTS_LIN[i]*OUTPUT_COEFF_LIN

    if (lin_vel > MAX_LIN_VEL):
        lin_vel = MAX_LIN_VEL
    elif (lin_vel < -MAX_LIN_VEL):
        lin_vel = -MAX_LIN_VEL 

    if (ang_vel > MAX_ANG_VEL):
        ang_vel = MAX_ANG_VEL
    elif (ang_vel < -MAX_ANG_VEL):
        ang_vel = -MAX_ANG_VEL 

    vel = SimpleVelocities(lin_vel, ang_vel)

    # 5) publish your computed velocities in the set_velocities topic

    # 6) if there are no waypoints left then set the velocities to 0 and wait for the next waypoint

    if current_wpt.is_empty == True:
        vel = SimpleVelocities(0, 0)

    if wpt_reached == True:
        # reset errors integrals when waypoint is reached
        I_ang = 0
        I_lin = 0

    vel_pub.publish(vel)

    # Start recording trajectory when we have enough waypoints
    wpts = list(get_wpts().waypoints)
    if (len(wpts) >= NUMBER_WAYPOINTS and start_recording == False):
        start_recording = True
        file = open("log.txt", "w")
        # save waypoints
        data = []
        for (i, wpt) in enumerate(wpts):
            data = "wpt%d;%.2f;%.2f\n" % (i, wpt.x, wpt.y)
            file.write(data)
            file.flush()
    
    if (start_recording == True and len(wpts) == 0):
        start_recording = False
        # file.close()

    if (start_recording == True):
        
        data = "%.2f;%.2f;%.2f;%.2f;%.2f\n" % (current_wpt.goal.x, current_wpt.goal.y, 
                                                cur_pos.pose.xyz.x, cur_pos.pose.xyz.y, cur_pos.pose.rpy.yaw)
        file.write(data)
        file.flush()

    err_ang_prev = err_ang
    err_lin_prev = err_lin

    pass


if __name__ == '__main__':

    rospy.init_node('thymio_control_pnode', anonymous=True)

    global chk_wpt_reached, get_cur_wpt, remove_wpt, get_wpts # services
    global vel_pub                      # publisher
    global I_ang, I_lin
    I_ang, I_lin = 0, 0
    global err_ang_prev, err_lin_prev
    err_ang_prev, err_lin_prev = 0, 0

    global start_recording
    start_recording = False

    chk_wpt_reached = rospy.ServiceProxy('check_waypoint_reached', CheckWaypointReached)
    get_cur_wpt = rospy.ServiceProxy('current_waypoint', CurrentWaypoint)
    remove_wpt = rospy.ServiceProxy('remove_waypoint', RemoveWaypoint)
    get_wpts = rospy.ServiceProxy('get_waypoints', GetWaypoints)

    rospy.Subscriber('robot_pose', SimplePoseStamped, poseCallback)
    rospy.Subscriber('proximity_sensors', ProximitySensors, sensorsCallback)
    vel_pub = rospy.Publisher('set_velocities', SimpleVelocities)


    loop_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        spin()
        loop_rate.sleep()

