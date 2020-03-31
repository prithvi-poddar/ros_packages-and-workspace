#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
from std_srvs.srv import Empty
import time
import numpy as np

x=0
y=0
yaw=0

def poseCallback(pose_message):
    global x
    global y, yaw
    x= pose_message.x
    y= pose_message.y
    yaw = pose_message.theta

    #print "pose callback"
    #print ('x = {}'.format(pose_message.x)) #new in python 3
    #print ('y = %f' %pose_message.y) #used in python 2
    #print ('yaw = {}'.format(pose_message.theta)) #new in python 3


def Angle_wrap(theta):
    if theta > np.pi:
        return theta - 2*np.pi
    elif theta < -np.pi:
        return theta + 2*np.pi
    else:
        return theta

def Turn_check(u, v, R):
    if u > v**2/R:
        return v**2/R
    elif u < -v**2/R:
        return -v**2/R
    else:
        return u

def Euclidean_distance(x,y):
    return np.sqrt((y[1]-x[1])**2 + (y[0]-x[0])**2)


def carrot_chase( P1, P2, d=2, K=0.1, v=2.0,  R_min=3):
    global x
    global y, yaw

    velocity_message = Twist()
    cmd_vel_topic='/turtle1/cmd_vel'

    path = []
    theta = np.arctan2((P2[1] - P1[1]),(P2[0] - P1[0]))
    
    
    while True:
        newPos = [x,y]
        psi = yaw
        theta_u = np.arctan2((newPos[1] - P1[1]),(newPos[0] - P1[0]))
        R_u = Euclidean_distance(newPos, P1)
        beta = Angle_wrap(theta - theta_u)

        R = R_u * np.cos(beta)
        x_t = P1[0] + (R+d) * np.cos(theta)
        y_t = P1[1] + (R+d) * np.sin(theta)
        psi_d = np.arctan2((y_t - newPos[1]),(x_t - newPos[0]))

        u = K * Angle_wrap((psi_d - psi))
        u = Turn_check(u, v, R_min)
       


        velocity_message.linear.x = v
        velocity_message.angular.z = u

    rate.sleep()


    




if __name__ == '__main__':
    try:
        
        rospy.init_node('turtlesim_motion_pose', anonymous=True)

        #declare velocity publisher
        cmd_vel_topic='/turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        
        position_topic = "/turtle1/pose"
        pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback) 
        time.sleep(2)

        carrot_chase([0.0,0.0], [10.0,10.0])
       
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")