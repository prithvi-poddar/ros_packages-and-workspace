#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
from std_srvs.srv import Empty
import time

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



def carrot_chase(x1,y1,x2,y2):
    global x
    global y, yaw

    velocity_message = Twist()
    cmd_vel_topic='/turtle1/cmd_vel'
    if(x2-x1 == 0):
        m = (y2-y1)/(x2-x1+0.001)
    else:
        m = (y2-y1)/(x2-x1)
    c = y1-(m*x1)
    x_ort = (m*y+x-m*c)/((m*m)+1)
    y_ort = m*x_ort+c

    d = 0.09

    R = math.sqrt(((x_ort-x1)**2) + ((y_ort-y1)**2))
    theta = math.atan2(y2-y1, x2-x1)
    i = 1

    carrot_x = x_ort
    carrot_y = y_ort

    while (True):

        carrot_x = carrot_x + d*math.cos(theta)
        carrot_y = carrot_y + d*math.sin(theta)

        distance = abs(math.sqrt(((carrot_x-x) ** 2) + ((carrot_y-y) ** 2)))
        k_linear = 1

        linear_speed = 1


        K_angular = 1
        desired_angle_goal = math.atan2(carrot_y-y, carrot_x-x+0.01)
        error = desired_angle_goal-yaw
        angular_speed = error*K_angular

        if (angular_speed<-4.0):
             angular_speed = -4.0
        if (angular_speed>4.0):
             angular_speed = 4.0
        # if(linear_speed > 1.0):
        #     linear_speed = 1.0


        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed

        velocity_publisher.publish(velocity_message)
        print("error : "+str(error)+"   carrrot :"+str(carrot_y) )
        time.sleep(0.1)
        # print("  ")
        # print(carrot_y)
        # print("\n")
        
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

        carrot_chase(5.0,0.0, 5.0,5.0)
       
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")