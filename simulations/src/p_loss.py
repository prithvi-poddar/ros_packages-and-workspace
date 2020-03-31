#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
from std_srvs.srv import Empty

x=0
y=0
yaw=0

def poseCallback(pose_message):
    global x
    global y, yaw
    x= pose_message.x
    y= pose_message.y
    yaw = pose_message.theta

def carrot_chase(x1,y1,x2,y2):
    global x
    global y, yaw

    velocity_message = Twist()
    cmd_vel_topic='/turtle1/cmd_vel'

    while (True):
        A = 1.0
        m = (y2-y1)/(x2-x1+0.001)
        B = -m
        C = m*x2-y2
        distance = abs(A*x + B*y + C)/math.sqrt(math.pow(A,2)+math.pow(B,2))

        k_d = 0.1


        linear_speed = 1.0


        K_angular = 1.0
        desired_angle_goal = math.atan2(y2-y1, x2-x1)
        angular_speed = (desired_angle_goal-yaw)*K_angular + k_d*distance

        if (angular_speed<-10.0):
            angular_speed = -10.0
        if (angular_speed>10.0):
            angular_speed = 10.0

        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed

        velocity_publisher.publish(velocity_message)

if __name__ == '__main__':
    try:
        
        rospy.init_node('turtlesim_carrot_chase', anonymous=True)

        #declare velocity publisher
        cmd_vel_topic='/turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        
        position_topic = "/turtle1/pose"
        pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback) 
        time.sleep(2)

        carrot_chase(6.0,4.0, 6.0,7.0)
       
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")