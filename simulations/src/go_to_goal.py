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

    #print "pose callback"
    #print ('x = {}'.format(pose_message.x)) #new in python 3
    #print ('y = %f' %pose_message.y) #used in python 2
    #print ('yaw = {}'.format(pose_message.theta)) #new in python 3



def go_to_goal(x_goal, y_goal):
    global x
    global y, yaw

    velocity_message = Twist()
    cmd_vel_topic='/turtle1/cmd_vel'
    rate = rospy.Rate(100)

    while (True):
        distance = abs(math.sqrt(((x_goal-x) ** 2) + ((y_goal-y) ** 2)))
        k_linear =0.25

        linear_speed = k_linear * distance


        K_angular = 1.0
        desired_angle_goal = math.atan2(y_goal-y, x_goal-x)
        error = abs(desired_angle_goal-yaw)
        angular_speed = error*K_angular

        if (angular_speed<-10.0):
            angular_speed = -10.0
        if (angular_speed>10.0):
            angular_speed = 10.0


        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed

        velocity_publisher.publish(velocity_message)
        print(str(desired_angle_goal)+"  "+str(yaw))
        
        if (distance <0.1):
            break
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

        go_to_goal(1.0, 5.0)
       
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")