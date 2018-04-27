#!/usr/bin/env python
import rospy
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
from kobuki_msgs.msg import *
import math
import random

front_distance = -1.0
wall_distance = -1.0

def laserCallback(laser_data):
#This function will set front_distance and wall_distance.  Both of these work in the same way. 
#If something is closer than the detection_distance for that sensor, the value will tell how close something is.  If nothing is closer than the detection distance, the value of -1 will be used.

    global front_distance
    global wall_distance

    front_detection_distance = 100000.0
    wall_detection_distance = 100000.0

    wf_min_right = 10.0
    wf_min_front = 10.0

    cur_angle = laser_data.angle_min

    for i in range(len(laser_data.ranges)):
        if abs(cur_angle + math.pi/2) < (math.pi / 8):
            #Wall sensor ((-5/8)*math.pi <= cur_angle <= (-3/8)*math.pi)
            if laser_data.ranges[i] < wf_min_right:
                wf_min_right = laser_data.ranges[i]

        if abs(cur_angle) < (math.pi / 8):
            #Front sensor ((-1/8)*math.pi <= cur_angle <= (1/8)*math.pi)
            if laser_data.ranges[i] < wf_min_front:
                wf_min_front = laser_data.ranges[i]

        cur_angle = cur_angle + laser_data.angle_increment

    #Set the sensor variables
    front_distance = -1
    wall_distance = -1
    if wf_min_front < front_detection_distance:
        front_distance = wf_min_front
    if wf_min_right < wall_detection_distance:
        wall_distance = wf_min_right

def bumpCallback(bump):
    twist = Twist()
    twist.angular.z=-math.pi/2
    pub.publish(twist)

if __name__ == '__main__':
    rospy.init_node('lab3', anonymous=True) #Initialize the ros node
    pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist) #Create our publisher to send drive commands to the robot
    rospy.Subscriber("/scan", LaserScan, laserCallback) #Subscribe to the laser scan topic
    rospy.Subscriber("/mobile_base/events/bumper",BumperEvent,bumpCallback)

    rate = rospy.Rate(10) #10 Hz

    #SENSOR VARIABLES
    global front_distance #How close is something in front of the robot (-1 = nothing is close)
    global wall_distance  #How close is something on the right side of the robot (-1 = nothing is close)

    while not rospy.is_shutdown():
        print(front_distance)
        #Declare the twist command that we will publish this time step
        twist = Twist()
        
        object_in_front=0<=front_distance<=0.65

	if object_in_front:
            twist.angular.z=-math.pi/2
        else:
            twist.linear.x=0.2

        #Publish drive command
        pub.publish(twist)
        rate.sleep() #Sleep until the next time to publish

    twist = Twist()
    pub.publish(twist)

