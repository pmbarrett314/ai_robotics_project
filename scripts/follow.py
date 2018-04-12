#!/usr/bin/env python
import rospy
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
import math
import random

##########################
# BEGIN Global Variable Definitions
front_distance = -1.0
wall_distance = -1.0
# END Global Variable Definitions
##########################

##########################
# BEGIN ROS Topic Callback Function [DON'T MESS WITH THIS STUFF]
##########################
def laserCallback(laser_data):
#This function will set front_distance and wall_distance.  Both of these work in the same way.  If something is closer than the detection_distance for that sensor, the value will tell how close something is.  If nothing is closer than the detection distance, the value of -1 will be used.

    global front_distance
    global wall_distance

    front_detection_distance = 0.9
    wall_detection_distance = 0.9

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

############################
##### END CALLBACK FUNCTION
############################

if __name__ == '__main__':
    rospy.init_node('lab3', anonymous=True) #Initialize the ros node
    pub = rospy.Publisher('cmd_vel', Twist) #Create our publisher to send drive commands to the robot
    rospy.Subscriber("base_scan_0", LaserScan, laserCallback) #Subscribe to the laser scan topic

    rate = rospy.Rate(10) #10 Hz

    #SENSOR VARIABLES
    global front_distance #How close is something in front of the robot (-1 = nothing is close)
    global wall_distance  #How close is something on the right side of the robot (-1 = nothing is close)

    #########################################
    # LAB 3 VARIABLE DECLARATION CODE : BEGIN
    #########################################

    # PART C CODE HERE:
    #  Define and initialize variables that
    #  you need inside the main loop


    STATE_START="start"
    STATE_TURN_LEFT_1="left 1"
    STATE_FOLLOW_FORWARD="follow"
    STATE_TURN_RIGHT="right"
    STATE_TURN_LEFT_2="left 2"
    state=STATE_START
    #######################################
    # LAB 3 VARIABLE DECLARATION CODE : END
    #######################################

    while not rospy.is_shutdown():

        #Declare the twist command that we will publish this time step
        twist = Twist()
        #print(state)
        ###############################
        # LAB 3 INNER LOOP CODE : BEGIN
        ###############################
        object_in_front=0<=front_distance<=0.5
        wall_too_close=0<wall_distance<0.2
        wall_ok=0.2<=wall_distance<=0.8
        wall_too_far=0.8<wall_distance
        wall=wall_too_close or wall_ok or wall_too_far
        no_wall=wall_distance==-1
        # PART C CODE HERE:
        # Make sure that twist gets set with your drive command
        if state==STATE_START:
            if object_in_front:
                state=STATE_TURN_LEFT_1
            elif wall:
                state=STATE_FOLLOW_FORWARD
            else:
                twist.linear.x = abs(front_distance)
        elif state==STATE_TURN_LEFT_1:
            if object_in_front:
                twist.angular.z=math.pi/3
            elif wall:
                state=STATE_FOLLOW_FORWARD
            else:
                state=STATE_START
        elif state==STATE_FOLLOW_FORWARD:
            if object_in_front:
                state=STATE_TURN_LEFT_2
            elif wall:
                diff=(0.5-wall_distance)
                turn_factor=math.log(abs(diff)/0.5+1)
                speed_factor=math.exp(-abs(diff)/0.5)
                sign=0 if diff == 0 else diff/(abs(diff))
                if wall_too_close:
                    #twist.linear.x=0.2
                    twist.angular.z=math.pi/6
                elif wall_too_far:
                    twist.linear.x=0.5
                    twist.angular.z=-math.pi/8
                else:
                    #print("diff {} turn factor {}".format(diff,turn_factor))
                    twist.angular.z=sign*turn_factor*math.pi/4
                    twist.linear.x=speed_factor
            else:
                state=STATE_TURN_RIGHT
        elif state==STATE_TURN_RIGHT:
            if object_in_front:
                state=STATE_TURN_LEFT_2
            elif wall:
                state=STATE_FOLLOW_FORWARD
            else:
                twist.angular.z=-math.pi/3
                twist.linear.x=0.7
        elif state==STATE_TURN_LEFT_2:
            if object_in_front:
                twist.angular.z=math.pi/3
            elif wall:
                state=STATE_FOLLOW_FORWARD
            else:
                state=STATE_TURN_RIGHT
        #############################
        # LAB 3 INNER LOOP CODE : END
        #############################

        #Publish drive command
        pub.publish(twist)
        rate.sleep() #Sleep until the next time to publish

    twist = Twist()
    pub.publish(twist)

