#!/usr/bin/env python

#This node uses the /encoder_output topic to compute the odometry of a differential robot. It outputs the topic /odometry_pose presenting the pose of hte robot in the 2D space.

import rospy, time, math
from math import cos, sin
import numpy as np
from differential_robot.msg import counter_message , pose2D


 #wheels radius W_r
W_r = 0.035
#wheels spacing W_s
W_s = 0.165
#encoder counts per wheel revolution cpr
cpr = 1440.0

#initialisation of the variable 
previous_time=0 
old_R = 0.0
old_L = 0.0
pose_x = 0.0
pose_y = 0.0
pose_theta = 0.0


def get_wheel_speed(prev_reading, current_reading, delta_t): 	#this function outputs the wheels rotations in 
																#radians given the previous and current encoder readings

	return Wheelrot/delta_t #rotation speed


def get_speed_robot_frame(phi_L, phi_R): #this function takes the wheels speeds as input and outputs the speed components of the robot in its own frame.

	return np.array([Vx_base, Vy_base, omega_base])


def get_speed_map_frame(theta, matrix_speed_robot_frame): #This function computes the robot speeds in the map frame based on the current robot angle and its speeds in its own frame.

	return matrix_speed_robot_frame.dot(transformation_matrix)


def position_integration(matrix_speed_map_frame, delta_t): #This function uses the speeds in the map frame and the elapsed time since the last measurement to estimate the change in position, and adds this position change to the last 

	return  #this function updates the global pose variables and hence doesn't need to return anything


def callback(msg):
	global old_R , old_L, previous_time, pose_x,pose_y, pose_theta
	time_now=time.time()
	dt=previous_time -time_now
	new_L,new_R=msg.count_left,msg.count_right
	phi_L = get_wheel_speed(old_L, new_L, dt)
	phi_R = get_wheel_speed(old_R, new_R, dt)
	speed_robot_frame = get_speed_robot_frame(phi_L, phi_R)
	speed_map_frame = get_speed_map_frame(pose_theta, speed_robot_frame)
	position_integration(speed_map_frame, dt)
	#rospy.loginfo("I heard the encoder counts: left %s and right %s", msg.count_left, msg.count_right)					#you can comment out one of these lines to have visual feedback of the calculation steps - provided that you already implemented the required functions.
	#rospy.loginfo("HELLO,  %f seconds elapsed and the speeds are left %f and right %f", time_now-previous_time, phi_L*W_r, phi_R*W_r)
	#rospy.loginfo("positions are: pose_x= %f, pose_y= %f and pose_theta= %f", pose_x, pose_y, pose_theta)
	odom_publisher.publish(pose_x, pose_y, pose_theta)
	old_L= new_L
	old_R = new_R
	previous_time= time.time()
	return


if __name__=='__main__':
    rospy.init_node('odometry_node')
    rospy.loginfo("Initiated")
    previous_time = time.time() #initialising reference time
    odom_publisher = rospy.Publisher('odometry_pose', pose2D, queue_size = 1000)
    sub=rospy.Subscriber('encoders_output', counter_message, callback)
    rospy.spin()