#!/usr/bin/env python
 

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
previous_time=0 #initialisation of the variable
old_R = 0.0
old_L = 0.0

pose_x = 0.0
pose_y = 0.0
pose_theta = 0.0
pose=[]

def get_wheel_speed(prev_reading, current_reading, delta_t): 	#this function outputs the wheels rotations in 
																				#radians given the previous and current encoder readings
	if current_reading - prev_reading >=1000: #this loop deals with the cases when the absollute encoder reaches the max or min value and starts over
		diff_reading=current_reading - prev_reading - cpr
	elif current_reading - prev_reading <=-1000:
		diff_reading=current_reading - prev_reading + cpr
	else:
		diff_reading = current_reading - prev_reading
	Wheelrot= 2*math.pi*diff_reading/cpr #wheel rotation in radians since last measurement

	return Wheelrot/delta_t #rotation speed

def get_speed_robot_frame(phi_L, phi_R):
	Vx_base = (phi_L*W_r + phi_R * W_r)/2
	Vy_base = 0
	omega_base = (phi_L*W_r - phi_R * W_r)/W_s

	return np.array([Vx_base, Vy_base, omega_base])

def get_speed_map_frame(theta, matrix_speed_robot_frame):
	transformation_matrix = np.array([[cos(pose_theta),-sin(pose_theta),0],[sin(pose_theta),cos(pose_theta),0],[0,0,1]]) #recompute transformation matrix with new theta
	return matrix_speed_robot_frame.dot(transformation_matrix)

def position_integration(matrix_speed_map_frame, delta_t):
	global pose_x,pose_y, pose_theta
	pose_x +=  matrix_speed_map_frame[0]*delta_t
	pose_y +=  matrix_speed_map_frame[1]*delta_t
	pose_theta += matrix_speed_map_frame[2]*delta_t
	return  #this function updates the global pose variables and hence doesn't need to return anything


def callback(msg):
	global old_R , old_L, previous_time, pose_x,pose_y, pose_theta
	#rospy.loginfo("I heard the encoder counts: left %s and right %s", msg.count_left, msg.count_right)
	time_now=time.time()
	dt=previous_time -time_now
	new_L,new_R=msg.count_left,msg.count_right
	phi_L = get_wheel_speed(old_L, new_L, dt)
	phi_R = get_wheel_speed(old_R, new_R, dt)

	speed_robot_frame = get_speed_robot_frame(phi_L, phi_R)
	speed_map_frame = get_speed_map_frame(pose_theta, speed_robot_frame)
	position_integration(speed_map_frame, dt)

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