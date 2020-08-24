#!/usr/bin/env python  

#This node uses /tf to publish the count of a fake absolute encoder for the two wheels of a differential robot. The topic published is encoders_output.

import roslib
import rospy
import math
import tf
from differential_robot.msg import counter_message

cpr = 1440.0

def corrector(x, y): #this function deals with the instability of the transformation from quaternion to euler angles
	if x < 1:
		if y > 0:
			return y
		if y <= 0:
			return y+360
	if x > 1:
		return 180-y


def get_count(rot): #this function takes a quaternion as an input and outputs the absolute encoder count of the rotation between the two franes.
	quaternion = (rot[0],rot[1],rot[2],rot[3])
	euler = tf.transformations.euler_from_quaternion(quaternion)
	roll = euler[0]*180.0/math.pi #converting rad to degree 
	pitch = euler[1]*180.0/math.pi
	interval= 360.0/cpr
	count = corrector(roll,pitch)/interval  #correcting the instability of the conversion quaternion to euler
	return int(math.modf(count)[1])

if __name__ == '__main__':
    rospy.init_node('encoders_node')
    listener = tf.TransformListener()
    count_publisher = rospy.Publisher('encoders_output', counter_message, queue_size = 1000)
    rate = rospy.Rate(25.0)
    while not rospy.is_shutdown():
        try:
            rot_left = listener.lookupTransform('base_link', 'left_wheel', rospy.Time(0))[1]
            rot_right = listener.lookupTransform('base_link', 'right_wheel', rospy.Time(0))[1]
        except (tf.LookupException, tf.ConnectivityException):
            continue
        count_publisher.publish(get_count(rot_left), get_count(rot_right))
        rate.sleep()
