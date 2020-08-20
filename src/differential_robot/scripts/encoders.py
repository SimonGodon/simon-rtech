#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import turtlesim.msg
import turtlesim.srv
from differential_robot.msg import counter_message

def corrector(x, y):
	if x < 1:
		if y > 0:
			return y
		if y <= 0:
			return y+360
	if x > 1:
		return 180-y
	


def counter(angle):
	interval= 360.0/1440.0
	count = angle/interval
	return math.modf(count)

def get_count(rot):
	quaternion = (rot[0],rot[1],rot[2],rot[3])
	euler = tf.transformations.euler_from_quaternion(quaternion)
	roll = euler[0]*180.0/math.pi
	pitch = euler[1]*180.0/math.pi
	yaw = euler[2]*180.0/math.pi
	interval= 360.0/1440.0
	count = corrector(roll,pitch)/interval
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



