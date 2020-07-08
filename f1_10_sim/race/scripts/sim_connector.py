#!/usr/bin/env python

import rospy
# from race.msg import drive_param
from ackermann_msgs.msg import AckermannDriveStamped
from ros_pololu_servo.msg import TBD_drive_cmds

import math

pub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=5)

def vel_and_angle(data):
	

	msg = AckermannDriveStamped();
	msg.header.stamp = rospy.Time.now();
	msg.header.frame_id = "base_link";

	# msg.drive.speed = data.velocity
	# note that the below just immediately changes velocity and doesn't account for acceleration (which could be mapped)
	STOPPED, FORWARD, REVERSE = 1,2,3
	if data.throttle_mode == STOPPED: 
		msg.drive.speed = 0
	elif data.throttle_mode == FORWARD: 
		msg.drive.speed = data.throttle_cmd
	elif data.throttle_mode == REVERSE: 
		msg.drive.speed = -1*data.throttle_cmd
		
	msg.drive.acceleration = 1
	msg.drive.jerk = 1
	msg.drive.steering_angle = data.steering_angle
	msg.drive.steering_angle_velocity = 1
	# print "velocity", data.velocity
	# print "angle", data.angle

	pub.publish(msg)



def listener():
	rospy.init_node('sim_connect', anonymous=True)
	rospy.Subscriber('/TBD_high_level_command', TBD_drive_cmds, vel_and_angle)
	# rospy.Subscriber('drive_parameters', drive_param, vel_and_angle)
	rospy.spin()





if __name__=="__main__":
	listener()
