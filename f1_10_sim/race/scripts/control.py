#!/usr/bin/env python

import rospy

from race.msg import drive_param
from race.msg import pid_input
import math
import numpy as np
from ros_pololu_servo.msg import TBD_drive_cmds


#pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)
pub = rospy.Publisher('TBD_high_level_command',TBD_drive_cmds,queue_size = 10)	

kp = 10.
kd = 0.01
kp_vel = 42.0
kd_vel = 0.0

ki = 0.0
servo_offset = 18.0*math.pi/180
prev_error = 0.0 
error = 0.0
integral = 0.0
vel_input = 1.0


def control(data):
	global integral
	global prev_error
	global vel_input
	global kp
	global ki
	global kd
	global kd_vel
	global kp_vel
	velocity = data.pid_vel
	angle = servo_offset
	error = 5*data.pid_error
	print "Error Control",error
	if error!=0.0:
		# if abs(error - prev_error)>0.5: 	
		# 	integral = integral + error	
		control_error = kp*error + kd*(error - prev_error)# + ki*integral
		print "Control", control_error
		# integral = integral/1.3
		
		# print "Control error",control_error
		angle = angle + control_error*np.pi/180
		# print "Control error",control_error

		control_error_vel = kp_vel*error + kd_vel*(error - prev_error)
		# print "Control error velocity",control_error_vel

		# velocity = velocity - abs(control_error_vel)/10
		velocity = velocity + abs(control_error_vel)

		

	prev_error = error

	if angle > 30*np.pi/180:
		angle = 30*np.pi/180
	if angle < -30*np.pi/180:
		angle = -30*np.pi/180



	# print "Velocity",velocity
	print "Angle in Degrees",angle*180/np.pi # Just for reference
	#msg = drive_param()
	msg = TBD_drive_cmds()

	# velocity = 1
	# # if angle > 20*np.pi/180 or angle < -20*np.pi/180:
	# # 	velocity = 0.3

	if angle >= 10*np.pi/180 or angle <= -10*np.pi/180:
		velocity = 2

	if angle > 20*np.pi/180 or angle < -20*np.pi/180:
		velocity = 2

	if angle >= -1*np.pi/180 and angle <= 1*np.pi/180:
		velocity = 5.0

	if velocity < 0:
		velocity = 1

	if velocity > 5:
		velocity = 5



	print "Velocity",velocity
	print "Angle",angle


	#msg.velocity = velocity
	#msg.angle = angle
	#pub.publish(msg)
	msg.throttle_mode = 2
	msg.throttle_cmd = velocity
	msg.steering_angle = angle
	pub.publish(msg)

def listener():
	rospy.init_node('pid_controller', anonymous=True)
	rospy.Subscriber("error", pid_input, control)
	global kp
	global ki
	global kd
	global vel_input
	kp = input("Enter Kp Value: ")
	ki = input("Enter Ki Value: ")
	kd = input("Enter Kd Value: ")
	vel_input = input("Enter Velocity: ")
	rospy.spin()


if __name__ == '__main__':
	print("Listening to error for PID")
	listener()
