#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point, Twist
from ros_pololu_servo.msg import MotorCommand
from ros_pololu_servo.msg import HARECommand

steering_cmd_msg = MotorCommand()
motor_cmd_msg = MotorCommand()
def cmd_cb(msg):
    # Limit incoming message to steering limits
    if msg.steering_angle > .78:
        msg.steering_angle = .78
    elif msg.steering_angle < -.78:
        msg.steering_angle = -.78

    steering_cmd_msg.position = msg.steering_angle

    # Limit the incoming throttle command
    if msg.throttle_cmd > 1:
        msg.throttle_cmd = 1
    elif msg.throttle_cmd < 0:
        msg.throttle_cmd = 0.0

    # Handle throttle mode
    if msg.throttle_mode == 0:
        motor_cmd_msg.position = 0.0
    elif msg.throttle_mode == 3:
        motor_cmd_msg.position = -1*msg.throttle_cmd
    else:
        motor_cmd_msg.position = msg.throttle_cmd


def msg_publisher():
    cmd_sub = rospy.Subscriber('/HARE_high_level_command', HARECommand, cmd_cb )
    msg_pub = rospy.Publisher('/pololu/command', MotorCommand, queue_size=10)
    rospy.init_node('cmd_publisher', anonymous=True)
    pub_rate = rospy.Rate(20)
    #cmd_msg = MotorCommand()

    motor_cmd_msg.joint_name = "drive_motor"
    motor_cmd_msg.position = 0
    motor_cmd_msg.speed = 1.0
    motor_cmd_msg.acceleration = 1.0

    steering_cmd_msg.joint_name = "steering_servo"
    steering_cmd_msg.position = 0.0
    steering_cmd_msg.speed = 1.0
    steering_cmd_msg.acceleration = 1.0


    while not rospy.is_shutdown():
        msg_pub.publish(motor_cmd_msg)
        msg_pub.publish(steering_cmd_msg)
        pub_rate.sleep()

if __name__ == '__main__':
    
    try:
        msg_publisher()
    except rospy.ROSInterruptException:
        print('Shutting down publisher...')
        pass
