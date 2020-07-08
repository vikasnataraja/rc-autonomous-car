#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from ros_pololu_servo.msg import TBD_drive_cmds
from math import pi

#stop_the_car = False
#car_control_manual = False
stop_the_car = rospy.get_param("Kill_switch_val")
car_control_manual = rospy.get_param("Manual_mode_val")

class Controller:

        def __init__( self ):
                # Intializes everything
	                
                # subscribed to joystick inputs on topic "joy"
         	rospy.Subscriber("joy", Joy, self.callback)
         
         	# create a command publisher
                self.command_publisher = rospy.Publisher('TBD_high_level_command',TBD_drive_cmds,queue_size = 10)	

                # starts the node
                rospy.init_node('tbd_joystick_listener')
                r = rospy.Rate(100) # 100hz this was set on intutition not hard research
                #rospy.spin() # this is only needed for nodes that are listeners only and need to be kept waiting with callbacks. Hangs code otherwise.
                
                self.command_msg = TBD_drive_cmds()

        def scale_steering(self, in_steer):
                min_out = -pi/2
                max_out = pi/2
                out_range = max_out - min_out
                
                in_steer = (in_steer + 1)/2 # scales input to be 0-1
                return in_steer*out_range + min_out

        # axis 1 aka left stick vertical controls linear speed
        # axis 0 aka left stick horizonal controls angular speed
        def callback(self, data):
			global car_control_manual, stop_the_car

			throttle_axis = data.axes[1] # this is the left stick. 1 is full forward, -1 is full reverse
			steering_axis = data.axes[2] # this is the right stick. -1 is full right, 1 is full left

			kill_switch_btn = data.buttons[2] # this is the A button -- preferable to prevent occasional accidental kills
			# kill_switch_btn = data.buttons[7] # conservative, a real deadman style safety switch 

			auto_manual_btn = data.buttons[0] # this is the Y button

			if auto_manual_btn: 
				car_control_manual = not car_control_manual # toggle auto manual mode
				if car_control_manual: print("car in manual control mode")
				else: print("car in autonomous control mode")

			if kill_switch_btn: 
				stop_the_car = not stop_the_car
				if stop_the_car: print("Kill switch triggered, put the car in safe mode")
				else: print("Kill switch deactivated, let the car rip")
	        
			# manual control 
			STOPPED, FORWARD, REVERSE = 1,2,3
			if stop_the_car: # kill switch activated
				self.command_msg.throttle_mode = STOPPED
			else: 
				if car_control_manual:
					# currently just sending steering msgs always
					self.command_msg.steering_angle = -self.scale_steering(steering_axis)
					if throttle_axis >= 0:  
						self.command_msg.throttle_mode = FORWARD 
					else:                  
						self.command_msg.throttle_mode = REVERSE
					self.command_msg.throttle_cmd = abs(throttle_axis)
                                        
                        #print('Throttle:',throttle_axis,' Steering:',steering_axis)

        def run(self):
                print('hey')
                while (not rospy.is_shutdown()):
                        # testing
                        self.command_msg.throttle_mode = 2
                        self.command_msg.throttle_cmd = 2
                        self.command_msg.steering_angle = -0.2
                        # print('command throttle:',self.command_msg.throttle_cmd,' command steer:',self.command_msg.steering_angle)
                        self.command_publisher.publish(self.command_msg)
                        
                        

if __name__ == '__main__':
        print('heeee')
	TBD_controller = Controller()
        TBD_controller.run()

