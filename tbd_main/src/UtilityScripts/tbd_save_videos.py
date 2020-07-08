#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from sensor_msgs.msg import LaserScan
from ros_pololu_servo.msg import TBD_drive_cmds
from math import pi
import numpy as np
import datetime as dt
import sys
import cv2
import pandas as pd
import os

# ------------- get launch file parameters ---------------
'''
kill_switch = rospy.get_param("Kill_switch_val") # global for E-stop
car_control_manual = rospy.get_param("Manual_mode_val") # global for manual control
Simulation = rospy.get_param("SIM") # running simulation or on robot 
'''
kill_switch = False
car_control_manual = False
Simulation = False

# ------------- setup global vars ---------------
Sim_scale = 10 # since the simulation runs in m/s and the servo operates on -1 to 1 

lidar_max_dist = 35 # max threshold of 20 meters
# ------ END of setup global vars ---------------

# -- vid vars --
recording = False
vid_count = 0
fname = '/TBD_vis_odom_Video0/f0000.jpg' # this is what the file name should look like
# -- end of vid vars --


# Setup that is simulation vs robot dependent
if Simulation: 
	from sensor_msgs.msg import LaserScan # as "commonname" ?
else: # running on robot
	# load rp lidar topic
	pass


class Controller:
	def __init__( self ):
		# Intialize everything
		
		# ------------- initialize variables ---------------
		self.command_msg = TBD_drive_cmds()
		self.command_msg.throttle_cmd = 0
		self.command_msg.steering_angle = 0
		self.debug_sent_time = dt.datetime.now()
		self.Debug_msg = ''
		self.scan_data_rdy = False
                self.new_scan = False
		self.one_shot = True
		
		self.num_largest = 75 # this will need to be tuned
		self.Kp = 0.003
		
                self.Kp_turn = 0.002
		self.front_depth_window = 50 # number of indices around center (forward) to evaluate for wall being close and starting a turn
                self.turn_thresh = 6 # meters till know at a turn
		
		self.max_speed = 0.1 
		self.min_speed = 0.2
		self.speed_scale = 0.16
		# ------ END of initialize variables ---------------
		
		# ------------- sensor input topic subscriptions ---------------
		rospy.Subscriber("joy", Joy, self.joystick_callback) # subscribe to joystick inputs on topic "joy"
		rospy.Subscriber("scan",LaserScan, self.lidar_callback) # subscribe to lidar scan data
		# ------ END of sensor input topic subscriptions ---------------
		
		# create a command publisher
		self.command_publisher = rospy.Publisher('TBD_high_level_command',TBD_drive_cmds,queue_size = 10)	

		self.r = rospy.Rate(100) # 100hz this was set on intutition not hard research

	def scale_joysteering(self, in_steer): # take in steering and scale it to pi/2, -pi/2 radians
		min_out = -pi/2
		max_out = pi/2
		out_range = max_out - min_out
		in_steer = (in_steer + 1)/2 # scales joystick input to be 0-1
		return in_steer*out_range + min_out


	def joystick_callback(self, data): # Estop and manual control
		global car_control_manual, kill_switch, recording, vid_count

		self.throttle_axis = data.axes[1] # this is the left stick. 1 is full forward, -1 is full reverse
		self.steering_axis = data.axes[2] # this is the right stick. -1 is full right, 1 is full left

		kill_switch_btn = data.buttons[2] # this is the B button -- preferable to prevent occasional accidental kills
		auto_manual_btn = data.buttons[0] # this is the X button
                # 1 is A, 3 is Y
                
                if data.buttons[3] and not recording: 
			print('started recording')
			recording = True
                if data.buttons[1] and recording: 
			print('stopped recording')
			recording = False
			vid_count += 1
		if auto_manual_btn: 
			car_control_manual = not car_control_manual # toggle auto manual mode
			if car_control_manual: self.Debug_msg += "Car is now in MANUAL control mode"
			else: self.Debug_msg += "Car is now in Autonomous control mode"

		if kill_switch_btn: 
			kill_switch = not kill_switch
			if kill_switch: self.Debug_msg += "Kill switch triggered, car is now in SAFE mode"
			else: self.Debug_msg += "Kill switch deactivated, car is now in DRIVE mode"
		
	def lidar_callback(self, data):
                if not self.one_shot:
                        self.scan_data_rdy = True
                        self.last_scan_data = self.scan_data
                 
                        
                #self.scan_data = np.asarray(data.ranges)
                self.scan_data = np.concatenate((data.ranges[-int(np.ceil(len(data.ranges)/2)):],data.ranges[0:int(np.floor(len(data.ranges)/2))]),0)
                self.scan_data[np.isinf(self.scan_data)] = 0
                self.scan_data[np.isnan(self.scan_data)] = 0
                self.scan_data[self.scan_data>15] = lidar_max_dist
                self.scan_data[self.scan_data<0] = 0
                self.one_shot = False
                self.new_scan = True
                #print('scan info:',len(data.ranges),max(self.scan_data),min(self.scan_data))

	def Drive_at_deepest(self):
		#print(self.scan_data[0],self.scan_data[-1],len(self.scan_data),max(self.scan_data),min(self.scan_data))
		
		self.new_scan = False
		self.front_scan = self.scan_data[int(len(self.scan_data)/3):2*int(len(self.scan_data)/3)] # should grab the front 180 degrees
		self.straights_cent_setpoint = int( len(self.front_scan)/2 )
                #print("sorting")
                #print(self.front_scan)
                self.sorted_scan_inds = np.argsort(self.front_scan) # sorted from smallest to largest
                #print("sorted")
                self.N_largest_inds = self.sorted_scan_inds[-self.num_largest:]
                #print(self.N_largest_inds)
                
		#largest = self.front_scan[self.N_largest_inds]
                largest = [self.front_scan[i] for i in self.N_largest_inds]
                #print(largest)
                # print('Sim?:',Simulation)
		#if Simulation: # doing some visualizatoion
                #plt.clf()
                #plt.plot(self.front_scan,'bo')
                #plt.ylim( [ 0 , 20 ] )
                #plt.hold( True )
                #plt.plot(self.N_largest_inds, largest,'ro')
                                
                #plt.pause( 0.001 )

		#print('stand dev:',np.std(self.N_largest_inds))
		input_center = np.mean( self.N_largest_inds )
		translation_err = input_center - self.straights_cent_setpoint
		
                if np.mean(self.front_scan[int(len(self.front_scan)/2)-int(self.front_depth_window/2):int(len(self.front_scan)/2)+int(self.front_depth_window/2)]) > self.turn_thresh: 
                        cntrl_steer = self.Kp*translation_err
		else: cntrl_steer = self.Kp_turn*translation_err
		
		self.command_msg.steering_angle = -1*cntrl_steer
		
                if np.degrees(abs(cntrl_steer)) > 6: speed_cmd = self.min_speed # slow down when large steering corrections are needed
		else: speed_cmd = np.mean(largest)*self.speed_scale - 0.6
		
		if speed_cmd > self.max_speed: speed_cmd = self.max_speed
		if speed_cmd < self.min_speed: speed_cmd = self.min_speed
		self.command_msg.throttle_cmd = speed_cmd
		
		#print('err',translation_err,'steer:',np.degrees(cntrl_steer),'mean largest',np.mean(largest))
		self.Debug_msg += 'err'+str(translation_err)+'steer:'+str(np.degrees(cntrl_steer))+'mean largest'+str(np.mean(largest))
                
	'''
	def UPenn_PID_wall(self):
		try:
			if self.old_scan_data != self.scan_data:
				up_vel, up_steer = PID_wall_follow(self.scan_data)
				
				self.command_msg.throttle_cmd = up_vel
				self.command_msg.steering_angle = up_steer
				
				self.old_scan_data = self.scan_data
		except AttributeError: 
			self.old_scan_data = self.scan_data
			pass # first pass 
        '''

		
	def outputs(self):
		STOPPED, FORWARD, REVERSE = 1,2,3
		if kill_switch: # kill switch activated
			self.command_msg.throttle_mode = STOPPED
                        self.Debug_msg = 'ESTOP on (B btn):' + self.Debug_msg
		else: 
			if car_control_manual:
				self.Debug_msg = 'Manual ctrl (X btn):' + self.Debug_msg
				self.command_msg.steering_angle = -self.scale_joysteering(self.steering_axis)
				if self.throttle_axis >= 0: self.command_msg.throttle_mode = FORWARD 
				else: self.command_msg.throttle_mode = REVERSE
				self.command_msg.throttle_cmd = abs(self.throttle_axis)
			else: # autonomous mode
				self.Debug_msg = 'Auton ctrl (X btn):' + self.Debug_msg
				# testing
				#self.command_msg.throttle_cmd = 1.2
				#self.command_msg.steering_angle = 0
				
				if self.command_msg.throttle_cmd >= 0: self.command_msg.throttle_mode = FORWARD 
				else: self.command_msg.throttle_mode = REVERSE

		
		if Simulation: # scale variables as needed for simulation
			self.command_msg.throttle_cmd = Sim_scale * self.command_msg.throttle_cmd
			#self.command_msg.steering_angle = -1*self.command_msg.steering_angle # steering is opposite sim to real?
                
                #self.command_msg.throttle_mode = STOPPED # temp!!!!!!!
		self.command_publisher.publish(self.command_msg)

	def run(self):
                recording_stopped_flag = False
		cap = cv2.VideoCapture(0)
		# Define the codec and create VideoWriter object.The output is stored in 'outpy.avi' file.
		fourcc = cv2.VideoWriter_fourcc(*'DIVX')
		out = cv2.VideoWriter('/media/tbd/4E28-7C59/video'+str(vid_count)+'.avi', fourcc, 20.0, (640,480))

		while (not rospy.is_shutdown()):
                        #if self.scan_data_rdy:
                                #print(np.mean(self.scan_data))
			now = dt.datetime.now() # updates with current time. This shouldn't be set/reset anywhere else.
			self.Debug_msg = '' 
			# inputs are handled automatically through call backs
			
			# call control strategy
			if self.scan_data_rdy and self.new_scan:
                                self.Drive_at_deepest()
                                #pass
			#	self.UPenn_PID_wall()
			
			# outputs
			self.outputs()
			
			debug_publish_period = 1 # seconds
			if (now - self.debug_sent_time).seconds >= debug_publish_period:
					self.debug_sent_time = now
					print(self.Debug_msg)

                        if not recording and recording_stopped_flag:
				deltas = pd.DataFrame(deltas_array)
				if vid_count > 0: deltas.to_csv('/media/tbd/4E28-7C59/deltas_video_'+str(vid_count-1))
				else: deltas.to_csv('/media/tbd/4E28-7C59/deltas_video_'+str(vid_count))

				out.release()
				out = cv2.VideoWriter('/media/tbd/4E28-7C59/video'+str(vid_count)+'.avi', fourcc, 20.0, (640,480))
                        if recording: 
			            if not os.path.exists('/media/tbd/4E28-7C59/TBD_vis_odom_Video'+str(vid_count)):
				    		os.makedirs('/media/tbd/4E28-7C59/TBD_vis_odom_Video'+str(vid_count))
				    #if recording_started_flag:
				    #		out = cv2.VideoWriter('/media/tbd/4E28-7C59/video'+str(vid_count)+'.avi', fourcc, 20.0, (640,480))


				    last = dt.datetime.now()
				    # Capture frame-by-frame
				    ret, frame = cap.read()

				    frame = cv2.flip(frame, 0 ) # the web cam is mounted upside down, so flip image

				    # Display the resulting frame
				    #cv2.imshow('frame',frame)

				    # update the frame name
				    if frame_count<10: imgstr = '000'+str(frame_count)
				    elif frame_count < 100: imgstr = '00'+str(frame_count)
				    elif frame_count < 1000: imgstr = '0'+str(frame_count)
				    else: imgstr = str(frame_count)
				    full_name = '/media/tbd/4E28-7C59/TBD_vis_odom_Video'+str(vid_count)+'/'+'f'+imgstr+'.jpg'
				    #full_name = '/media/tbd/4E28-7C59/'+'f'+imgstr+'.jpg'
				    #print(full_name)
				    # save the frame
				    cv2.imwrite(full_name,frame)
				    if cv2.waitKey(1) & 0xFF == ord('q'):
						break

				    frame_count += 1

				    delta_microseconds = (dt.datetime.now()-last).microseconds
				    deltas_array.append(delta_microseconds)
				    #print(delta_microseconds)
                                    out.write(frame)
                                    recording_stopped_flag = True
				    recording_started_flag = False

                        else: 
				deltas_array = [] # array of the delta in microseconds between every image
				frame_count = 0
				recording_stopped_flag = False
				recording_started_flag = True
				pass

                        self.r.sleep()


					
                        

if __name__ == '__main__':
        # start the node
        rospy.init_node('tbd_drive_at_deepest')
	TBD_controller = Controller()
	TBD_controller.run()

