#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from sensor_msgs.msg import LaserScan
from ros_pololu_servo.msg import TBD_drive_cmds
from math import pi
import numpy as np
import datetime as dt
import sys
import matplotlib.pyplot as plt
import time
from chuck_cars_in_gaps import find_gaps
# ------------- get launch file parameters ---------------

kill_switch = rospy.get_param("Kill_switch_val") # global for E-stop
car_control_manual = rospy.get_param("Manual_mode_val") # global for manual control
Simulation = rospy.get_param("SIM") # running simulation or on robot 

#kill_switch = False
#car_control_manual = False
#Simulation = True

verbose = False # set to True for gaps visualization

# ------------- setup global vars ---------------
Sim_scale = 6 # since the simulation runs in m/s and the servo operates on -1 to 1 

# laser ray casting max is different in SIM v real
if Simulation: lidar_max_dist = 11
else: lidar_max_dist = 35 # max threshold of 20 meters
# ------ END of setup global vars ---------------

class Controller:
    def __init__( self ):
        # Intialize everything
        
        # ------------- initialize variables ---------------
        self.command_msg = TBD_drive_cmds()
        self.command_msg.throttle_cmd = 0
        self.command_msg.steering_angle = 0 # max min pi/2 -pi/2
        self.debug_sent_time = dt.datetime.now()
        self.Debug_msg = ''
        self.scan_data_rdy = False
        self.new_scan = False
        self.one_shot = True
        
        self.num_largest = 75 # this will need to be tuned
        self.Kp = 0.006 # need to tune with speed adjustments
        
        self.Kp_turn = 0.002 # need to tune with speed adjustments
        self.front_depth_window = 50 # number of indices around center (forward) to evaluate for wall being close and starting a turn
        self.turn_thresh = 6 # meters till know at a turn # look at midterm commit! but seems to have been 6

        if Simulation:
            self.max_speed = 1.0
            self.min_speed = 0.5
        else: # real speed caps below
            self.max_speed = 0.2 
            self.min_speed = 0.15 
            
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
        global car_control_manual, kill_switch

        self.throttle_axis = data.axes[1] # this is the left stick. 1 is full forward, -1 is full reverse
        self.steering_axis = data.axes[2] # this is the right stick. -1 is full right, 1 is full left

        kill_switch_btn = data.buttons[2] # this is the A button -- preferable to prevent occasional accidental kills
        auto_manual_btn = data.buttons[0] # this is the Y button

        if auto_manual_btn: 
            car_control_manual = not car_control_manual # toggle auto manual mode
            if car_control_manual: self.Debug_msg += "Car is now in MANUAL control mode"
            else: self.Debug_msg += "Car is now in Autonomous control mode"

        if kill_switch_btn: 
            kill_switch = not kill_switch
            if kill_switch: self.Debug_msg += "Kill switch triggered, car is now in SAFE mode"
            else: self.Debug_msg += "Kill switch deactivated, car is now in DRIVE mode"
        
    def lidar_scan_preprocess(self, laserscan):
        # RP lidar is mounted with wrap around point straight ahead on real car, opposite of sim
        if Simulation: self.scan_data = np.asarray(laserscan)
        # following line unwraps lidar data since the end points in real life are directly ahead
        else: self.scan_data = np.concatenate((laserscan[-int(np.ceil(len(laserscan)/2)):],laserscan[0:int(np.floor(len(laserscan)/2))]),0) 
        
        # sensor characteristics are different SIM to real for INF handling
        if Simulation: self.scan_data[np.isinf(self.scan_data)] = lidar_max_dist
        else: self.scan_data[np.isinf(self.scan_data)] = 0
        
        self.scan_data[np.isnan(self.scan_data)] = 0 
        self.scan_data[self.scan_data>lidar_max_dist] = lidar_max_dist
        self.scan_data[self.scan_data<0] = 0
        
        # SIMulation lidar covers 270 degrees, REAL lidar covers 360 degrees 
        if Simulation: self.front_scan = self.scan_data[int(len(self.scan_data)/6):5*int(len(self.scan_data)/6)] # should grab the front 180 degrees
        else: self.front_scan = self.scan_data[int(len(self.scan_data)/4):3*int(len(self.scan_data)/4)] # should grab the front 180 degrees
        
        #TODO --- make this not so loopy and slow, FIRST measure the elapsed time though to see if it is slow.
        ## filter out zero points that don't have huge patches of zeros
        # the idea is that this should ditch points that are reflections and such
        # but not cause the car to actually ram a reflective door
        zeros_inds = np.where(self.front_scan == 0.0)[0]
        for pt in zeros_inds:
            if pt + 12 < self.front_scan.shape[0]: pe = pt + 12
            else: pe = self.front_scan.shape[0] - 1
            if pt - 12 > 0: ps = pt - 12 
            else: ps = 1
            
            num_untrustworthy_neighbors = len(np.where(self.front_scan[ps:pe] == 0)[0])
            if num_untrustworthy_neighbors < 22: # we will filter
                if len(np.where(self.front_scan[ps:pe] != 0)[0]) > 0:
                    replace_val = self.front_scan[ps:pe][np.where(self.front_scan[ps:pe] != 0)[0]].mean()
                    self.front_scan[pt] = replace_val
        
    def lidar_callback(self, data):
        if not self.one_shot:
            self.scan_data_rdy = True
            self.last_scan_data = self.scan_data
         
        self.lidar_scan_preprocess(data.ranges)

        self.one_shot = False
        self.new_scan = True

    def Drive_at_deepest(self):
        s_time = time.time()
        self.new_scan = False # reset this flag since we only update our control output if we get new scan data
            
        self.straights_cent_setpoint = int( len(self.front_scan)/2 ) #index of center scanpoint
        
        
        alpha = 0.001 # higher alpha will prioritize trajectory over gap depth. Alpha also should be smaller for higher fidelity LIDAR scans
        num_gaps = 5
        # currently both the simulator and the real are using "front_scan" which is the forward 180 degrees of the lidar 
        scan_degs = float(180)
        gaps = find_gaps(self.front_scan, num_gaps, scan_degs)
        if(len(gaps) == 0): return #just maintain current heading
        
        # # gaps = [index of the optimal steering for the gap, avg distance of the gap, start gap ind, end gap ind]        
        gap_costs = alpha*abs(gaps[:,0]-self.straights_cent_setpoint) - (1-alpha)*gaps[:,1] #first term penalizes distance from center, second term rewards average distance
        best_gap_ind = np.where(gap_costs==min(gap_costs))[0][0]
        input_center = int(gaps[best_gap_ind,0])
        safe_dist = gaps[best_gap_ind,1]
        
        
        #Display (optional)
        if Simulation and verbose: # doing some visualizatoion
            plt.clf()
            plt.plot(self.front_scan, 'bo')
            inds = range(len(self.front_scan))
            for g in range(gaps.shape[0]):
                ps = int(gaps[g,2])
                pe = int(gaps[g,3])
                i = int(gaps[g,0])
                plt.plot(inds[ps:pe], self.front_scan[ps:pe], 'ko')
                plt.plot(i,self.front_scan[i],'ro')
            plt.pause(0.0001) # this gets the plot to show
        #alternative approach: average N largest indices:
        
        
        '''
        self.sorted_scan_inds = np.argsort(self.front_scan) # sorted from smallest to largest
        self.N_largest_inds = self.sorted_scan_inds[-self.num_largest:] # array of indices of the N largest scan points

        # these should be equivalent
        #largest = self.front_scan[self.N_largest_inds]
        largest = [self.front_scan[i] for i in self.N_largest_inds]
        safe_dist = np.mean(largest)
        #print(largest)

        #print('stand dev:',np.std(self.N_largest_inds))
        input_center = np.mean( self.N_largest_inds ) # center index
        '''
        
        translation_err = input_center - self.straights_cent_setpoint
        if np.mean(self.front_scan[int(len(self.front_scan)/2)-int(self.front_depth_window/2):int(len(self.front_scan)/2)+int(self.front_depth_window/2)]) > self.turn_thresh: 
            cntrl_steer = self.Kp*translation_err
        else: cntrl_steer = self.Kp_turn*translation_err
        
        # note currently this only works with 180 degree front scans!
        hallway_centering = True
        if hallway_centering:
        
            right_dead_ahead = self.front_scan[int(len(self.front_scan)*3/8):int(len(self.front_scan)/2)].mean()
            left_dead_ahead = self.front_scan[int(len(self.front_scan)/2):int(len(self.front_scan)*5/8)].mean()
            
            right_forward_shoulder = self.front_scan[int(len(self.front_scan)*1/8):int(len(self.front_scan)*3/8)].mean()
            left_forward_shoulder = self.front_scan[int(len(self.front_scan)*5/8):int(len(self.front_scan)*7/8)].mean()

            right_side = self.front_scan[0:30].mean()
            left_side = self.front_scan[-30:].mean()
            '''
            # dead aheads
            dead_ahead_thresh = 1.0
            if left_dead_ahead < dead_ahead_thresh: 
                cntrl_steer -= 0.5*abs(dead_ahead_thresh-left_dead_ahead)
                print('avoiding left dead ahead captn!')
            elif right_dead_ahead < dead_ahead_thresh:
                cntrl_steer += 0.5*abs(dead_ahead_thresh-right_dead_ahead)
                print('avoiding right dead ahead captn!')
            '''
            # shoulders then sides
            shoulder_thresh = 0.75
            hip_thresh = 0.75
            shoulder_kp = 0.5
            hip_kp = 0.25
            if left_forward_shoulder < shoulder_thresh:
                cntrl_steer -= shoulder_kp*abs(shoulder_thresh - left_forward_shoulder)
                if Simulation: print('avoiding left shoulder captn!')
            elif left_side < hip_thresh:
                cntrl_steer -= hip_kp*abs(hip_thresh - left_side)
                if Simulation: print('avoiding left hip captn!')
                
            if right_forward_shoulder < shoulder_thresh:
                cntrl_steer += shoulder_kp*abs(shoulder_thresh - right_forward_shoulder)
                if Simulation: print('avoiding right shoulder captn!')
            elif right_side < hip_thresh:
                cntrl_steer += hip_kp*abs(hip_thresh - right_side)
                if Simulation: print('avoiding right hip captn!')

        
        self.command_msg.steering_angle = -1*cntrl_steer

        if np.degrees(abs(cntrl_steer)) > 6: speed_cmd = self.min_speed # slow down when large steering corrections are needed
        else: speed_cmd = safe_dist*self.speed_scale - 0.6

        if speed_cmd > self.max_speed: speed_cmd = self.max_speed
        if speed_cmd < self.min_speed: speed_cmd = self.min_speed
        self.command_msg.throttle_cmd = speed_cmd

        #print('err',translation_err,'steer:',np.degrees(cntrl_steer),'gap distance',safe_dist,'time',time.time() - s_time)
        self.Debug_msg += 'err'+str(translation_err)+'steer:'+str(np.degrees(cntrl_steer))+'dist largest'+str(safe_dist)
        self.trim_controls()

    
    def trim_controls(self):
        # limit the steering control...
        if self.command_msg.steering_angle > pi/2: self.command_msg.steering_angle = pi/2
        elif self.command_msg.steering_angle < -pi/2: self.command_msg.steering_angle = -pi/2

        if Simulation: # scale variables as needed for simulation
            self.command_msg.throttle_cmd = Sim_scale * self.command_msg.throttle_cmd
            self.command_msg.steering_angle = -1*self.command_msg.steering_angle

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

                if self.command_msg.throttle_cmd >= 0: self.command_msg.throttle_mode = FORWARD 
                else: self.command_msg.throttle_mode = REVERSE

        self.command_publisher.publish(self.command_msg)

    def run(self):
        
        while (not rospy.is_shutdown()):
            #if self.scan_data_rdy:
                #print(np.mean(self.scan_data))
            now = dt.datetime.now() # updates with current time. This shouldn't be set/reset anywhere else.
            self.Debug_msg = '' 
            # inputs are handled automatically through call backs
            
            # call control strategy
            if self.scan_data_rdy and self.new_scan:
                self.Drive_at_deepest()
            
            # outputs
            self.outputs()
            
            debug_publish_period = 1 # seconds
            if (now - self.debug_sent_time).seconds >= debug_publish_period:
                self.debug_sent_time = now
                print(self.Debug_msg)
            self.r.sleep()
                    
                        

if __name__ == '__main__':
    # start the node
    rospy.init_node('tbd_drive_at_deepest')
    TBD_controller = Controller()
    TBD_controller.run()

