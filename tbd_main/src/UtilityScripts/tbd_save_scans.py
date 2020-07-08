import rospy
from sensor_msgs.msg import Joy
from sensor_msgs.msg import LaserScan
from ros_pololu_servo.msg import TBD_drive_cmds
from math import pi
import numpy as np
import datetime as dt
import sys
import matplotlib.pyplot as plt
import signal


Simulation = True
scan_hist = np.asarray([])
save_flag = True
        
# laser ray casting max is different in SIM v real
if Simulation: lidar_max_dist = 11
else: lidar_max_dist = 35 # max threshold of 20 meters

def save():
    print(' ')
    print('saving!')
    print(' ')
    np.save('scan_hists.npy',scan_hist)

def sigint_handler(sig, frame):
    print("Exiting gracefully...")
    save()
    sys.exit(0)

def lidar_callback(data):
    global save_flag, scan_hist 
     
    # RP lidar is mounted with wrap around point straight ahead on real car, opposite of sim
    scan_data = np.asarray(data.ranges)
    
    one_shot = False
    new_scan = True
    #print('scan info:',len(data.ranges),max(scan_data),min(scan_data))
    
    number_of_frames_to_save = 1500
    if scan_hist.shape[0] < number_of_frames_to_save:
        if scan_hist.shape[0] == 0:
            scan_hist = scan_data.reshape(1,scan_data.shape[0])
        else: scan_hist = np.concatenate((scan_hist,scan_data.reshape(1,scan_data.shape[0])),0)
    elif save_flag:
        save()
        save_flag = False
    print('scans saved:',scan_hist.shape[0])

signal.signal(signal.SIGINT, sigint_handler)
rospy.init_node('scanlistener')
rospy.Subscriber("scan",LaserScan, lidar_callback) # subscribe to lidar scan data
rospy.spin()

