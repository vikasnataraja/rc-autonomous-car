import rospy
from sensor_msgs.msg import LaserScan
from math import pi
import numpy as np
import datetime as dt
import sys
import matplotlib.pyplot as plt
import signal
from chuck_cars_in_gaps import find_gaps

Simulation = False
        
# laser ray casting max is different in SIM v real
if Simulation: lidar_max_dist = 11
else: lidar_max_dist = 35 # max threshold of 20 meters


def sigint_handler(sig, frame):
    print("Exiting gracefully...")
    sys.exit(0)

def lidar_callback(data):     
    # RP lidar is mounted with wrap around point straight ahead on real car, opposite of sim
    if Simulation: scan_data = np.asarray(data.ranges)
    
    # unwrap scan array, to account for deadpoint being in front not behind
    else: scan_data = np.concatenate((data.ranges[-int(np.ceil(len(data.ranges)/2)):],data.ranges[0:int(np.floor(len(data.ranges)/2))]),0)
    
    # sensor characteristics are different SIM to real for INF handling
    if Simulation: scan_data[np.isinf(scan_data)] = lidar_max_dist
    else: scan_data[np.isinf(scan_data)] = 0
    
    scan_data[np.isnan(scan_data)] = 0
    scan_data[scan_data>15] = lidar_max_dist ### this may be wrong --- check midterm commit 
    scan_data[scan_data<0] = 0
    one_shot = False
    new_scan = True
    #print('scan info:',len(data.ranges),max(scan_data),min(scan_data))
    
    visualizer(scan_data)
    
    
def visualizer(scan_data):
    alpha = 0.001 # higher alpha will prioritize trajectory over gap depth. Alpha also should be smaller for higher fidelity LIDAR scans
    num_gaps = 3
    if Simulation: scan_degs = float(270) # this is dependent on how much of the scan is being used... not sure if this is right
    else: scan_degs = float(180) # 360 # note this is dependent on how how much of the scan is being used
    gaps = find_gaps(scan_data, num_gaps, scan_degs)
    
    straights_cent_setpoint = int( len(scan_data)/2 ) #index of center scanpoint
    
    # gaps = [index of the center of the gap, avg distance of the gap, start gap ind, end gap ind]        
    gap_costs = alpha*abs(gaps[:,0]-straights_cent_setpoint) - (1-alpha)*gaps[:,1] #first term penalizes distance from center, second term rewards average distance
    best_gap_ind = np.where(gap_costs==min(gap_costs))[0][0]
    input_center = int(gaps[best_gap_ind,0])
    safe_dist = gaps[best_gap_ind,1]
    
    
    plt.clf()
    plt.plot(scan_data, 'bo')
    inds = range(len(scan_data))
    #print(gaps.shape[0])
    #print(gaps[:,0])
    plt.plot(input_center,scan_data[input_center],'ro')
    plt.pause(0.0001)
    '''
    for g in range(gaps.shape[0]):
        ps = int(gaps[g,2])
        pe = int(gaps[g,3])
        i = int(gaps[g,0])
        plt.plot(inds[ps:pe], scan_data[ps:pe], 'ko')
        plt.plot(i,scan_data[i],'ro')
    plt.pause(0.0001) # this gets the plot to show
    '''

signal.signal(signal.SIGINT, sigint_handler)
rospy.init_node('gapviewer')
rospy.Subscriber("scan",LaserScan, lidar_callback) # subscribe to lidar scan data
rospy.spin()


