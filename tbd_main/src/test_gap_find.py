from math import pi
import numpy as np
import datetime as dt
import math
import matplotlib.pyplot as plt
import sys
from chuck_cars_in_gaps import find_gaps

#USAGE: python test_gap_find.py [scan filename] [index to start at in scan file]

scan_str = 'REAL_FTG_scan_hists.npy' #filename of scan file to use
#scan_str = 'FTG_SIM_scan_hists.npy' #filename of scan file to use
#scan_str = 'REAL_scan_hists.npy' #filename of scan file to use

Simulation = False

scan_start_pt = 160 # use this to start part way through a scan file, eg to investigate turns quickly
if(len(sys.argv) > 1):
    scan_str = sys.argv[1]
    if(len(sys.argv) > 2):
        scan_start_pt = eval(sys.argv[2])

if Simulation: lidar_max_dist = 11
else: lidar_max_dist = 35 # max threshold of 20 meters

def lidar_scan_preprocess(laserscan):
    # RP lidar is mounted with wrap around point straight ahead on real car, opposite of sim
    if Simulation: scan_data = np.asarray(laserscan)
    # following line unwraps lidar data since the end points in real life are directly ahead
    else: scan_data = np.concatenate((laserscan[-int(np.ceil(len(laserscan)/2)):],laserscan[0:int(np.floor(len(laserscan)/2))]),0) 
    
    # sensor characteristics are different SIM to real for INF handling
    if Simulation: scan_data[np.isinf(scan_data)] = lidar_max_dist
    else: scan_data[np.isinf(scan_data)] = 0
    
    scan_data[np.isnan(scan_data)] = 0
    scan_data[scan_data>lidar_max_dist] = lidar_max_dist
    scan_data[scan_data<0] = 0
    
    # SIMulation lidar covers 270 degrees, REAL lidar covers 360 degrees 
    if Simulation: front_scan = scan_data[int(len(scan_data)/6):5*int(len(scan_data)/6)] # should grab the front 180 degrees
    else: front_scan = scan_data[int(len(scan_data)/4):3*int(len(scan_data)/4)] # should grab the front 180 degrees
    
    # TODO --- make this not so loopy and slow, FIRST measure the elapsed time though to see if it is slow.
    ## filter out zero points that don't have huge patches of zeros
    # the idea is that this should ditch points that are reflections and such
    # but not cause the car to actually ram a reflective door
    zeros_inds = np.where(front_scan == 0.0)[0]
    for pt in zeros_inds:
        if pt + 12 < front_scan.shape[0]: pe = pt + 12
        else: pe = front_scan.shape[0] - 1
        if pt - 12 > 0: ps = pt - 12 
        else: ps = 1
        
        num_untrustworthy_neighbors = len(np.where(front_scan[ps:pe] == 0)[0])
        if num_untrustworthy_neighbors < 22:
            # we will filter
            
            #print(front_scan[ps:pe])
            #print(np.where(front_scan[ps:pe] != 0))
            if len(np.where(front_scan[ps:pe] != 0)[0]) > 0:
                replace_val = front_scan[ps:pe][np.where(front_scan[ps:pe] != 0)[0]].mean()
                #print(replace_val)
                
                # will update pt index with this 
                front_scan[pt] = replace_val
        #print()
        
    return front_scan
        
data = np.load(scan_str)
try:
    for dd in range(scan_start_pt,data.shape[0]):
        #print('showing scan ',dd,' of ',data.shape[0])
        fdat = data[dd,:]

        fdat = lidar_scan_preprocess(fdat)
        
        straights_cent_setpoint = int( len(fdat)/2 ) #index of center scanpoint
        
        inds = range(len(fdat))

        alpha = 0.001 # higher alpha will prioritize trajectory over gap depth. Alpha also should be smaller for higher fidelity LIDAR scans
        num_gaps = 5
        # currently both the simulator and the real are using "front_scan" which is the forward 180 degrees of the lidar 
        scan_degs = float(180)
        gaps = find_gaps(fdat, num_gaps, scan_degs)
        # # gaps = [index of the center of the gap, avg distance of the gap, start gap ind, end gap ind]        
        gap_costs = alpha*abs(gaps[:,0]-straights_cent_setpoint) - (1-alpha)*gaps[:,1]  #first term penalizes distance from center, second term rewards average distance
        gap_widths = abs(gaps[:,2]-gaps[:,3])
        #print(gap_widths)  # include gap widths in the gap costs calc?
        #print(gap_costs)
        best_gap_ind = np.where(gap_costs==min(gap_costs))[0][0]
        #print(best_gap_ind)
            
        plt.clf()
        plt.plot(fdat,'bo')
        
        for g in range(gaps.shape[0]):
            ps = int(gaps[g,2])
            pe = int(gaps[g,3])
            i = int(gaps[g,0])
            plt.plot(inds[ps:pe],fdat[ps:pe],'ko')
            plt.plot(i,fdat[i],'ro')
            
        # show the chosen best gap
        if 1:
            ps = int(gaps[best_gap_ind,2])
            pe = int(gaps[best_gap_ind,3])
            i = int(gaps[best_gap_ind,0])
            plt.plot(inds[ps:pe],fdat[ps:pe],'go')
            plt.plot(i,fdat[i],'rx')
            
        plt.ylim( [ 0 , 36 ] )
        plt.pause(0.001)
        #plt.show()
except KeyboardInterrupt:
    plt.close()
    pass
