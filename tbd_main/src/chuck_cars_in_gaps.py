from math import pi
import numpy as np
import datetime as dt
import math


class car_ray:
    def __init__(self, ind, scan, scan_deg):
        self.oneshot = False
        self.ind = ind
        self.scan = scan
        self.car_width = 0.75 # meters, this is conservative
        self.dppt = float(scan_deg)/scan.shape[0] # degrees (angle) per laser scan point
        
    def gofar(self):
        max_search_steps = 50 # limit the search to a small number of steps since running in bounded real time is critical
        best_gap_dist = 0
        best_ind = self.ind
        best_ps = self.ind
        best_pe = self.ind
        for its in range(max_search_steps): 
            d = self.scan[self.ind] # d is the distance of a single point that we'll call the center of the "gap" we're checking
            if d < 0.3: d = .3 # d is for gap calculation only, this prevents the gap size being made too large by close collisions
            iwide = np.rad2deg(math.tan(self.car_width/d)) # degrees wide the gap should span at this distance
            # have a max to prevent searching for massive gaps too close to the car. This effectively limits at around 45 degrees of gap
            if iwide > 45: iwide = float(45)
            idwide = int(iwide/self.dppt) # number of scan points wide the gap should be to fit the car
            if idwide < 8: idwide = 8 # have a minimum to prevent outlier effects at far distances
            idwide = int(idwide*1/2) # actually half the width for math convenience later on
            ps = self.ind - idwide # define the start boundary of the gap in lidar scan space
            pe = self.ind + idwide # define the end boundary of the gap in lidar scan space
            if ps < 1: ps = 1 # don't use the very endpoints or go past the end points
            if pe > len(self.scan)-1: pe = len(self.scan)-1
            gap_dist = self.scan[ps:pe].mean() # average distace of the gap
            if self.oneshot and gap_dist - best_gap_dist < 0.1: break # stop the gap traversal search if the avg dist of the gap shrinks

            #record new best gap parameters
            best_gap_dist = gap_dist 
            best_ps = ps
            best_pe = pe
            best_ind = self.ind

            avderiv = np.diff(self.scan[ps:pe]).mean()  # get the average derivative between all scan points in the patch
                                                        # this is used to guide the gap search to deeper areas
            
            if  avderiv > 0: pos_deriv = True # set a sign flag for checking if the derivative switches sign during the search
            else: pos_deriv = False
            
            if self.oneshot and last_pos_deriv != pos_deriv:
                #print('terminate, derivative switched sign')
                break # terminate, reached a maximum
            
            if pos_deriv:   # if the derivative is positive then move the path search in that direction
                self.ind += idwide
            else:           # if the derivative is negative then move the path search in that direction
                self.ind -= idwide
            last_pos_deriv = pos_deriv

            #don't use a buffer around the endpoints
            if self.ind < 10: self.ind = 10
            elif self.ind > len(self.scan)-10: self.ind = len(self.scan)-10
            
            self.oneshot = True
        
        return self.floodfill(best_ind, best_gap_dist, best_ps, best_pe)

    #width is the number of points in the scan allocated to this search
    def fullsearch(self, width):
        num_spots_to_check = 10 #how many candidate gaps to setup (larger numbers can cause issues with timing)
        spot_width = int(math.floor(width / num_spots_to_check))
        current_ind = int(math.floor((self.ind - (width / 2)) + (spot_width / 2)))
        best_gap_dist = 0
        best_ind = self.ind
        best_ps = self.ind
        best_pe = self.ind
        for its in range(num_spots_to_check): 
            d = self.scan[current_ind] # d is the distance of a single point that we'll call the center of the "gap" we're checking
            if d < 0.3: d = .3 # d is for gap calculation only, this prevents the gap size being made too large by close collisions
            iwide = np.rad2deg(math.tan(self.car_width/d)) # degrees wide the gap should span at this distance
            # have a max to prevent searching for massive gaps too close to the car. This effectively limits at around 45 degrees of gap
            if iwide > 45: iwide = float(45)
            idwide = int(iwide/self.dppt) # number of scan points wide the gap should be to fit the car
            if idwide < 8: idwide = 8 # have a minimum to prevent outlier effects at far distances
            idwide = int(idwide*1/2) # actually half the width for math convenience later on
            ps = self.ind - idwide # define the start boundary of the gap in lidar scan space
            pe = self.ind + idwide # define the end boundary of the gap in lidar scan space
            if ps < 1: ps = 1 # don't use the very endpoints or go past the end points
            if pe > len(self.scan)-1: pe = len(self.scan)-1
            gap_dist = self.scan[ps:pe].mean() # average distace of the gap

            #record new best gap parameters
            if(gap_dist > best_gap_dist):
                best_gap_dist = gap_dist 
                best_ps = ps
                best_pe = pe
                best_ind = current_ind
            current_ind += spot_width
        return self.floodfill(best_ind, best_gap_dist, best_ps, best_pe) 

    
    def floodfill(self, best_ind, best_gap_dist, best_ps, best_pe):
        # flood search to try and expand the gap if possible
        # check one way first then the other
        # the success of this assumes that the gap is currently at the max centerpoint, although not necessarily as wide as it could be
        
        # first expand ps to the left
        exp_its = 0
        new_ps = best_ps
        #print('best gap dist:',best_gap_dist)
        if best_gap_dist > 4:
            gap_depth_variation_limit = 1.0 # very large
            move_by = 2
        else: 
            gap_depth_variation_limit = 0.25 # smaller 
            move_by = 1
        
        while (exp_its == 0 or abs(self.scan[new_ps]-best_gap_dist) <= gap_depth_variation_limit) and exp_its < 100:
            if new_ps < 10: break
            else: new_ps -= move_by
            exp_its += 1
        #print('ps: ',exp_its)
        new_ps += move_by # reseting since it violated the stopping condition on the last loop
        new_gap_dist = self.scan[new_ps:best_pe].mean()
        if(abs(new_gap_dist-best_gap_dist) <= gap_depth_variation_limit):
            best_ps = new_ps
            best_gap_dist = new_gap_dist
        
        # now expand pe to the right
        exp_its = 0
        new_pe = best_pe
        #print('best gap dist:',best_gap_dist)
        while (exp_its == 0 or abs(self.scan[new_pe]-best_gap_dist) <= gap_depth_variation_limit) and exp_its < 100:
            if new_pe > len(self.scan)-10: break
            else: new_pe += move_by
            exp_its += 1
        #print('pe: ',exp_its)
        new_pe -= move_by # reseting since it violated the stopping condition on the last loop
        new_gap_dist = self.scan[best_ps:new_pe].mean()
        if(abs(new_gap_dist-best_gap_dist) <= gap_depth_variation_limit):
            best_pe = new_pe
            best_gap_dist = new_gap_dist
        
        # recalculate final center of gap
        best_ind = int((best_ps + (best_pe - best_ps)/2.0)) # recenter ind at the end

        
        return best_ind, best_gap_dist, best_ps, best_pe
        

'''
Class for applying nonholonomic constraints to gaps and finding optimal heading for shooting through a gap
'''
class gap_processor:
    def __init__(self, car_turn_radius, scan_dppt, scan):
        self.min_turn = car_turn_radius #min turn radius of car
        self.dppt = scan_dppt # degrees (angle) per laser scan point
        self.scan = scan

    #Expects gaps to be given in form: [index of the center of the gap, avg distance of the gap, start gap ind, end gap ind]
    #(for now) assuming scan is 180 deg
    def add_constraints(self, gap):
        start_edge = gap[2]
        new_start_edge = self.add_nhnm_constraint(start_edge, 1, gap[0], self.scan[start_edge])

        end_edge = gap[3]
        new_end_edge = self.add_nhnm_constraint(end_edge, -1, gap[0], self.scan[end_edge])

        if(new_end_edge - new_start_edge < 1): return None

        new_avg_dist = self.scan[new_start_edge:new_end_edge].mean() 

        new_gap = [int(math.floor((new_end_edge - new_start_edge)/2)) + start_edge, new_avg_dist, new_start_edge, new_end_edge]
        return new_gap

    #adds single nonholonomic constraint (one side of a gap)
    #gap_side should be either 1 (this is start of gap indices) or -1 (this is end of gap indices)
    #assuming scan is 180 degrees total
    #RETURNS: new index of given obstacle edge
    def add_nhnm_constraint(self, obstacle_edge_index, gap_side, gap_center, ray_dist):
        if(abs(gap_side) > 1): gap_side = np.sign(gap_side)

        #determine deg of heading to the center of this gap
        ray_deg = gap_center*self.dppt
        ray_deg_fixed = (180.0 - ray_deg) if ray_deg > 90.0 else ray_deg
        turn_portion = (90.0 - ray_deg_fixed) / 90.0 #what portion of the minimum turn radius would need to be completed to steer toward this gap

        #add nonholonomic constraint (car turn radius)
        angle_adjust = np.rad2deg(np.arctan2(self.min_turn*turn_portion, ray_dist))

        #add in this angle of the padding to the edge of the gap
        new_obstacle_edge = int(math.floor(obstacle_edge_index + gap_side*(angle_adjust/self.dppt)))
        return new_obstacle_edge



#Inputs:
# scan_in : array of data points
# num_gaps : number of gaps to identify
# scan_deg : total number of degrees covered in this scan (could be less than 360 if only looking at front of car)
def find_gaps(scan_in, num_gaps, scan_deg):
    if num_gaps % 2 == 0: num_gaps += 1 # should always be an odd number of gaps so there is a gap initialized in the center of the trajectory
    start_t = dt.datetime.now() # start timer 
    found_gaps = np.asarray([]) # array for storing the final gaps
    stind = np.linspace(50,len(scan_in)-50,num_gaps)  # starting center indices for each gap, uniformly spread throughout scan
    gap_width = (stind[0] - 50) * 2
    dppt = float(scan_deg)/scan_in.shape[0] # scan angles (degrees) per scan point
    car_min_turn_radius = 0.5 #TUNE THIS

    gap_proc = gap_processor(car_min_turn_radius, dppt, scan_in)
    
    # the following randomized which gaps are started first, in reality all the gap searchers should be started 
    # in threads and the master could do early stopping based on updates of the threads results, this would 
    # increase runtime performance
    order = np.random.permutation(num_gaps) 
    for kk in order:
        ray = int(stind[kk]) # index of the search start point
        a = car_ray(ray,scan_in, scan_deg) # create a gap patch 
        
        # Set it to search for the local deepest
        # returns for the final gap location are:
        # gap center index, average distance (away from car) of the gap
        # start and end index of the gap (determines gap width)
        i,gdist, ps, pe = a.fullsearch(gap_width)

        #process for nonholonomic constraints
        print("Old Gap: ",i,gdist,ps,pe)
        new_gap = gap_proc.add_constraints([i, gdist, ps, pe])
        if(new_gap is None): continue
        print("New Gap: ",new_gap[0],new_gap[1],new_gap[2],new_gap[3])
        
        # concatenate each newly finalized gap location into the returned candidate gaps array
        if found_gaps.shape[0] < 1: # first pass
            found_gaps = np.concatenate((found_gaps.reshape(1,-1),np.asarray(new_gap).reshape(1,-1)),1)
        else: found_gaps = np.concatenate((found_gaps,np.asarray(new_gap).reshape(1,-1)),0)




    elapsed = dt.datetime.now()-start_t # stop timer
    #print('elapsed time in microseconds: ',elapsed.microseconds)
    print('elapsed time in microseconds: ',elapsed.microseconds,' num gaps:',len(found_gaps))
    print(found_gaps)

    return found_gaps # these are candidate gaps






