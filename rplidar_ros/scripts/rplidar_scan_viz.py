import rospy
from sensor_msgs.msg import LaserScan
from math import pi
import numpy as np
import sys
import matplotlib.pyplot as plt
import datetime as dt


class viz():
	def __init__( self ):
		# Intialize everything
		
		# ------------- initialize variables ---------------
		# ------ END of initialize variables ---------------
		
		# ------------- sensor input topic subscriptions ---------------
		rospy.Subscriber("scan",LaserScan, self.lidar_callback) # subscribe to lidar scan data
		# ------ END of sensor input topic subscriptions ---------------


        def lidar_callback(self, data):

                
                self.scan_data = np.asarray(data.ranges)
                self.scan_data[np.isinf(self.scan_data)] = 0
                self.scan_data[np.isnan(self.scan_data)] = 0
                self.scan_data[np.isnan(self.scan_data)] = 0
                self.scan_data[self.scan_data<0] = 0
                #print('scan info:',len(data.ranges),max(self.scan_data),min(self.scan_data))
                
                print(np.mean(self.scan_data))

        def signal_handler(sig,frame):
            sys.exit(0)
            
            
                        
if __name__ == '__main__':
        # start the node
        signal.signal(signal.SIGINT,signal_handler)
        rospy.init_node('doathing')
	scan_viz = viz()
	#rospy.spin()
        
        
        while 1:
                init_time = dt.datetime.now()
                plt.clf()
                plt.plot(scan_viz.scan_data,'bo')
                plt.ylim( [ 0 , 20 ] )
                plt.xlim( [ 0 , 1450] )
                plt.hold( True )
                        
                plt.pause( 0.001 )
                #print(dt.datetime.now()-init_time)
