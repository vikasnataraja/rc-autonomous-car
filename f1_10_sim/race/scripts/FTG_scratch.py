#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt


def getRange(data,angle):
	if angle > 179.9:
		angle = 179.9
	# if angle > np.pi-1e-4:
		# angle = np.pi-1e-4
	index = len(data.ranges)*angle/angle_range
	dist = data.ranges[int(index)]
	if math.isinf(dist) or math.isnan(dist):
		return 4.0
	return data.ranges[int(index)]
	
def scan_callback(data):
	
	print(len(data.ranges))
	print('here')



rospy.init_node('FTG_scratch',anonymous = True)
rospy.Subscriber("scan",LaserScan,scan_callback)
try:
	while ( not rospy.is_shutdown() ):
		pass
		# ~ if __name__ == '__main__':
		# ~ print("Laser node started")

		# ~ rospy.Subscriber( "/scan" , LaserScan , scan_cb )
		# ~ rospy.spin()
	
except KeyboardInterrupt:
	pass
	
