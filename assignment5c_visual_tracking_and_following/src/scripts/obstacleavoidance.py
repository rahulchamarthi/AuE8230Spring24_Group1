#!/usr/bin/env python3
import rospy
import numpy as np
from numpy import inf
from geometry_msgs.msg  import Twist, Vector3
from sensor_msgs.msg import LaserScan

start_angle = 40 
side_scanrange      = 50          
front_scanrange     = 20
distancefromwall    = 0.6

x   = np.zeros((360))
fw_d = 0 # front wall distance
lw_d = 0 # left wall distance
rw_d = 0 # right wall distance

kp = 4


def scan(data):
	global lw_d, rw_d,x,fw_d,front_scanrange,start_angle,side_scanrange

	x  = list(data.ranges)
	
	for i in range(360):
		if x[i] == inf:
			x[i] = 7
		if x[i] == 0:
			x[i] = 6

        # store scan data 
	lw_d= min(x[start_angle:start_angle+side_scanrange])          # left wall distance
	rw_d= min(x[360-start_angle-side_scanrange:360-start_angle])  # right wall distance
	fw_d= min(min(x[0:int(front_scanrange/2)],x[int(360-front_scanrange/2):360])) # front wall distance

def obstacleavoidance_controller():

	
	global kp,s_d,x,y_r,y_l
	global distancefromwall

	

	rospy.init_node('obstacleavoidance_control', anonymous=True)
	velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	scan_subscriber    = rospy.Subscriber('/scan', LaserScan, scan)
	rate               = rospy.Rate(10)                 

	while not rospy.is_shutdown():

		if fw_d < 1:
			linear_vel=0.0
			angular_zvel=1
		else:
			linear_vel= 0.4
			angular_zvel=0.0

		print('distance from front wall  =',fw_d)
		print('linear_vel=',linear_vel,' angular_vel=',angular_zvel)
		

		#publish cmd_vel
		vel_msg = Twist(Vector3(linear_vel,0,0), Vector3(0,0,angular_zvel))
		velocity_publisher.publish(vel_msg)
		rate.sleep()

	velocity_publisher.publish(Twist(Vector3(0,0,0), Vector3(0,0,0)))
	print('Turtlebot stopped')

if __name__ == '__main__':
	try:
		
		obstacleavoidance_controller()

	except rospy.ROSInterruptException: pass
