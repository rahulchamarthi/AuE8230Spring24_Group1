#!/usr/bin/env python3
import rospy
import numpy as np
import time
from numpy import inf
from geometry_msgs.msg  import Twist, Vector3
from sensor_msgs.msg import LaserScan

start_angle = 20 
side_scanrange      = 60          
front_scanrange     = 16
distancefromwall    = 0.7

x   = np.zeros((360))
fw_d = 0 # front wall distance
lw_d = 0 # left wall distance
rw_d = 0 # right wall distance

kp = 4

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0
        self.prev_time   = 0
   

    def output(self, error, time):
        
        dt = time - self.prev_time 
        if dt > 0:
            
            self.integral += error * dt
            derivative = (error - self.prev_error) / dt
            u = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
            self.prev_error = error 
            self.prev_time = time 
            
            return u 

def scan(data):
	global lw_d, rw_d,x,fw_d,front_scanrange,start_angle,side_scanrange

	x  = list(data.ranges)
	
	for i in range(360):
		if x[i] == inf:
			x[i] = 7
		if x[i] == 0:
			x[i] = 6

        # store scan data 
	lw_d= np.mean(x[30:30+55])          # left wall distance
	rw_d= np.mean(x[330-55:330])  # right wall distance
	fw_d= min(min(x[0:int(front_scanrange/2)],x[int(360-front_scanrange/2):360])) # front wall distance

def wallfollowing_controller():

	
	global kp,s_d,x,y_r,y_l
	global distancefromwall

	

	rospy.init_node('wallfollowing_control', anonymous=True)
	velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	scan_subscriber    = rospy.Subscriber('/scan', LaserScan, scan)
	rate               = rospy.Rate(10)                 

	while not rospy.is_shutdown():
		

		delta = lw_d-rw_d   # distance error 

		
		pid=PID(1,0,1)
		t=time.time()
		P_output  = pid.output(delta,t)

		print(P_output)

		#define min aad max ranges of lin_vel and ang_vel
		angular_zvel = np.clip(P_output,-1.2,1.2)

		linear_vel   = np.clip((fw_d-0.2),-0.1,0.4)
		print(x)

		print('distance from right wall  =',rw_d)
		print('distance from front wall  =',fw_d)
		print('distance from front wall  =')
		print('linear_vel=',linear_vel,' angular_vel=',angular_zvel)
		rospy.loginfo('\n') 

		#publish cmd_vel
		vel_msg = Twist(Vector3(linear_vel,0,0), Vector3(0,0,angular_zvel))
		velocity_publisher.publish(vel_msg)
		rate.sleep()

	velocity_publisher.publish(Twist(Vector3(0,0,0), Vector3(0,0,0)))
	print('Turtlebot stopped')

if __name__ == '__main__':
	try:
		
		wallfollowing_controller()

	except rospy.ROSInterruptException: pass
