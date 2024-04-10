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
rw_dist_actual = 0 
lw_dist_actual = 0
fw_sec_one = 0
fw_sec_two = 0
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
	global lw_d, rw_d, rw_dist_actual, lw_dist_actual,x,fw_d,fw_sec_one, fw_sec_two, front_scanrange,start_angle,side_scanrange

	x  = list(data.ranges)
	
	for i in range(360):
		if x[i] == inf:
			x[i] = 7
		if x[i] == 0:
			x[i] = 6

        # store scan data 
	#TODO: ADD PARAMETERIZATION BACK INTO THE STUFF DOWN BELOW
	lw_d= np.mean(x[30:30+55])          # left wall distance 
	rw_d= np.mean(x[330-55:330])  # right wall distance
	
	lw_dist_actual = min(x[30:85])
	rw_dist_actual = min(x[275:330])
	fw_sec_one = min(x[0:30])
	fw_sec_two = min(x[330:360]) 
	#fw_d= min(min(fw_sec_one, fw_sec_two)) # front wall distance

def wallfollowing_controller():
	global kp,s_d,x,y_r,y_l
	global distancefromwall

	

	rospy.init_node('wallfollowing_control', anonymous=True)
	velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	scan_subscriber    = rospy.Subscriber('/scan', LaserScan, scan)
	rate               = rospy.Rate(10)          

	# buffer variables 
	side_min_dist = 0.15
	front_min_dist = 0.2

	while not rospy.is_shutdown():
		delta = lw_d-rw_d   # distance error

		
		pid=PID(1,0,1)
		t=time.time()
		P_output  = pid.output(delta,t)

		print(P_output)


		if rw_dist_actual >= side_min_dist and lw_dist_actual >= side_min_dist and fw_sec_one >= front_min_dist and fw_sec_two >= front_min_dist:
			#normal behavior 
			rospy.loginfo('NORMAL BEHAVIOR DETECTED')
			angular_zvel = np.clip(P_output,-1.2,1.2)
			linear_vel   = np.clip((fw_d-0.2),-0.1,0.2)

		else: 
			rospy.loginfo('DUMB SHIT DETECTED')
			#select the max of the mins and proceed forward 
			all_distances = [int(rw_dist_actual), int(lw_dist_actual), int(fw_sec_one), int(fw_sec_two)]
			max_val_index = all_distances.index(max(all_distances))
	
			#TODO: if max_index = 0 (RIGHT)
			if max_val_index == 0 or max_val_index == 3: 
				linear_vel = 0 
				angular_zvel = -1 

			#TODO: elif max_index = 1 (LEFT)
			elif max_val_index == 1 or max_val_index == 2: 
				linear_vel = 0
				angular_zvel = 1

			# #TODO: elif max_index = 2 (FRONT WALL - S1)
			# elif max_val_index == 2: 

			# #TODO: elif max_index = 3 (FRONT WALL - S2)
			# elif max_val_index == 3: 
  
	

		#print(x)

		print('min distance right sector / right wall  =', rw_dist_actual)
		print('min distance left sector / left wall  =', lw_dist_actual)
		print('min distance front sector #1 =', fw_sec_one)
		print('min distance front sector #2 =', fw_sec_two)
		# print('linear_vel=',linear_vel,' angular_vel=',angular_zvel)
		rospy.loginfo('\n') 

		# #publish cmd_vel
		vel_msg = Twist(Vector3(linear_vel,0,0), Vector3(0,0,angular_zvel))
		velocity_publisher.publish(vel_msg)
		rate.sleep()

	velocity_publisher.publish(Twist(Vector3(0,0,0), Vector3(0,0,0)))
	print('Turtlebot stopped')

if __name__ == '__main__':
	try:
		wallfollowing_controller()

	except rospy.ROSInterruptException: pass