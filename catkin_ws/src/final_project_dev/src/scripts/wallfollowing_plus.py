#!/usr/bin/env python3
import rospy
import numpy as np
import time
from numpy import inf
from geometry_msgs.msg  import Twist, Vector3
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

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

bot_x = 0 
bot_y = 0 
bot_z = 0 

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
	fw_d= min(float(fw_sec_one), float(fw_sec_two)) # front wall distance
	
	# print('MAIN: min distance right sector / right wall  =', rw_dist_actual)
	# print('MAIN: min distance left sector / left wall  =', lw_dist_actual)
	# print('MAIN: min distance front sector #1 =', fw_sec_one)
	# print('MAIN: min distance front sector #2 =', fw_sec_two)
	# print('\n')

def odometryCb(msg):
	global bot_x, bot_y, bot_z 

	bot_x = msg.pose.pose.position.x 
	bot_y = msg.pose.pose.position.y 
	bot_z = msg.pose.pose.position.z 

def free_space(sector_dist):
	free_space_mask=[]
	treshold_dist=min(sector_dist)+0.3
	for i in range(len(sector_dist)):
		if sector_dist[i] <= treshold_dist:
			free_space_mask.append(0)
		else:
			free_space_mask.append(sector_dist[i])

	return free_space_mask 

def max_gap(free_space_mask):
	#returns the index ox of centre of max gap
	masked_data=np.ma.masked_equal(free_space_mask,0)
	gaps = np.ma.notmasked_contiguous(masked_data)
	max_len = gaps[0].stop - gaps[0].start
	chosen_slice = gaps[0]

	for gap in gaps[1:]:
				
		gap_len = gap.stop - gap.start
		if gap_len > max_len:
			max_len = gap_len
			chosen_slice = gap

	return (chosen_slice.start+chosen_slice.stop-1)/2
				
def desired_angle(index):
	theta_d=10*(9-index)
	
	return theta_d 

def wallfollowing_controller():
	global kp,s_d,x,y_r,y_l
	global distancefromwall

	rospy.init_node('wallfollowing_control', anonymous=True)
	velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	odometery_publisher = rospy.Subscriber('odom', Odometry, odometryCb)
	scan_subscriber    = rospy.Subscriber('/scan', LaserScan, scan)
	rate               = rospy.Rate(10)        
	start_time = time.time()

	pid=PID(1,0,1)
	pid_oa=PID(20,0,5)

	# buffer variables 
	side_min_dist = 0.2
	front_min_dist = 0.5

	switch_condition = False 

	counter = 0 
	first_wall_x = 0.0 
	first_wall_y = 0.0 

	while not rospy.is_shutdown():
		current_time = time.time()
		
		if current_time - start_time > 2.5: 
			delta = lw_d-rw_d   # distance error

			
			# pid=PID(1,0,1)
			t=time.time()

			#P_output  = pid.output(delta,t)

			sector_dist=np.empty(18)
			n=0 
			for sector_num in range(9):
				sector_dist[sector_num]=np.mean(x[80-n:90-n])
				n=n+10
			n=0
			for sector_num in range(9):
				sector_dist[9+sector_num]=np.mean(x[350-n:360-n])
				n=n+10
			free_space_mask=free_space(sector_dist)
			max_gap_center=max_gap(free_space_mask)
			theta_d=desired_angle(max_gap_center)

			if rw_dist_actual >= side_min_dist and lw_dist_actual >= side_min_dist and fw_sec_one >= front_min_dist and fw_sec_two >= front_min_dist:
				linear_vel = np.clip((fw_d-0.2),-0.1,0.2)
				angular_zvel = np.clip(pid.output(delta,t),-0.8,0.8)
				rospy.loginfo('WALL FOLLOWING')

			else: 
				linear_vel   = np.clip(0.1,-0.1,0.1)
				angular_zvel = np.clip(0.05*theta_d,-1,1)
				rospy.loginfo('OBSTACLE AVOIDANCE')

			print('min distance right sector / right wall  =', rw_dist_actual)
			print('min distance left sector / left wall  =', lw_dist_actual)
			print('min distance front sector #1 =', fw_sec_one)
			print('min distance front sector #2 =', fw_sec_two)
			print('linear_vel=',linear_vel,' angular_vel=',angular_zvel)
			print('odom x position =', bot_x)
			print('odom y position =', bot_y)
			print('odom z position =', bot_z)
			# print('OBSTACLE COUNTER COUNT =', counter)
			print('start time', start_time) 
			print('current_time', time.time())
			rospy.loginfo('\n') 

			# #publish cmd_vel
			#vel_msg = Twist(Vector3(linear_vel,0,0), Vector3(0,0,angular_zvel))
			vel_msg = Twist(Vector3(0,0,0), Vector3(0,0,0))
			velocity_publisher.publish(vel_msg)
			rate.sleep()

	velocity_publisher.publish(Twist(Vector3(0,0,0), Vector3(0,0,0)))
	print('Turtlebot stopped')

if __name__ == '__main__':
	try:
		wallfollowing_controller()

	except rospy.ROSInterruptException: pass