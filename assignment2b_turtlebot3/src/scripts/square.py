#!/usr/bin/env python3
import rospy 
from geometry_msgs.msg import Twist 
from turtlesim.msg import Pose
import math


class TurtleBot: 
	def __init__(self): 
		rospy.init_node('turtlebot_controller', anonymous=True) 
		
		self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10) 
		
		self.pose_subscriber = rospy.Subscriber('/pose', Pose, self.update_pose)
		
		self.pose = Pose() 
		
		self.rate = rospy.Rate(1)
		
	def update_pose(self, data): 
		self.pose = data
		self.pose.x = round(self.pose.x, 4) 
		self.pose.y = round(self.pose.y, 4) 
	
	def forward(self, speed): 
		#knowing that linear velocity is in m/s, we can execute the following equation: 
		#0.2m/s * 10 seconds = 2 meters; formulation that is used to calculate time 
		
		#create the velocity message
		vel_msg = Twist()   
		calc_time = 10 #default value to avoid errors
		side_length = 2  
		
		if speed == 0: 
			vel_msg.linear.x = 0.3
			calc_time = round((side_length / 0.3), 2) 
		elif speed == 1: 
			vel_msg.linear.x = 0.2
			calc_time = round((side_length / 0.2), 2) 
		elif speed == 2: 
			vel_msg.linear.x = 0.4
			calc_time = round((side_length / 0.4), 2)
		elif speed == 3: 
			vel_msg.linear.y = 0.8
			calc_time = round((side_length / 0.8), 2)
		
		#move forward for 10 seconds 
		start_time = rospy.Time.now().to_sec() 
		while (rospy.Time.now().to_sec() - start_time < calc_time) and not rospy.is_shutdown(): 
			self.velocity_publisher.publish(vel_msg)
			self.rate.sleep() 
			
		#stop the turtle 
		vel_msg.linear.x = 0
		self.velocity_publisher.publish(vel_msg) 
		rospy.sleep(1)
		
	def rotate(self, speed): 
		#rotate 90 degrees using the same calculation logic above, but with angular velocity
		vel_msg = Twist() 
		
		#convert the angular speed and relative angle to proper units 
		if speed == 0: 
			angular_speed = 0.3 #already in radians/sec 
		elif speed == 1: 
			angular_speed = 0.2
		elif speed == 2: 
			angular_speed = 0.4
		elif speed == 3: 
			angular_speed = 0.8
			
		relative_angle = 90 * 2 * math.pi/360 
		
		#linear components not necessary so 0 them out 
		vel_msg.linear.x = 0 
		vel_msg.linear.y = 0 
		vel_msg.linear.z = 0 
		
		#also 0 out x and y angular changes
		vel_msg.angular.x = 0 
		vel_msg.angular.y = 0 
		
		#set angular speed 
		vel_msg.angular.z = abs(angular_speed) 
		
		start_time = rospy.Time.now().to_sec() 
		current_angle = 0 
		
		while(current_angle < relative_angle): 
			self.velocity_publisher.publish(vel_msg)
			current_time = rospy.Time.now().to_sec() 
			current_angle = angular_speed * (current_time - start_time) 
		
		#stop the turtle 
		vel_msg.angular.z = 0 
		self.velocity_publisher.publish(vel_msg) 
		rospy.sleep(1)
		
	def square_movement(self): 
		for speed in range(0, 4): 
			if speed == 0: 
				print('running square loop with DEFAULT speed: angular velocity = 0.3, linear velocity = 0.3')
			elif speed == 1: 
				print('running square loop with SLOW speed: angular velocity = 0.2, linear velocity = 0.2')
			elif speed == 2: 
				print('running square loop with MEDIUM speed: angular velocity = 0.4, linear velocity = 0.4')
			elif speed == 3: 
				print('running square loop with FAST speed: angular velocity = 0.8, linear velocity = 0.8')
				
			#draw line for base of square
			self.forward(speed) 
			#rotate the turtle 90 degrees 
			self.rotate(speed) 
			#draw right side of square 
			self.forward(speed) 
			#rotate the turtle 90 degrees 
			self.rotate(speed) 
			#draw top of the square 
			self.forward(speed) 
			#rotate the turtle 90 degrees
			self.rotate(speed) 
			#draw the left side of the square
			self.forward(speed) 
			#rotate the turtle back to starting position 
			self.rotate(speed) 
		
if __name__ == '__main__':
	try: 
		x = TurtleBot() 
		x.square_movement()  
	except rospy.ROSInterruptException: 
		pass 
