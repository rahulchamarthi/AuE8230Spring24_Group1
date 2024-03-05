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
		
		self.rate = rospy.Rate(10) 
		
	def update_pose(self, data): 
		self.pose = data
		self.pose.x = round(self.pose.x, 4) 
		self.pose.y = round(self.pose.y, 4) 
		
	def euclidean_distance(self, goal_pose): 
		return sqrt(math.pow((goal_pose.x - self.pose.x), 2) + math.pow((goal_pose.y - self.pose.y), 2))
		
	def go_in_circle(self): 
		#NOTE: 
		#linear_velocity: m/s 
		#angular_velocity: rad/s 
		linear_velocity = float(input('Please enter your desired linear velocity(m/s):\n')) 
		angular_velocity = float(input('Please enter your desired angular velocity(rad/s):\n')) 
	
		#init a goal state 
		goal_pose = Pose() 
		
		#calculate the amount of time needed to traverse the circumference
		#using time formula =  (pi * diameter)/linear_velocity
		act_radius = linear_velocity / abs(angular_velocity) 
		circumference = (2 * math.pi * act_radius)
		duration = circumference / linear_velocity 
		print('Approximated Time for Traversal Completion: ' + str(duration))
		

		#init vel_msg object
		vel_msg = Twist() 
		
		#init iteration_no so we can keep track of looping
		iteration_no = 0
		
		#iterate through loop until rospy is shutdown 
		start_time = rospy.Time.now().to_sec() 
		while rospy.Time.now().to_sec() - start_time < duration and not rospy.is_shutdown(): 
			#only move x in linear and z in angular
			vel_msg.linear.x = linear_velocity
			vel_msg.linear.y = 0 
			vel_msg.linear.z = 0 
				
			vel_msg.angular.x = 0 
			vel_msg.angular.y = 0 
			vel_msg.angular.z = angular_velocity 
			
			self.velocity_publisher.publish(vel_msg)
			self.rate.sleep() 
		
		#forcing the robot to stop post loop 
		#vel_msg.linear.x = 0 
		#vel_msg.angular.z = 0 
		#self.velocity_publisher.publish(vel_msg)
	
if __name__ == '__main__':
	try:  
		x = TurtleBot() 
		x.go_in_circle() 
	except rospy.ROSInterruptException: 
		pass 
