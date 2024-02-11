#!/usr/bin/env python3
import rospy 
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
# from turtlesim.msg import Pose
import math
import numpy as np

class turtlebot():
    def __init__(self):
        #Creating our node,publisher and subscriber
        rospy.init_node('turtlebot_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/odom', Odometry, self.getOdom)
        self.odom = Odometry()
        self.vel = Twist()
        self.vel.linear.y = 0
        self.vel.linear.z = 0
        self.vel.angular.x = 0
        self.vel.angular.y = 0
        self.rate = rospy.Rate(10)

    #Callback function implementing the pose value received
    def getOdom(self, data):
        self.odom = data


    def moveInCircle(self):
        radius = 1
        for i in range(4):
            # get speeds
            speed = 0.1*np.exp(i) # run at different speeds
            omega = speed/radius
            distance = 2*math.pi*radius

            #Set Twist msg
            self.vel.linear.x= speed
            self.vel.angular.z = omega

            #Setting the current time for distance calculus
            t0 = rospy.Time.now().to_sec()
            current_distance = 0

            #Loop to move the turtle for a specified distance
            while(current_distance < distance):
                #Publish the velocity
                self.velocity_publisher.publish(self.vel)
                # print(self.odom)
                #Takes actual time to velocity calculus
                t1=rospy.Time.now().to_sec()
                #Calculates distancePoseStamped
                current_distance= speed*(t1-t0)
            #After the loop, stops the robot
            self.vel.linear.x = 0
            self.vel.angular.z = 0
            #Force the robot to stop
            self.velocity_publisher.publish(self.vel)
   

if __name__ == '__main__':
    try:
        #Testing our function
        x = turtlebot()
        x.moveInCircle()

    except rospy.ROSInterruptException: pass