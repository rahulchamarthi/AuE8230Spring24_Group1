#!/usr/bin/env python3

import rospy
import numpy as np
import time
from math import inf
from numpy import inf
from geometry_msgs.msg  import Twist, Vector3
from sensor_msgs.msg import LaserScan



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
    
class TB3_controller:
      def __init__(self):
            rospy.init_node('wallfollowing_control', anonymous=True)
            self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
            self.scan_subscriber    = rospy.Subscriber('/scan', LaserScan, self.scan)
            self.laserscan=[]
            self.vel_msg=Twist()
            self.rate               = rospy.Rate(10)

      def scan(self,data):
           self.laserscan=list(data.ranges)
           #print(self.laserscan)

           for i in range(360):
              if self.laserscan[i] ==inf:
                self.laserscan[i]=7
              if self.laserscan[i] ==0:
                self.laserscan[i]=6
                

      def wallfollowing(self):
            print("hello booo")
            self.rate.sleep()
            pid=PID(1,0,1)
            pid_oa=PID(20,0,5)
            while not rospy.is_shutdown():
                  #print("hello")

                  lw_d=np.mean(self.laserscan[30:90])
                  rw_d=np.mean(self.laserscan[270:330])
                  fw_sec_one = np.mean(self.laserscan[0:60])
                  fw_sec_two = np.mean(self.laserscan[300:360])
                  error=lw_d-rw_d
                  fw_d=np.mean(self.laserscan[0:5])+np.mean(self.laserscan[360-5:360])/2 
                  t=time.time()
                  print(len(self.laserscan))
                  if self.laserscan[90]>3 and self.laserscan[270]>3 and fw_d>3:
                       linear_vel   = np.clip((fw_d-0.2),-0.1,0.2)
                       angular_zvel = 0
                       print('no wall')
                  
                  elif fw_d < 0.5:
                       
                       linear_vel   = 0.007#np.clip((fw_d-0.2),-0.1,0.2)
                       angular_zvel = np.clip(pid_oa.output((fw_sec_two-fw_sec_one),t),-1.2,1.2)
                       print("OA-f")     
                
                  elif fw_sec_one < 1 and fw_sec_two <1 and fw_d < 1:
                       print("OAf1")
                       linear_vel   = 0.007#np.clip((fw_d-0.2),-0.1,0.2)
                       angular_zvel = np.clip(pid.output(error,t),-0.8,0.8)#angular_zvel = np.clip(pid_oa.output((fw_sec_two-fw_sec_one),t),-1.2,1.2)
                  else:
                       linear_vel   = np.clip((fw_d-0.2),-0.1,0.2)
                       angular_zvel = np.clip(pid.output(error,t),-0.8,0.8)
                       print("WF")


                #   linear_vel   = np.clip((fw_d-0.2),-0.1,0.2)
                #   angular_zvel = np.clip(pid.output(error,t),-0.8,0.8)
                  self.vel_msg.linear.x=linear_vel
                  self.vel_msg.angular.z=angular_zvel
                  self.velocity_publisher.publish(self.vel_msg)
                  self.rate.sleep()
                  print('distance from right =',fw_sec_one)
                  print('distance from left  =',fw_sec_two)
                  print('distance from front wall  =',fw_d)

                  print('linear_vel=',linear_vel,' angular_vel=',angular_zvel)
                  rospy.loginfo('\n') 

                  
            
            
      
    

if __name__ == '__main__':
        
        t=TB3_controller()
        
      
        try:
            #print('hb')
            t.wallfollowing()
                
        except rospy.ROSInterruptException: pass
        
