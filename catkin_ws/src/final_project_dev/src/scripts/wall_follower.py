#!/usr/bin/env python3
import rospy
import numpy as np
import time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from pid import PID

class WallFollower(object):
    def __init__(self):
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.state_sub = rospy.Subscriber("/turtle_state", Int16, self.state_callback)
        self.vel_pub = rospy.Publisher("/wall_cmd_vel", Twist, queue_size=10)
        self.state = Int16()
        self.twist = Twist()
        self.scan = LaserScan()
        self.twist.linear.x = 0
        self.twist.linear.y = 0
        self.twist.linear.z = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = 0
        # self.state.data = 0
        self.kp = 4
        # self.s_d = 
        self.laserscan = np.zeros(360)
        # self.y = 
        # self.y_r = 
        # self.y_l =
        self.side_min_dist = 0.15
        self.front_min_dist = 0.2
        self.distance_from_wall = 0.7
        self.pid = PID(1,0,1)
        self.pid_oa=PID(20,0,5)
        self.lw_d = 0
        self.rw_d = 0
        self.fw_d = 0
        self.rw_dist_actual = 0
        self.lw_dist_actual = 0
        self.fw_sec_one = 0
        self.fw_sec_two = 0
        self.front_scanrange = 16
        self.start_angle = 20
        self.side_scanrange = 60

    def scan_callback(self, data):
        self.scan  = data


    def state_callback(self, data):
        self.state = data    
	    

    def wallfollowing_controller(self):
        self.laserscan  = np.array(self.scan.ranges)
        # global kp,s_d,x,y_r,y_l
        # global distancefromwall
        
        # buffer variables 
        # side_min_dist = 0.15
        # front_min_dist = 0.2

        for i in range(360):
            if self.laserscan[i] == np.inf:
                self.laserscan[i] = 7
            if self.laserscan[i] == 0:
                self.laserscan[i] = 6

            # store scan data 
        #TODO: ADD PARAMETERIZATION BACK INTO THE STUFF DOWN BELOW
        self.lw_d= np.mean(self.laserscan[30:30+55])          # left wall distance 
        self.rw_d= np.mean(self.laserscan[330-55:330])  # right wall distance
        
        self.lw_dist_actual = min(self.laserscan[30:85])
        self.rw_dist_actual = min(self.laserscan[275:330])
        self.fw_sec_one = min(self.laserscan[0:30])
        self.fw_sec_two = min(self.laserscan[330:360])
        # self.lw_d=np.mean(self.laserscan[30:90])
        # self.rw_d=np.mean(self.laserscan[270:330])
        # self.fw_sec_one = np.mean(self.laserscan[0:60])
        # self.fw_sec_two = np.mean(self.laserscan[300:360])

        switch_condition = False 

        # while not rospy.is_shutdown():
        delta = self.lw_d-self.rw_d   # distance error

        
        # pid=PID(1,0,1) is this expected behavior?

        t=time.time()
        P_output  = self.pid.output(delta,t)

        # self.fw_d=np.mean(self.laserscan[0:5])+np.mean(self.laserscan[360-5:360])/2 
        self.fw_d= min(float(self.fw_sec_one), float(self.fw_sec_two))
        
        # print(len(self.laserscan))
        # if self.laserscan[90]>3 and self.laserscan[270]>3 and self.fw_d>3:
        #     linear_vel   = np.clip((self.fw_d-0.2),-0.1,0.2)
        #     angular_zvel = 0
        #     print('no wall')
        
        # elif self.fw_d < 0.5:
            
        #     linear_vel   = 0.007#np.clip((fw_d-0.2),-0.1,0.2)
        #     angular_zvel = np.clip(self.pid_oa.output((self.fw_sec_two-self.fw_sec_one),t),-1.2,1.2)
        #     print("OA-f")     
    
        # elif self.fw_sec_one < 1 and self.fw_sec_two <1 and self.fw_d < 1:
        #     print("OAf1")
        #     linear_vel   = 0.007#np.clip((fw_d-0.2),-0.1,0.2)
        #     angular_zvel = np.clip(self.pid.output(error,t),-0.8,0.8)#angular_zvel = np.clip(pid_oa.output((fw_sec_two-fw_sec_one),t),-1.2,1.2)
        # else:
        #     linear_vel   = np.clip((self.fw_d-0.2),-0.1,0.2)
        #     angular_zvel = np.clip(self.pid.output(error,t),-0.8,0.8)
        #     print("WF")

        if (self.rw_dist_actual >= self.side_min_dist and 
                self.lw_dist_actual >= self.side_min_dist and 
                self.fw_sec_one >= self.front_min_dist and 
                self.fw_sec_two >= self.front_min_dist):
        # 	switch_condition = True 

        # if rw_dist_actual >= side_min_dist and lw_dist_actual >= side_min_dist and fw_sec_one >= front_min_dist and fw_sec_two >= front_min_dist:
        # if switch_condition is False:
            #wall following:  
            # rospy.loginfo('WALL FOLLOWING MODE')
            rospy.loginfo('NORMAL BEHAVIOR')
            angular_zvel = np.clip(P_output,-1.2,1.2)
            linear_vel   = np.clip((self.fw_d-0.2),-0.1,0.4)

        else: 
            #obstacle avoidance 
            # rospy.loginfo('OBSTACLE AVOIDANCE MODE')
            # if rw_dist_actual >= side_min_dist and lw_dist_actual >= side_min_dist and fw_sec_one >= front_min_dist and fw_sec_two >= front_min_dist:
            # 	linear_vel = 0.2 
            # 	angular_vel = 0 
            # else: 
            #select the max of the mins and proceed forward 
            rospy.loginfo('OBSTACLE DETECTED')
            all_distances = [int(self.rw_dist_actual), int(self.lw_dist_actual), int(self.fw_sec_one), int(self.fw_sec_two)]
            max_val_index = all_distances.index(max(all_distances))
        
            #TODO: if max_index = 0 (RIGHT)
            if max_val_index == 0 or max_val_index == 3: 
                linear_vel = np.clip((self.fw_d-0.2),-0.1,0.4)
                angular_zvel = -1 

            #TODO: elif max_index = 1 (LEFT)
            elif max_val_index == 1 or max_val_index == 2: 
                linear_vel = np.clip((self.fw_d-0.2),-0.1,0.4)
                angular_zvel = 1

        print('min distance right sector / right wall  =', self.rw_dist_actual)
        print('min distance left sector / left wall  =', self.lw_dist_actual)
        print('min distance front sector #1 =', self.fw_sec_one)
        print('min distance front sector #2 =', self.fw_sec_two)
        print('linear_vel=',linear_vel,' angular_vel=',angular_zvel)
        rospy.loginfo('\n')
        self.twist.linear.x = linear_vel
        self.twist.angular.z = angular_zvel
        self.vel_pub.publish(self.twist)
        # return linear_vel, angular_zvel 
    
def main():
    rospy.init_node('wall_following_node', anonymous=False)

    wallFollower_object = WallFollower()

    rate = rospy.Rate(10)

    ctrl_c = False
    def shutdownhook():
        # wallFollower_object.clean_up()
        rospy.loginfo("Shutting Down Wall Follower")
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)

    while not ctrl_c:
        if wallFollower_object.state.data == 1:
            wallFollower_object.wallfollowing_controller()
        rate.sleep()

if __name__ == '__main__':
    main()    

        #     # #publish cmd_vel
        #     vel_msg = Twist(Vector3(linear_vel,0,0), Vector3(0,0,angular_zvel))
        #     velocity_publisher.publish(vel_msg)
        #     rate.sleep()

        # velocity_publisher.publish(Twist(Vector3(0,0,0), Vector3(0,0,0)))
        # print('Turtlebot stopped')