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
        self.front_min_dist = 0.3
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

        self.bot_x = 0
        self.bot_y = 0
        self.bot_z = 0
        self.start_time = 0

    def scan_callback(self, data):
        self.scan  = data


    def state_callback(self, data):
        self.state = data

    def odometryCb(self, msg):
	# global bot_x, bot_y, bot_z 

        self.bot_x = msg.pose.pose.position.x 
        self.bot_y = msg.pose.pose.position.y 
        self.bot_z = msg.pose.pose.position.z 

    def free_space(self, sector_dist):
        free_space_mask=[]
        treshold_dist=min(sector_dist)+0.3
        for i in range(len(sector_dist)):
            if sector_dist[i] <= treshold_dist:
                free_space_mask.append(0)
            else:
                free_space_mask.append(sector_dist[i])

        return free_space_mask 

    def max_gap(self, free_space_mask):
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
                    
    def desired_angle(self,index):
        theta_d=10*(9-index)
        
        return theta_d         
	    

    def wallfollowing_controller(self):
        self.laserscan  = np.array(self.scan.ranges)

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
        pid=PID(1,0,1)
 

        switch_condition = False
        if (time.time()-self.start_time > 2.5):
            #self.start_time = time.time()

            # while not rospy.is_shutdown():
            delta = self.lw_d-self.rw_d   # distance error

            
            # pid=PID(1,0,1) is this expected behavior?

            t=time.time()
            P_output  = self.pid.output(delta,t)

            # self.fw_d=np.mean(self.laserscan[0:5])+np.mean(self.laserscan[360-5:360])/2 
            self.fw_d= min(float(self.fw_sec_one), float(self.fw_sec_two))

            sector_dist=np.empty(18)
            n=0 
            for sector_num in range(9):
                sector_dist[sector_num]=np.mean(self.laserscan[80-n:90-n])
                n=n+10
            n=0
            for sector_num in range(9):
                sector_dist[9+sector_num]=np.mean(self.laserscan[350-n:360-n])
                n=n+10
            free_space_mask=self.free_space(sector_dist)
            max_gap_center=self.max_gap(free_space_mask)
            theta_d=self.desired_angle(max_gap_center)

            if (self.rw_dist_actual >= self.side_min_dist and 
                    self.lw_dist_actual >= self.side_min_dist and 
                    self.fw_sec_one >= self.front_min_dist and 
                    self.fw_sec_two >= self.front_min_dist):
                linear_vel = np.clip((self.fw_d-0.15),-0.1,0.15)
                # print("#######################################")
                # print("delta", delta)
                # print("t",t)
                # print("pid",pid.output(delta,t))
                print("self_pid", P_output)
                angular_zvel = np.clip(P_output,-0.8,0.8)
                rospy.loginfo('WALL FOLLOWING')

            else: 
                linear_vel   = np.clip(0.1,-0.1,0.1)
                angular_zvel = np.clip(0.03*theta_d,-0.8,0.8)
                rospy.loginfo('OBSTACLE AVOIDANCE')

            # print('min distance right sector / right wall  =', self.rw_dist_actual)
            # print('min distance left sector / left wall  =', self.lw_dist_actual)
            # print('min distance front sector #1 =', self.fw_sec_one)
            # print('min distance front sector #2 =', self.fw_sec_two)
            # print('linear_vel=',linear_vel,' angular_vel=',angular_zvel)
            # print('odom x position =', self.bot_x)
            # print('odom y position =', self.bot_y)
            # print('odom z position =', self.bot_z)
            # print('OBSTACLE COUNTER COUNT =', counter)
            # print('start time', self.start_time) 
            # print('current_time', time.time())
            rospy.loginfo('\n') 

        # # #publish cmd_vel
        # #vel_msg = Twist(Vector3(linear_vel,0,0), Vector3(0,0,angular_zvel))
        # vel_msg = Twist(Vector3(0,0,0), Vector3(0,0,0))
        # velocity_publisher.publish(vel_msg)
        # rate.sleep()
        

        # if (self.rw_dist_actual >= self.side_min_dist and 
        #         self.lw_dist_actual >= self.side_min_dist and 
        #         self.fw_sec_one >= self.front_min_dist and 
        #         self.fw_sec_two >= self.front_min_dist):
        # # 	switch_condition = True 
        #     rospy.loginfo('NORMAL BEHAVIOR')
        #     angular_zvel = np.clip(P_output,-1.2,1.2)
        #     linear_vel   = np.clip((self.fw_d-0.2),-0.1,0.4)

        # else: 

        #     #select the max of the mins and proceed forward 
        #     rospy.loginfo('OBSTACLE DETECTED')
        #     all_distances = [int(self.rw_dist_actual), int(self.lw_dist_actual), int(self.fw_sec_one), int(self.fw_sec_two)]
        #     max_val_index = all_distances.index(max(all_distances))
        
        #     #TODO: if max_index = 0 (RIGHT)
        #     if max_val_index == 0 or max_val_index == 3: 
        #         linear_vel = np.clip((self.fw_d-0.2),-0.1,0.4)
        #         angular_zvel = -1 

        #     #TODO: elif max_index = 1 (LEFT)
        #     elif max_val_index == 1 or max_val_index == 2: 
        #         linear_vel = np.clip((self.fw_d-0.2),-0.1,0.4)
        #         angular_zvel = 1

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