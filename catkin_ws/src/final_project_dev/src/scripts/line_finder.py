#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist


class LineFollower(object):
    def __init__(self):
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_raw",Image,self.camera_callback)
        self.state_sub = rospy.Subscriber("/turtle_state", Int16, self.state_callback)
        self.vel_pub = rospy.Publisher("/line_cmd_vel", Twist, queue_size=10)
        self.cv_image = None
        self.state = Int16
        self.twist = Twist()
        self.twist.linear.x = 0
        self.twist.linear.y = 0
        self.twist.linear.z = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = 0
        # self.state.data = 0
        self.cx_prev = 0
        self.cx = 0
        self.cx_int = 0
        self.cx_upper = 0

    def state_callback(self, data):
        self.state = data    

    def camera_callback(self, data):
        # We select bgr8 because its the opneCV encoding by default
        # print("in line_follower_callback")
        self.cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        #cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="rgb8")    
          

    def line_finder(self):
        def nothing(x):
            pass

        cv2.namedWindow("Trackbars")
        cv2.createTrackbar("L-H", "Trackbars", 0, 179, nothing)
        cv2.createTrackbar("L-S", "Trackbars", 0, 255, nothing)
        cv2.createTrackbar("L-V", "Trackbars", 0, 255, nothing)
        cv2.createTrackbar("U-H", "Trackbars", 179, 179, nothing)
        cv2.createTrackbar("U-S", "Trackbars", 255, 255, nothing)
        cv2.createTrackbar("U-V", "Trackbars", 255, 255, nothing)

        while True:
            hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

            l_h = cv2.getTrackbarPos("L-H", "Trackbars")
            l_s = cv2.getTrackbarPos("L-S", "Trackbars")
            l_v = cv2.getTrackbarPos("L-V", "Trackbars")
            u_h = cv2.getTrackbarPos("U-H", "Trackbars")
            u_s = cv2.getTrackbarPos("U-S", "Trackbars")
            u_v = cv2.getTrackbarPos("U-V", "Trackbars")

            l_range = np.array([l_h, l_s, l_v])
            u_range = np.array([u_h, u_s, u_v])

            mask = cv2.inRange(hsv, l_range, u_range)

            res = cv2.bitwise_and(self.cv_image, self.cv_image, mask=mask)

            mask_3 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

            stacked = np.hstack((mask_3, self.cv_image, res))

            cv2.imshow('Trackbars', stacked)

            key = cv2.waitKey(1)
            if key == 27:
                break

            if key == ord('s'):


                thearray = [[l_h, l_s, l_v], [u_h, u_s, u_v]]

                print(thearray)

                np.save('hsv_value', thearray)

                break





       

    def clean_up(self):
        # self.moveTurtlebot3_object.clean_class()
        cv2.destroyAllWindows()

def main():
    rospy.init_node('line_following_node', anonymous=True)

    lineFollower_object = LineFollower()
    rate = rospy.Rate(10)

    ctrl_c = False
    def shutdownhook():
        lineFollower_object.clean_up()
        rospy.loginfo("Shutting Down Line Follower")
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)

    while not ctrl_c:
        if lineFollower_object.state.data == 2 and lineFollower_object.cv_image is not None:
            lineFollower_object.line_finder()
        rate.sleep()

if __name__ == '__main__':
    main()                

    # def set_velocities_to_zero(self):
    #     self.twist.linear.x = 0
    #     self.twist.angular.z = 0

    # def main(self):
    #     if self.state.data == 0:
    #         self.set_velocities_to_zero()
    #     elif self.state.data == 1:
    #         self.wall_following()
    #     elif self.state.data == 2:
    #         self.obstacle_avoidance()
    #     elif self.state.data == 3:
    #         self.line_follower()
    #     else:
    #         print("no state data published")
    #         self.set_velocities_to_zero()

    #     self.moveTurtlebot3_object.move_robot(self.twist)