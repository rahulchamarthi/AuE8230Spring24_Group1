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
        self.cv_image = Image()
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
        self.cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        #cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="rgb8")    
          

    def line_follower(self):

        # We get image dimensions and crop the parts of the image we dont need
        height, width, channels = self.cv_image.shape
        # crop_img = cv_image[int((height/2)+100):int((height/2)+120)][1:int(width)]
        crop_img = self.cv_image[int((height/2))+80:-1,:]
        # crop_img_upper = cv_image[int((height/2))+10:-60,:]
        crop_img_upper = self.cv_image[int((height/2)):-60,:]

        # Convert from RGB to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        hsv_upper = cv2.cvtColor(crop_img_upper, cv2.COLOR_BGR2HSV)

        # hsv = cv2.cvtColor(crop_img, cv2.COLOR_RGB2HSV)
        # hsv_upper = cv2.cvtColor(crop_img_upper, cv2.COLOR_RGB2HSV)

        # Define the Yellow Colour in HSV

        """
        To know which color to track in HSV use ColorZilla to get the color registered by the camera in BGR and convert to HSV. 
        """

        # Threshold the HSV image to get only yellow colors
        # lower_yellow = np.array([20,100,100])
        # upper_yellow = np.array([50,255,255])

        l_blue = np.array([100,100,20])
        u_blue = np.array([140,255,255])
        mask = cv2.inRange(hsv, l_blue, u_blue)
        mask_upper = cv2.inRange(hsv_upper, l_blue, u_blue)

        # Calculate centroid of the blob of binary image using ImageMoments
        m = cv2.moments(mask, False)
        m_upper = cv2.moments(mask_upper, False)


        try:
            self.cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            self.cx, cy = height/2, width/2

        try:
            self.cx_upper, cy_upper = m_upper['m10']/m_upper['m00'], m_upper['m01']/m_upper['m00']
        except ZeroDivisionError:
            self.cx_upper, cy_upper = height/2, width/2    


        # # Draw the centroid in the resultut image
        # # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]]) 
        # cv2.circle(mask,(int(self.cx), int(cy)), 10,(0,0,255),-1)
        # # cv2.circle(mask_upper, (int(cx2), int(cy2)), 10, (0,0,255), -1)
        # cv2.imshow("Original", cv_image)
        # cv2.imshow("MASK", mask)
        # cv2.imshow("MASK UPPER", mask_upper)
        # cv2.imshow("hsv", hsv)
        # cv2.imshow("hsv upper", hsv_upper)
        # cv2.waitKey(1)

        #################################
        ###   ENTER CONTROLLER HERE   ###
        #################################
        # self.twist.angular.z = -0.1*(cx2-cx)/np.abs(cx2-cx)
        # this is the basic line following implementation it is a simple proportional controller keeping the blob centroid in the middle of the screen. We will work on a more robust solution for the final project
        cx_error = (self.cx-width/2)
        error_derivative = cx_error-self.cx_prev

        # print(-0.003*cx_error, - 0.001*(error_derivative), - 0.0001*self.cx_int)

        # print(np.sum(mask))
        # print(self.objects)
        if (np.sum(mask)==0 and np.sum(mask_upper)==0 and self.stop_sign_detected!=True):
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.15
            self.cx_int = 0
            self.cx_prev = 0
        elif (np.sum(mask)==0 and self.stop_sign_detected!=True):
            self.twist.linear.x = 0.04
            self.twist.angular.z = -0.001*(self.cx_upper-width/2)
        elif (self.stop_sign_detected == True):
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
        else:
            self.twist.linear.x = 0.06
            self.twist.angular.z = -0.003*cx_error - 0.002*(error_derivative) - 0.0001*self.cx_int
            self.cx_prev = cx_error
            self.cx_int = self.cx_int + cx_error

        # self.twist.angular.z = -0.001*(cx-width/2)
        # rospy.loginfo("ANGULAR VALUE SENT===>"+str(self.twist.angular.z))
        # Make it start turning
        # self.moveTurtlebot3_object.move_robot(self.twist)
        self.vel_pub.publish(self.twist)

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
        if lineFollower_object.state.data == 2:
            lineFollower_object.line_follower()
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