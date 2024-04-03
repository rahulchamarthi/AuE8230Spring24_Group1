#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from move_robot import MoveTurtlebot3

class LineFollower(object):

    def __init__(self):
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_raw",Image,self.camera_callback)
        self.moveTurtlebot3_object = MoveTurtlebot3()
        self.twist = Twist()
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0
        self.cx_prev = 0
        self.cx = 0
        self.cx_int = 0
        self.cx_upper = 0

    def camera_callback(self, data):
        # We select bgr8 because its the OpneCV encoding by default
        cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")

        # We get image dimensions and crop the parts of the image we dont need
        height, width, channels = cv_image.shape
        # crop_img = cv_image[int((height/2)+100):int((height/2)+120)][1:int(width)]
        crop_img = cv_image[int((height/2))+80:-1,:]
        crop_img_upper = cv_image[int((height/2))+10:-60,:]

        # Convert from RGB to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        hsv_upper = cv2.cvtColor(crop_img_upper, cv2.COLOR_BGR2HSV)

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
   
        
        # Draw the centroid in the resultut image
        # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]]) 
        cv2.circle(mask,(int(self.cx), int(cy)), 10,(0,0,255),-1)
        # cv2.circle(mask_upper, (int(cx2), int(cy2)), 10, (0,0,255), -1)
        cv2.imshow("Original", cv_image)
        cv2.imshow("MASK", mask)
        cv2.imshow("MASK UPPER", mask_upper)
        cv2.imshow("hsv", hsv)
        cv2.imshow("hsv upper", hsv_upper)
        cv2.waitKey(1)

        #################################
        ###   ENTER CONTROLLER HERE   ###
        #################################
        # self.twist.angular.z = -0.1*(cx2-cx)/np.abs(cx2-cx)
        # this is the basic line following implementation it is a simple proportional controller keeping the blob centroid in the middle of the screen. We will work on a more robust solution for the final project
        cx_error = (self.cx-width/2)
        error_derivative = cx_error-self.cx_prev

        print(-0.003*cx_error, - 0.001*(error_derivative), - 0.0001*self.cx_int)

        # print(np.sum(mask))
        print(np.sum(mask_upper))
        if (np.sum(mask)==0 and np.sum(mask_upper)==0):
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.15
            self.cx_int = 0
            self.cx_prev = 0
        elif (np.sum(mask)==0):
            self.twist.linear.x = 0.05
            self.twist.angular.z = -0.001*(self.cx_upper-width/2)
        else:
            self.twist.linear.x = 0.05
            self.twist.angular.z = -0.003*cx_error - 0.002*(error_derivative) - 0.0001*self.cx_int
            self.cx_prev = cx_error
            self.cx_int = self.cx_int + cx_error
        
        # self.twist.angular.z = -0.001*(cx-width/2)
        rospy.loginfo("ANGULAR VALUE SENT===>"+str(self.twist.angular.z))
        # Make it start turning
        self.moveTurtlebot3_object.move_robot(self.twist)

    def clean_up(self):
        self.moveTurtlebot3_object.clean_class()
        cv2.destroyAllWindows()

def main():
    rospy.init_node('line_following_node', anonymous=True)
    line_follower_object = LineFollower()
    rate = rospy.Rate(5)
    ctrl_c = False
    def shutdownhook():
        # Works better than rospy.is_shutdown()
        line_follower_object.clean_up()
        rospy.loginfo("Shutdown time!")
        ctrl_c = True
    rospy.on_shutdown(shutdownhook)
    while not ctrl_c:
        rate.sleep()

if __name__ == '__main__':
        main()
