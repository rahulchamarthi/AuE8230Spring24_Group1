#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class ColorPicker(object):

    def __init__(self):
        self.bridge_object = CvBridge()
        # autorace simulation
        #self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        # using raspicam node
        #self.image_sub = rospy.Subscriber("/raspicam_node/image",Image,self.camera_callback)
        # usig autorace cam node
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.camera_callback)
        self.winname = "Color Picker"
        
        self.frame = cv2.namedWindow("Color Picker")
        self.H = 0
        self.S = 0
        self.V = 0
        self.H2 = 0
        self.S2 = 0
        self.V2 = 0

        cv2.createTrackbar('H',self.winname,0,255,self.nothing)
        cv2.createTrackbar('S',self.winname,0,255,self.nothing)
        cv2.createTrackbar('V',self.winname,0,255,self.nothing)
        cv2.createTrackbar('H2',self.winname,0,255,self.nothing)
        cv2.createTrackbar('S2',self.winname,0,255,self.nothing)
        cv2.createTrackbar('V2',self.winname,0,255,self.nothing)



    def camera_callback(self, data):
        # We select bgr8 because its the OpneCV encoding by default
        cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        

        # We get image dimensions and crop the parts of the image we dont need
        height, width, channels = cv_image.shape
        
        # crop_img = cv_image[int((height/2)+100):int((height/2)+120)][1:int(width)]
        crop_img = cv_image[int((height/2))+0:-1,:]
        # crop_img_upper = cv_image[int((height/2)+150):int((height/2)+170)][1:int(width)]
        #crop_img = cv_image[340:360][1:640]

        # Convert from RGB to HSV
        # hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        #hsv2 = cv2.cvtColor(crop_img_upper, cv2.COLOR_BGR2HSV)

        # Define the Yellow Colour in HSV

        """
        To know which color to track in HSV use ColorZilla to get the color registered by the camera in BGR and convert to HSV. 
        """

        # Threshold the HSV image to get only yellow colors
        l_blue = np.array([100,150,110])
        u_blue = np.array([100,200,110])
        mask = cv2.inRange(hsv, l_blue, u_blue)
        #mask2 = cv2.inRange(hsv2, lower_yellow, upper_yellow)

        # Calculate centroid of the blob of binary image using ImageMoments
        m = cv2.moments(mask, False)
        #m2 = cv2.moments(mask2, False)


        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cx, cy = height/2, width/2

        # try:
        #     cx2, cy2 = m2['m10']/m['m00'], m['m01']/m['m00']
        # except ZeroDivisionError:
        #     cx2, cy2 = height/4, width/2    

        # print(cx2)
        # print(cy2)    
        # print(cx)
        # print(cy)    
        # print(hsv)
        
        # # Draw the centroid in the resultut image
        # # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]]) 
        # cv2.circle(mask,(int(cx), int(cy)), 10,(0,0,255),-1)
        # #cv2.circle(mask2, (int(cx2), int(cy2)), 10, (0,0,255), -1)
        # cv2.imshow("Original", cv_image)
        # cv2.imshow("MASK", mask)
        # cv2.imshow("hsv", hsv)
        # cv2.waitKey(1)
        print("here")

        #while(cv2.waitKey(0) != 27):
        self.H = cv2.getTrackbarPos('H', 'Color Picker')
        self.S = cv2.getTrackbarPos('S', 'Color Picker')
        self.V = cv2.getTrackbarPos('V', 'Color Picker')
        self.H2 = cv2.getTrackbarPos('H2', 'Color Picker')
        self.S2 = cv2.getTrackbarPos('S2', 'Color Picker')
        self.V2 = cv2.getTrackbarPos('V2', 'Color Picker')

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_bound = np.array([self.H,self.S,self.V])
        upper_bound = np.array([self.H2,self.S2,self.V2])
        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        final = cv2.bitwise_and(cv_image, cv_image, mask)
        cv2.imshow(self.frame, final)

    def nothing(self, x):
        pass

    def clean_up(self):
        cv2.destroyAllWindows()

def main():
    rospy.init_node('color_picker', anonymous=True)
    color_picker_object = ColorPicker()
    rate = rospy.Rate(5)
    ctrl_c = False
    def shutdownhook():
        # Works better than rospy.is_shutdown()
        color_picker_object.clean_up()
        rospy.loginfo("Shutdown time!")
        ctrl_c = True
    rospy.on_shutdown(shutdownhook)
    while not ctrl_c:
        rate.sleep()

if __name__ == '__main__':
        main()
