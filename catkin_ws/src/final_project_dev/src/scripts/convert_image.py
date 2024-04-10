#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from move_robot import MoveTurtlebot3
from darknet_ros_msgs.msg import BoundingBoxes



class ColorCorrector(object):

    def __init__(self):
        self.bridge_object = CvBridge()
        # self.object_detection_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.object_detection_callback)
        self.image_sub = rospy.Subscriber("/camera/image_raw",Image,self.camera_callback)
        self.image_pub = rospy.Publisher("image_rgb",Image, queue_size=2)
        #self.object_detection_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBox, self.object_detection_callback)
        # self.moveTurtlebot3_object = MoveTurtlebot3()
        # self.objects = BoundingBoxes()
        # self.twist = Twist()
        self.image = Image()
        self.rate = rospy.Rate(5)
        # self.twist.linear.x = 0.0
        # self.twist.linear.y = 0.0
        # self.twist.linear.z = 0.0
        # self.twist.angular.x = 0.0
        # self.twist.angular.y = 0.0
        # self.twist.angular.z = 0.0
        # self.cx_prev = 0
        # self.cx = 0
        # self.cx_int = 0
        # self.cx_upper = 0
        # self.stop_sign_detected = False

    # def object_detection_callback(self, data):
    #     self.objects = data
    #     for i in range(len(self.objects.bounding_boxes)):
    #         if (self.objects.bounding_boxes[i].Class == "stop sign" and self.objects.bounding_boxes[i].probability > 0.5):
    #             print(self.objects.bounding_boxes[i].probability)
    #             print("stop sign detected")
    #             self.stop_sign_detected = True


    def camera_callback(self, data):
        # We select bgr8 because its the OpneCV encoding by default
        print("in camera callback")
        cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        # # We get image dimensions and crop the parts of the image we dont need
        # height, width, channels = cv_image.shape
        # # crop_img = cv_image[int((height/2)+100):int((height/2)+120)][1:int(width)]
        # crop_img = cv_image[int((height/2))+80:-1,:]
        # # crop_img_upper = cv_image[int((height/2))+10:-60,:]
        # crop_img_upper = cv_image[int((height/2)):-60,:]

        # HSV = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        # hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        # hsv_upper = cv2.cvtColor(crop_img_upper, cv2.COLOR_BGR2HSV)

        # # Define the Yellow Colour in HSV

        # """
        # To know which color to track in HSV use ColorZilla to get the color registered by the camera in BGR and convert to HSV. 
        # """

        # # Threshold the HSV image to get only yellow colors
        # # lower_yellow = np.array([20,100,100])
        # # upper_yellow = np.array([50,255,255])

        # l_blue = np.array([100,100,20])
        # u_blue = np.array([140,255,255])
        # mask = cv2.inRange(hsv, l_blue, u_blue)
        # mask_upper = cv2.inRange(hsv_upper, l_blue, u_blue)
        # whole_mask = cv2.inRange(HSV, l_blue, u_blue)

        # # Calculate centroid of the blob of binary image using ImageMoments
        # m = cv2.moments(mask, False)
        # m_upper = cv2.moments(mask_upper, False)

        # # Convert from RGB to HSV
        # hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        self.image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        self.image_pub.publish(self.bridge_object.cv2_to_imgmsg(self.image, encoding="rgb8"))
        rospy.loginfo("PUBLISHED RGB IMAGE")
        
        # cv2.imshow("Original", cv_image)
        # cv2.imshow("Whole Mask", whole_mask)
        # cv2.imshow("HSV", HSV)
        # cv2.imshow("MASK", mask)
        # cv2.imshow("MASK UPPER", mask_upper)
        # cv2.imshow("hsv", hsv)
        # cv2.imshow("hsv upper", hsv_upper)
        # cv2.waitKey(1)

        self.rate.sleep()


    def clean_up(self):
        # self.moveTurtlebot3_object.clean_class()
        cv2.destroyAllWindows()

def main():
    # print("in main")
    rospy.init_node('color_correcting_node', anonymous=True)
    color_corrector_object = ColorCorrector()
    rate = rospy.Rate(5)
    ctrl_c = False
    def shutdownhook():
        # Works better than rospy.is_shutdown()
        color_corrector_object.clean_up()
        rospy.loginfo("Shutdown time!")
        ctrl_c = True
    rospy.on_shutdown(shutdownhook)
    while not ctrl_c:
        rate.sleep()

if __name__ == '__main__':
        main()
        