#!/usr/bin/env python3

import rospy
from darknet_ros_msgs.msg import BoundingBoxes

class ObjectDetector(object):

    def __init__(self):
        rospy.init_node('object_detector', anonymous=False)
        self.object_detection_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.object_detection_callback, queue_size=5)
        self.objects = None
        self.rate = rospy.Rate(5)


    def object_detection_callback(self, data):
        print(data)
        self.objects = data

    def main(self):
         while (self.objects == None):
            self.rate.sleep()
         while not rospy.is_shutdown():     
            print(self.objects)
            self.rate.sleep()



# def main():
#     rospy.init_node('object_detector_node', anonymous=False)
#     object_detector_object = ObjectDetector()
#     rate = rospy.Rate(5)
#     ctrl_c = False
#     def shutdownhook():
#         # Works better than rospy.is_shutdown()
#         rospy.loginfo("Shutdown time!")
#         ctrl_c = True
#     rospy.on_shutdown(shutdownhook)
#     while not ctrl_c:
#         object_detector_object.main()
#         rate.sleep()

if __name__ == '__main__':
    OD = ObjectDetector()
    OD.main()
