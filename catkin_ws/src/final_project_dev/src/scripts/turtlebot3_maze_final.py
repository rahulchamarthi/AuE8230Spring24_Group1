#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

from move_robot import MoveTurtlebot3
from darknet_ros_msgs.msg import BoundingBoxes

from line_follower import LineFollower
from wall_follower import WallFollower



class MazeRunner(object):

    def __init__(self):

        self.object_detection_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.object_detection_callback)
        self.state_sub = rospy.Subscriber("/turtle_state", Int16, self.state_callback)
        self.wall_vel_sub = rospy.Subscriber("/wall_cmd_vel", Twist, self.wall_cmd_callback)
        self.line_vel_sub = rospy.Subscriber("/line_cmd_vel", Twist, self.line_cmd_callback)
        self.moveTurtlebot3_object = MoveTurtlebot3()
        self.objects = BoundingBoxes()
        self.twist = Twist()
        self.wall_twist = Twist()
        self.line_twist = Twist()
        self.state = Int16()
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0
        self.stop_sign_detected = False

    
    def wall_cmd_callback(self, data):
        self.wall_twist = data

    def line_cmd_callback(self, data):
        self.line_twist = data    

    def state_callback(self, data):
        self.state = data

    def object_detection_callback(self, data):
        self.objects = data
        for i in range(len(self.objects.bounding_boxes)):
            if (self.objects.bounding_boxes[i].Class == "stop sign" and self.objects.bounding_boxes[i].probability > 0.5):
                print(self.objects.bounding_boxes[i].probability)
                print("stop sign detected")
                self.stop_sign_detected = True

    def clean_up(self):
        self.moveTurtlebot3_object.clean_class()
        # self.line_follower.clean_up()

    def set_velocities_to_zero(self):
        self.twist.linear.x = 0
        self.twist.angular.z = 0

    def main(self):
        if self.stop_sign_detected == True:
            print("stop sign detected")
            self.set_velocities_to_zero()
        elif self.state.data == 0:
            self.set_velocities_to_zero()
        elif self.state.data == 1:
            # self.wall_follower.wall_following()
            self.twist = self.wall_twist
        elif self.state.data == 2:
            self.twist = self.line_twist
        else:
            print("no state data published")
            self.set_velocities_to_zero()

        self.moveTurtlebot3_object.move_robot(self.twist)                   


def main():
    rospy.init_node('maze_runner_brain', anonymous=False)
    turdel = MazeRunner()
    rate = rospy.Rate(10)
    ctrl_c = False
    def shutdownhook():
        # Works better than rospy.is_shutdown()
        turdel.clean_up()
        rospy.loginfo("Shutdown time!")
        ctrl_c = True
    rospy.on_shutdown(shutdownhook)
    while not ctrl_c:
        turdel.main()
        rate.sleep()

if __name__ == '__main__':
        main()
