#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan

class Lidar(object):

    def __init__(self):
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=1)
        # rospy.init_node('laser_scan_node', anonymous=False)
        # rospy.init_node('move', anonymous=True)

        self.ranges = None
        self.angles = None
        self.increment = None


    def scan_callback(self, msg):
        """ this function helps us find the minimum value on the left and on the right side of the laser scanner """

        # Convert all range measurement values to array
        self.ranges = np.array(msg.ranges)
        self.ranges[self.ranges > 10] = 10
        num_ranges  = np.arange(len(self.ranges))

        # Array of radian angle measures corresponding to each ray
        self.angles = msg.angle_min + num_ranges * msg.angle_increment

        # Store the angle incrememnt of the Lidar
        self.increment = msg.angle_increment

        # Slice values into right and left measurements based on the angle measure
        left  = self.ranges[(self.angles<=np.pi/2)]
        right = self.ranges[(self.angles>np.pi/2) & (self.angles<=np.pi)]

        # minimum values corresponding to the closest proximity to the robot
        # maximum value set to 10
        min_left  = min(left) if min(left) < 10 else 10
        min_right = min(right) if min(right) < 10 else 10

        # print(' l ', min_left, ' r ', min_right)
        # print(min_right)

    def get_ranges(self):
        return self.ranges


    def get_frontal_dist(self):

        min_left, min_right = self.get_frontal_side_dist()

        # range_test = self.ranges[0] if self.ranges[0] < 10 else 10
        return min([min_left, min_right])

    def get_frontal_side_dist(self):
        right = self.ranges[(self.angles<=1/4*np.pi)]
        left = self.ranges[(self.angles>7/4*np.pi) & (self.angles<2*np.pi)]
        # print(self.ranges)
        # front = left
        min_left = min(left) if min(left) < 10 else 10
        min_right = min(right) if min(right) < 10 else 10

        return min_left, min_right


    def get_closest(self):

        # Find the closest range and the angle measure associated with it
        # relative to the robot frame
        closest_range = min(self.ranges)

        angle = np.argmin(self.angles)*self.increment

        return closest_range, angle


    def my_first_controller(self):
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('laser_scan_node', anonymous=False)

        rospy.Subscriber("scan", LaserScan, scan_callback, queue_size=1)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


    # this is the main function: the program starts here
if __name__ == '__main__':

    my_first_controller()
