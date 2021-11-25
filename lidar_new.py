#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan


def scan_callback(msg):
    """ this function helps us find the minimum value on the left and on the right side of the laser scanner """

    # Convert all range measurement values to array
    ranges     = np.array(msg.ranges)
    num_ranges = np.arange(len(ranges))

    # Array of angles corresponding to each range in radians
    angles = msg.angle_min + num_ranges * msg.angle_increment

    # Slice values into right and left measurements based on the angle measure
    left  = ranges[(angles<np.pi/2)]
    right = ranges[(angles>=np.pi/2) & (angles < np.pi)]

    # minimum values corresponding to the closest proximity to the robot
    # maximum value set to 10
    min_left  = min(left) if min(left) < 10 else 10
    min_right = min(right) if min(right) < 10 else 10

    print(' l ', min_left, ' r ', min_right)
    # print(min_right)


def my_first_controller():
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
