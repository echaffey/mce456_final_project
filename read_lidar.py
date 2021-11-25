#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan


def scan_callback(msg):
    # this function helps us find the minimum value on the left and on the right side of the laser scanner
    # to use a global variable in a function, you must explicitly
    # redefine with the global attribute
    # global cmd_pub
    samples = len(msg.ranges)
    # initialize the min_left and min_right to 10. This is because 10 is the sensor's maximum reading.
    min_left = 10.0
    min_right = 10.0
    for i in range(samples):
        angle = msg.angle_min + i * msg.angle_increment
        if -1.57 < angle < 0:  # right hand side of the sensor.
            if min_right > msg.ranges[i]:
                min_right = msg.ranges[i]
            # print(i,' r ',msg.ranges[i],' ',angle)
        if 0 < angle < 1.57:  # left hand side of the sensor
            if min_left > msg.ranges[i]:
                min_left = msg.ranges[i]
            # print(i,' l ',msg.ranges[i],' ',angle)

    print(' l ', min_left, ' r ', min_right)


def my_first_controller():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('laser_scan_node', anonymous=False)

    rospy.Subscriber("/scan", LaserScan, scan_callback, queue_size=1)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


# this is the main function: the program starts here
if __name__ == '__main__':
    print("hello")
    my_first_controller()
