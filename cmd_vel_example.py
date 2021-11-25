#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import time

# This script exhibits how to publish /cmd_vel messages. The robot will move straight and then pirouette.

# Define global variables
X = 0
PI = 3.14159265359

rospy.init_node('usv_velocity_controller')  # define a node name
# define a variable that contains the Twist message
cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
# define a variable as a Twist message
cmd_vel_msg = Twist()
rate = rospy.Rate(1)


def publish_vel(cmd_vel):
    global cmd_vel_pub
    global rate
    # Publish the Twist message to "/cmd_vel"
    cmd_vel_pub.publish(cmd_vel)
    rate.sleep()


def move_straight():
    # --------- GO STRAIGHT FOR TWO METERS ---------
    global cmd_vel_msg
    print("Moving robot forward.")
    linear_velocity_x = 0.5  # this is in meters per second
    angular_velocity_z = 0.0  # this is in radians per second
    # Filling the Twist message
    cmd_vel_msg.linear.x = linear_velocity_x
    cmd_vel_msg.angular.z = angular_velocity_z
    # Send the Twist message to the publishing function
    publish_vel(cmd_vel_msg)


def stop_movement():
    # ---------- STOP THE ROBOT ---------
    global cmd_vel_msg
    print("Stopping robot.")
    linear_velocity_x = 0.0  # this in in meters per second
    angular_velocity_z = 0.0  # this is in radians per second
    # Filling the Twist message
    cmd_vel_msg.linear.x = linear_velocity_x
    cmd_vel_msg.angular.z = angular_velocity_z
    # Send the Twist message to the publishing function
    publish_vel(cmd_vel_msg)


def turn_counterclockwise():
    # --------- ROTATE COUNTERCLOCKWISE 360 DEGREES ----------
    global cmd_vel_msg
    print("Turning robot counterclockwise")
    linear_velocity_x = 0.0  # this in in meters per second
    angular_velocity_z = 0.5  # this is in radians per second
    # Filling the Twist message
    cmd_vel_msg.linear.x = linear_velocity_x
    cmd_vel_msg.angular.z = angular_velocity_z
    # Publish the Twist message to "/cmd_vel"
    publish_vel(cmd_vel_msg)


def robot_controller():
    global PI
    time.sleep(1)

    # move the robot straight at a velocity of 1 meter/second
    move_straight()
    time.sleep(4)  # run the velocity command for two seconds. (0.5 m/sec)*(4 sec) = 2 meters

    # stop the robot
    stop_movement()
    time.sleep(1)

    # rotate the robot counter clockwise at a rate of 1 radian/second
    turn_counterclockwise()
    time.sleep(4 * PI)  # run the velocity command for two seconds. (0.5 rad/sec)*(4PI sec) = 2 rad = 1 full rotation.

    # stop the robot
    stop_movement()
    time.sleep(1)


# This is the main function; the program starts here.
if __name__ == '__main__':
    try:
        robot_controller()
    except KeyboardInterrupt:
        print("Shutting Down")
