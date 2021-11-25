#!/usr/bin/env python3


""" # PLANNING

_Classes_
- Movement
-- Linear velocity
-- Angular velocity

- Vision
-- Color detection
-- Bounding boxes

- Sensors
-- Lidar

- Position
-- Robot coordinates
-- Pillar coordinates
-- Transformations
-- Distance calculations

- Logic
-- Path finding
-- Trajectory planning

_Ideas_

- Robot initalizes in random location
-- May have obstacles/walls

- Spin full rotation to attempt to identify any and all pillars
- Log camera frame locations and sensor distance (if able)
-- Use the centroid of the pillar as the coordinate origin?
- DO WE KNOW THE HEIGHTS OF THE PILLARS?
- Calculate distance to all of the pillars
- Move to them in order of closest distance

- If there are obstacles/walls, move around the room and SLAM
- Continue looking for pillars

- Need some way of keeping track of pillar, location, distance from it, and pillars found/remaining

- Movement correction/steering


"""









import rospy
import numpy as np
from geometry_msgs.msg import Twist
import time
import sys

# This script exhibits how to publish /vel_control messages. The robot will move straight and then pirouette.

class vel_control:

    def __init__(self):

        # create a publisher object to send the twist message
        # Twist is the common_msg type that includes linear and angular velocity parameters
        self.pub = rospy.Publisher("vel_control", Twist, queue_size=10)

        # Create a node instance for the velocity controller with name 'move'
        rospy.init_node('move', anonymous=True)

        # create a message object containing the Twist message
        self.vel_control_msg = Twist()

        # Set sleep rate in Hz
        self.rate = rospy.Rate(1)

    def publish_vel_control_control(self, vel_control_msg):
        # Publish the message to "vel_control"
        self.pub.publish(vel_control_msg)
        self.rate.sleep()


    def move_forward(self):
        linear_velocity_x = 0.5  # this is in meters per second
        # angular_velocity_z = 0.0  # this is in radians per second
        # Filling the Twist message
        self.vel_control_msg.linear.x = linear_velocity_x
        # self.vel_control_msg.angular.z = angular_velocity_z
        # Send the Twist message to the publishing function
        self.publish_vel_control(self.vel_control_msg)


    def stop(self):
        # ---------- STOP THE ROBOT ---------
        print("Stopping robot.")
        linear_velocity_x = 0.0  # this in in meters per second
        angular_velocity_z = 0.0  # this is in radians per second
        # Filling the Twist message
        self.vel_control_msg.linear.x = linear_velocity_x
        self.vel_control_msg.angular.z = angular_velocity_z
        # Send the Twist message to the publishing function
        self.publish_vel_control(self.vel_control_msg)


    def move_left(self):
        # --------- ROTATE COUNTERCLOCKWISE 360 DEGREES ----------
        print("Turning robot counterclockwise")
        linear_velocity_x = 0.0  # this in in meters per second
        angular_velocity_z = 0.5  # this is in radians per second
        # Filling the Twist message
        # self.vel_control_msg.linear.x = linear_velocity_x
        self.vel_control_msg.angular.z = angular_velocity_z
        # Publish the Twist message to "/vel_control"
        self.publish_vel_control(self.vel_control_msg)

    def move_right(self):
        # --------- ROTATE COUNTERCLOCKWISE 360 DEGREES ----------
        print("Turning robot counterclockwise")
        # linear_velocity_x = 0.0  # this in in meters per second
        angular_velocity_z = -0.5  # this is in radians per second
        # Filling the Twist message
        # self.vel_control_msg.linear.x = linear_velocity_x
        self.vel_control_msg.angular.z = angular_velocity_z
        # Publish the Twist message to "/vel_control"
        self.publish_vel_control(self.vel_control_msg)

    def move(self, new_linear_vel, new_angluar_vel):
        self.vel_control_msg.linear.x = new_linear_vel
        self.vel_control_msg.angular.z = new_angular_vel
        self.publish_vel_control(self.vel_control_msg)


    # def robot_controller(self):
    #     time.sleep(1)
    #
    #     # move the robot straight at a velocity of 1 meter/second
    #     self.move_straight()
    #     time.sleep(4)  # run the velocity command for two seconds. (0.5 m/sec)*(4 sec) = 2 meters
    #
    #     # stop the robot
    #     self.stop_movement()
    #     time.sleep(1)
    #
    #     # rotate the robot counter clockwise at a rate of 1 radian/second
    #     self.turn_left()
    #     time.sleep(4 * np.pi)  # run the velocity command for two seconds. (0.5 rad/sec)*(4PI sec) = 2 rad = 1 full rotation.
    #
    #     # stop the robot
    #     self.stop_movement()
    #     time.sleep(1)


    # This is the main function; the program starts here.
if __name__ == '__main__':
    try:
        # controller = vel_control()
        # controller.robot_controller()
        pass

    except KeyboardInterrupt:
        print("Shutting Down")
