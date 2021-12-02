#!/usr/bin/env python3


""" # PLANNING

_Classes_
- Position
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
from nav_msgs.msg import Odometry
import time
import sys

# This script exhibits how to publish /vel_control messages. The robot will move straight and then pirouette.

class vel_control:

    def __init__(self):

        # Position and orientation variables
        self.x = 0
        self.y = 0
        self.z = 0
        self.orientation = None

        # Create a node instance for the velocity controller with name 'move'
        rospy.init_node('move', anonymous=True)

        # Set sleep rate in Hz
        # self.rate = rospy.Rate(10)

        # create a publisher object to send the twist message
        # Twist is the common_msg type that includes linear and angular velocity parameters
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.pos_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # create a message objects
        self.vel_control_msg = Twist()
        self.odom_msg        = Odometry()

    def get_pos_orientation(self):
        return (self.x, self.y, self.z), self.orientation

        
    def odom_callback(self, msg):

        # Update the robots current position and orientation
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z
        self.orientation = msg.pose.pose.orientation


        # print(self.orientation)
        # self.rate.sleep()

    def publish_vel_control(self, vel_control_msg):
        # Publish the message to "vel_control"
        self.vel_pub.publish(vel_control_msg)
        # self.rate.sleep()


    def move_forward(self):
        linear_velocity_x = 0.5  # this is in meters per second
        # Filling the Twist message
        self.vel_control_msg.linear.x = linear_velocity_x
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
        # --------- ROTATE CLOCKWISE 360 DEGREES ----------
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
        self.vel_control_msg.angular.z = new_angluar_vel
        self.publish_vel_control(self.vel_control_msg)

    def seek(self, dir='r', echo=False):
        """Rotate one full turn.
            :param dir: direction to rotate.
            .. r -> right
            .. l -> left
            :param echo: if True, report current position and orientation."""

        # print('Seeking...')

        linear_velocity_x  = 0
        angular_velocity_z = -0.5 if dir == 'r' else 0.5

        self.vel_control_msg.linear.x = linear_velocity_x
        self.vel_control_msg.angular.z = angular_velocity_z
        self.publish_vel_control(self.vel_control_msg)

        pos = (self.x, self.y, self.z)

        if echo: return (pos, self.orientation)

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
        controller = vel_control()
        controller.move(0, 0.5)
        rospy.spin()
        # controller.robot_controller()

    except KeyboardInterrupt:
        print("Shutting Down")
