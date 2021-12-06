#!/usr/bin/env python3

import rospy
import numpy as np
from cmd_vision import Vision
from cmd_control import vel_control
from control import PI_controller
from lidar_new import Lidar
from tf.transformations import euler_from_quaternion
import geometry_msgs

import numpy as np


class Brain():

    def __init__(self):

        rospy.init_node('move', anonymous=True)
        # rospy.Rate(1)

        self.vision = Vision()
        self.robot  = vel_control()
        self.PID    = PI_controller()
        self.lidar  = Lidar()

        robot_speed = 1 #m/s

        # Set controller to try to maintain pillar within the center of the screen
        # and limit the output from -1.5 to 1.5 rad/sec
        self.PI_angular_vel = PI_controller(
                                    # Proportional gain
                                    Kp=(robot_speed/3)/self.vision.CAMERA_WIDTH,
                                    # Integral gain
                                    Ki=1e-5,
                                    # Try to maintain in center of the screen
                                    setpoint=0,
                                    # No faster than 1.5 rad/sec angular velocity
                                    output_constraints=(-1.5, 1.5))

        # Store orientation and position of pillars
        self.center_yaw = [None, None, None]
        self.pillar_center = [None, None, None]

        # Pillar colors
        self.colors = ['RED', 'GREEN', 'BLUE']
        self.found = [False, False, False]
        self.mute_lidar = False

        self.run()


    def search_2(self, color_index):

        if(self.lidar.get_frontal_ranges() <= 1.25):
            while(self.lidar.get_frontal_ranges() <= 1.25):
                self.robot.move(0, 0.15)

        while(self.pillar_center[color_index] is None):

            # Rotate
            self.robot.move(0, 0.5)

            found_center = self.vision.get_center()

            if found_center[color_index] is not None:
                self.pillar_center[color_index] = np.abs(found_center[color_index])
                print(f'Found {self.colors[color_index]}')
                print(f'     in camera frame at x = {self.pillar_center[color_index]}')
                self.robot.move(0,0)

    def go_2(self, color_index):

        if(self.lidar.get_frontal_ranges() > 1.25):
            # while(self.get_distance() > 1.30):
            rate = rospy.Rate(50)
            while(self.lidar.get_frontal_ranges() > 1.25):
                # print(self.lidar.get_frontal_ranges())
            # while(not self.found[color_index]):
                # print(self.vision.get_center()[color_index])
                w = self.PI_angular_vel(self.vision.get_center()[color_index])
                self.robot.move(1, w)
                rate.sleep()

            self.robot.move(0,0)

    def search_for_pillars(self):
        """Spin one full rotation and identify each of the pillars. Yaw angle
           is logged when a pillar is identified."""

        yaw = 0.0
        print('SEEKING....')

        # While yaw angle is <= than 359 degrees
        while(yaw <= (2*np.pi - 5*np.pi/180)):
            roll, pitch, yaw = self.get_RPY_degrees()

            # Rotate
            self.robot.move(0, 0.75)

            found_center = self.vision.get_center()

            for i, center in enumerate(found_center):
                if center is not None:

                    # Initial conditions
                    if self.pillar_center[i] is None:
                        self.pillar_center[i] = np.abs(center)
                        self.center_yaw[i] = yaw

                    else:
                        # Find the yaw angle where the pillar is closest to the center of the camera frame
                        if np.abs(center) < self.pillar_center[i]:
                            self.pillar_center[i] = np.abs(center)
                            self.center_yaw[i] = yaw

        for i in range(3):
            print(f'Found {self.colors[i]}')
            print(f'     in camera frame at x = {self.pillar_center[i]}')
            print(f'     in world frame at orientation (0, 0, {self.center_yaw[i]})')

        # Stop moving
        self.robot.move(0,0)

    def go(self, angle, color_index):
        _, _, yaw = self.get_RPY_degrees()

        # Rotate to angle corresponding to the target pillar
        if angle is not None:
            while(np.abs(angle-yaw) > 1):
                _, _, yaw = self.get_RPY_degrees()
                self.robot.move(0, 0.5*(angle-yaw))

        # Stop rotating
        self.robot.move(0,0)

        # Go to within 1.2m of the pillar
        if(self.lidar.get_frontal_ranges() > 1.30):
            while(self.get_distance() > 1.30):
            # while(not self.found[color_index]):
                # print(self.vision.get_center()[color_index])
                w = self.PI_angular_vel(self.vision.get_center()[color_index])
                self.robot.move(1, w)

            print(f'found {self.colors[color_index]}')
            self.robot.move(0,0)

    def find_next(self, index):
        _, _, yaw = self.get_RPY_degrees()

        if angle is not None:
            while(np.abs(angle-yaw) > 1):
                _, _, yaw = self.get_RPY_degrees()
            self.robot.move(0,0)

    def get_RPY_degrees(self):
        robot_pos, robot_orientation = self.robot.get_pos_orientation()
        robot_quaternion = [robot_orientation.x, robot_orientation.y, robot_orientation.z, robot_orientation.w]
        roll, pitch, yaw = euler_from_quaternion(robot_quaternion)

        if yaw < 0:
            yaw = 2*np.pi+yaw

        return roll, pitch, yaw

    def get_distance(self):
        dist, _ = self.lidar.get_closest()

        return dist

    def run(self):
        # Give time for subscribers to receive messages
        rospy.sleep(2)

        # self.search_for_pillars()
        for i in [1, 2, 0]:
            self.search_2(i)
            self.go_2(i)


if __name__ == '__main__':
    brain = Brain()
