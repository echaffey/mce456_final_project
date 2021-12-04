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

        self.vision = Vision()
        self.robot  = vel_control()
        self.PID    = PI_controller()
        self.lidar  = Lidar()

        self.center_yaw = [None, None, None]

        self.run()


    def search_for_pillars(self):

        yaw = 0.0

        # While yaw angle is <= than 359 degrees
        while(yaw <= (2*np.pi - np.pi/180)):
            # Get orientation in quaternions and convert to Euler coords
            robot_pos, robot_orientation = self.robot.get_pos_orientation()
            robot_quaternion = [robot_orientation.x, robot_orientation.y, robot_orientation.z, robot_orientation.w]
            roll, pitch, yaw = euler_from_quaternion(robot_quaternion)

            # Rotate
            self.robot.move(0, 0.5)

            # print(self.vision.get_center()[0])
            for i in range(len(self.vision.get_center())):
                if(self.vision.get_center()[i] is not None):
                    # if center of pillar is near the center of the image frame
                    if(self.vision.get_center()[i] <= 100 and self.vision.get_center()[i] >= -100):
                        self.center_yaw[i] = yaw
                        break

            # Convert angle measure to 0 to 360 degrees
            if yaw < 0:
                yaw = 2*np.pi+yaw
                # print(yaw)

        print(self.center_yaw)
        self.robot.move(0,0)

    def go(self):
        while(self.get_distance() > 1.25)

    def get_distance(self):
        print(self.lidar.get_closest())

    def run(self):
        rospy.sleep(2)
        self.search_for_pillars()

if __name__ == '__main__':
    brain = Brain()
