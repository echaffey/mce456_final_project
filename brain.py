#!/usr/bin/env python3

import rospy
import numpy as np
from cmd_vision import Vision
from cmd_control import vel_control
from control import PI_controller
from lidar_new import Lidar

import numpy as np


class Brain():

    def __init__(self):

        rospy.init_node('move', anonymous=True)

        self.vision = Vision()
        self.robot   = vel_control()
        self.PID    = PI_controller()
        self.lidar  = Lidar()


    def search_for_pillars(self):

        robot_pos, robot_orientation = robot.get_pos_orientation()

        while(robot_orientation[2]<2*np.pi):
            robot.move(0, 0.5)
