#!/usr/bin/env python3

import rospy
import numpy as np
from cmd_vision import Vision
from cmd_control import vel_control
from control import PI_controller
from lidar_new import Lidar


class Brain():

    def __init__(self):

        rospy.init_node('move', anonymous=True)

        self.vision = Vision()
        self.move   = vel_control()
        self.PID    = PI_controller()
        self.lidar  = Lidar()

        
