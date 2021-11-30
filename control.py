#!/usr/bin/env python3

import rospy
from cmd_vision import Vision
from cmd_control import vel_control

vis = Vision()
robot = vel_control()

_, width, height = vis.camera_properties()

rospy.init_node('move', anonymous=True)

k_p = 1
k_i = 1
dt = 0.25



def error_func(center_x_val):
    dest = width/2
    err = (dest - center_x_val)/dest
    return err

while(True):
    err = error_func(vis.get_center())
    robot.move(0.5, k_p*err)
