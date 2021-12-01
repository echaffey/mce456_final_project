#!/usr/bin/env python3

import rospy
from cmd_vision import Vision
from cmd_control import vel_control
from control import PI_control


rospy.init_node('move', anonymous=True)

vis   = Vision()
robot = vel_control()

_, width, height = vis.camera_properties()

# Set controller to try to maintain pillar within the center of the screen
# and limit the output from -1.5 to 1.5 rad/sec
PI_angular_vel = PI_control(Kp=2/width,
                            Ki=1e-7, 
                            setpoint=width/2,
                            output_constraints=(-1.5, 1.5))

# Set linear velocity controller to attempt to get the calculated area
# of the pillar contour area to 100 with speed between 0 and 5 m/s
PI_linear_vel  = PI_control(setpoint=100, output_constraints=(0,0.5))

# k_p = 1
# k_i = 1
# dt = 0.5

r = rospy.Rate(10)  # 5Hz loop
while(True):

    w = PI_angular_vel(vis.get_center())
    # print(w)
    # v = PI_linear_vel(vis.get_pillar_area())  #Not implemented yet
    # robot.move(bias + v, w)
    robot.move(0.5, w)

    r.sleep()
