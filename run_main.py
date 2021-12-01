#!/usr/bin/env python3

import rospy
import time
import numpy as np
from cmd_vision import Vision
from cmd_control import vel_control
from control import PI_controller


# Indexing constants
RED, GREEN, BLUE = 0, 1, 2

def run():
    rospy.init_node('move', anonymous=True)

    vis   = Vision()
    robot = vel_control()

    _, width, height = vis.camera_properties()

    # Set controller to try to maintain pillar within the center of the screen
    # and limit the output from -1.5 to 1.5 rad/sec
    PI_angular_vel = PI_controller(Kp=2/width,
                                Ki=1e-7,
                                setpoint=width/2,
                                output_constraints=(-1.5, 1.5))

    # Set linear velocity controller to attempt to get the calculated area
    # of the pillar contour area to 100 with speed between 0 and 5 m/s
    PI_linear_vel  = PI_controller(setpoint=100,
                                   output_constraints=(0,0.5))

    p_red = np.empty(1)
    p_green = np.empty(1)
    p_blue = np.empty(1)


    ref_time = time.time()
    i = 0
    cent = np.array(3)

    while(time.time() - ref_time <= 4*np.pi):

        robot.seek()
        # print(vis.get_center()[1])
        if vis.get_center()[i] is not None:
            print('in')
            if vis.get_center()[i] <= 20 and vis.get_center()[i] >= -20:
                if robot.orientation is not None:
                    # cent[i] = robot.orientation.z
                    # print(robot.orientation.z)
                    print(f'found {i}')

                    i += 1
        # np.append(p_red, vis.get_pillar_areas()[0])
        # np.append(p_blue, vis.get_pillar_areas()[1])
        # np.append(p_green, vis.get_pillar_areas()[2])

        # print(vis.get_pillar_areas())
    # maxes = np.array(max(p_red),max(p_blue),max(p_green))

    order = [RED, GREEN, BLUE]

    r = rospy.Rate(10)  # 10Hz loop
    ref_time = time.time()
    while(True):

        if(time.time() - ref_time <= 4*np.pi):
            robot.move(-1.0,0)
            if len(order)>1:
                order = order[1:]

        print(order[0])
        # ordered list = (green, red, blue)
        # if at pillar: remove color from list
        w = PI_angular_vel(vis.get_center()[order[0]])
        # print(w)
        # v = PI_linear_vel(vis.get_pillar_area())  #Not implemented yet
        # robot.move(bias + v, w)
        robot.move(0.5, w)

        r.sleep()

if __name__ == '__main__':
    run()
