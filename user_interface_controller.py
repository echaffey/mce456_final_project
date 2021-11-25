#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

PI = 3.14159265359


class movement:
    def __init__(self):
        rospy.init_node('user_interface_node', anonymous=False)
        self.pub_movement = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.movement = Twist()

    def publish_vel(self):
        self.pub_movement.publish(self.movement)

    def move_forward(self):
        self.movement.linear.x = 0.5
        self.movement.angular.z = 0.0

    def move_backward(self):
        self.movement.linear.x = -0.5
        self.movement.angular.z = 0.0

    def stop(self):
        self.movement.linear.x = 0.0
        self.movement.angular.z = 0.0

    def turn_left(self):
        self.movement.linear.x = 0.0
        self.movement.angular.z = 0.5

    def turn_right(self):
        self.movement.linear.x = 0.0
        self.movement.angular.z = -0.5


if __name__ == "__main__":
    mov = movement()
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        movement = input('Enter desired movement (forward, backward, left, right, stop): ')

        if movement == 'forward':
            mov.move_forward()

        if movement == 'backward':
            mov.move_backward()

        if movement == 'left':
            mov.turn_left()

        if movement == 'right':
            mov.turn_right()

        if movement == 'stop':
            mov.stop()

        mov.publish_vel()
        rate.sleep()
